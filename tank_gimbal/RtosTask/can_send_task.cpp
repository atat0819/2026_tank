/**
 * @file can_send_task.cpp
 * @brief 云台通信、输入调度及 Yaw/Pitch 闭环控制任务。
 *
 * 控制顺序：读取输入和反馈 -> 在线保护 -> 故障恢复锚定 -> 两轴 FSM 更新
 *           -> 目标限位检查 -> 控制器复位 -> PID/LQR 与补偿
 *           -> +/-9 N*m 幅值限制 -> FSM 向外力矩拦截 -> MIT 指令发送。
 *
 * 两轴正常使用 IMU，IMU 异常降级为编码器；角为 rad、速度为 rad/s、力矩为 N*m。
 * 两轴限位始终使用原始电机编码器，与 IMU/编码器闭环来源无关。
 * 编码器到边界后向外目标收回到当前控制角，全部补偿叠加后再拦截向外力矩。
 * 本任务更新两轴云台 FSM；摩擦轮、拨弹控制由其他任务中的对应 FSM 负责。
 */
#include "can_send_task.hpp"
#include "remote_control_task.hpp"
#include "boards_communication.hpp"
#include "../communication_between_boards/boards_communication_uart_485.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "gimbal_task.hpp"
#include "usart.h"
#include "usbd_cdc_if.h"
#include "../feeder_fsm/gimbal_fsm.hpp"
#include "../feeder_fsm/motor_recovery_fsm.hpp"
#include "../feeder_fsm/imu_feedback_fsm.hpp"
#include "../communication_between_boards/input_dispatcher.hpp"
#include "../user/core/Alg/Feedforward/Feedforward.hpp"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include "../user/core/BSP/Motor/DM/DmMotor.hpp"
#include "../YAW_Auto_Controller-main/yaw_auto_lqr_eso_controller.h"
#include <string.h>

extern "C" USBD_HandleTypeDef hUsbDeviceHS;

namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float DEG_TO_RAD = kPi / 180.0f;
constexpr float YAW_MAX_UNWRAP_STEP_RAD = 45.0f * DEG_TO_RAD; // 相邻有效 Yaw 样本最大接受增量；不是行程限位
constexpr float J4340_TORQUE_LIMIT_NM = 9.0f; // 两轴最终 MIT 力矩幅值上限，N*m
constexpr float PITCH_ENCODER_HORIZONTAL_RAD = 46.7f * DEG_TO_RAD; // 实测水平时 ID2 原始编码器角

// 外部 IMU/编码器的度数和配置中的角度数值，进入控制计算前统一转换为弧度。
float DegreesToRadians(float value)
{
    return value * DEG_TO_RAD;
}

float AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/**
 * @brief 将有界 IMU Yaw 展开为连续角，避免跨越 +/-pi 时产生近 2pi 的假误差。
 * @param raw_yaw IMU 原始 Yaw，rad；force_reset 在启动重新建立展开基准时使用。
 * @return 累计后的连续 Yaw，rad；该角用于姿态闭环，不用作编码器行程限位。
 */
float GetContinuousYawAngle(float raw_yaw, bool force_reset = false)
{
    static bool initialized = false;
    static float last_raw_yaw = 0.0f;
    static float continuous_yaw = 0.0f;

    if (!initialized || force_reset)
    {
        initialized = true;
        last_raw_yaw = raw_yaw;
        continuous_yaw = raw_yaw;
        return continuous_yaw;
    }

    float delta = raw_yaw - last_raw_yaw;
    while (delta > kPi)
    {
        delta -= kTwoPi;
    }
    while (delta < -kPi)
    {
        delta += kTwoPi;
    }

    if (AbsFloat(delta) > YAW_MAX_UNWRAP_STEP_RAD)
    {
        // 拒绝本次异常跳变，连续角和上一有效原始角均保持；后续仍与上一有效样本比较。
        return continuous_yaw;
    }

    continuous_yaw += delta;
    last_raw_yaw = raw_yaw;
    return continuous_yaw;
}
}


/* 注：原 bxCAN 时代的 txDataBuffer / rxDataBuffer0 / rxDataBuffer1 / txHeader /
   rxHeader0 / rxHeader1 已删除 —— H7 的 FDCAN 收发缓冲区由 HAL::FDCAN 内部管理。*/

/******************************************************************************** */
// ============ 角度双环（普通角度模式：外环角度PID → 目标速度，内环速度PID → 扭矩） ============
// PID 构造参数依次为 Kp、Ki、Kd、输出幅值上限、积分状态上限、积分分离误差阈值。
// 普通角度/速度环按 IMU、编码器独立配置；编码器初值复制原参数，后续分别整定。
// 外环输入角度误差(rad)，输出目标速度(rad/s)；内环输入速度误差(rad/s)，输出力矩(N*m)。
// PID 自身的输出上限不等于允许电机力矩；发送前仍统一夹到 +/-9 N*m 并检查方向限位。
// yaw轴角度环外环PID
ALG::PID::PID yaw_angle_imu_pid(19.5f, 0.01f, 0.0f, 5000.0f, 1000.0f, 100.0f);
// 编码器角度外环独立调参和保存状态；初值沿用 IMU 参数，待实机整定。
ALG::PID::PID yaw_angle_encoder_pid(19.5f, 0.01f, 0.0f, 5000.0f, 1000.0f, 100.0f);
// yaw轴角度环内环PID
ALG::PID::PID yaw_angle_to_speed_imu_pid(4.8f, 0.01f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID yaw_angle_to_speed_encoder_pid(4.8f, 0.01f, 0.0f, 5000.0f, 1000.0f, 100.0f);
// pitch轴角度环外环PID
ALG::PID::PID pitch_angle_imu_pid(32.0f,0.015f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID pitch_angle_encoder_pid(32.0f,0.015f, 0.0f, 5000.0f, 1000.0f, 100.0f);
// pitch轴角度环内环PID
ALG::PID::PID pitch_angle_to_speed_imu_pid(7.4f, 0.01f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID pitch_angle_to_speed_encoder_pid(7.4f, 0.01f, 0.0f, 5000.0f, 1000.0f, 100.0f);


// 键鼠角度双环与遥控器独立；每个环再按 IMU/编码器分开参数和积分/微分状态。
ALG::PID::PID yaw_keymouse_angle_imu_pid(18.0f, 0.0f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID yaw_keymouse_angle_encoder_pid(18.0f, 0.0f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID yaw_keymouse_angle_to_speed_imu_pid(4.7f, 0.0f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID yaw_keymouse_angle_to_speed_encoder_pid(4.7f, 0.0f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID pitch_keymouse_angle_imu_pid(26.5f, 0.0f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID pitch_keymouse_angle_encoder_pid(26.5f, 0.0f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID pitch_keymouse_angle_to_speed_imu_pid(6.0f, 0.01f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID pitch_keymouse_angle_to_speed_encoder_pid(6.0f, 0.01f, 0.0f, 5000.0f, 1000.0f, 100.0f);
// Pitch 重力前馈参数依次为补偿幅值(N*m)、相位(rad)；沿用原值，待实机标定。
Alg::Feedforward::Gravity pitch_gravity_ff(
    0.0f, 0.0f);
/********************************************************************************** */

// ============ 速度单环（保留的速度控制分支；当前 FSM 正常运行档位均映射为角度模式） ============
//yaw轴遥控器单速度环PID（继承原 yaw_single_speed_pid 参数，遥控器手感不变）
ALG::PID::PID yaw_remote_speed_imu_pid(4.3f, 0.05f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID yaw_remote_speed_encoder_pid(4.3f, 0.05f, 0.0f, 5000.0f, 1000.0f, 100.0f);
//pitch轴遥控器单速度环PID（继承原 pitch_single_speed_pid 参数，遥控器手感不变）
ALG::PID::PID pitch_remote_speed_imu_pid(4.0f, 0.04f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID pitch_remote_speed_encoder_pid(4.0f, 0.04f, 0.0f, 5000.0f, 1000.0f, 100.0f);
/********************************************************************************** */

// ============ 视觉模式（仅 IMU，编码器降级时视觉关闭；PID 和前馈状态独立） ============
// yaw 视觉角度环外环PID
ALG::PID::PID yaw_version_angle_imu_pid(16.5f, 0.01f, 0.01f, 2000.0f, 1000.0f, 100.0f);
// yaw 视觉角度环内环PID
ALG::PID::PID yaw_version_speed_imu_pid(1.54f, 0.0f, 0.0f, 5000.0f, 1000.0f, 100.0f);
// 开源脚本示例值，尚未针对本机标定；仅视觉 Yaw 使用。
static const YawAutoLqrEsoConfig_t yaw_auto_config = {
    0.039f, 0.30f, 12.0f, 3.4f, // J(kg*m^2)、B(N*m*s/rad)、角误差反馈增益、角速度误差反馈增益
    0.0f, 0.0f,                 // 角误差积分增益、积分状态上限(rad*s)；当前关闭积分补偿
    0.0f, 0.0f,                 // 库仑摩擦力矩(N*m)、平滑速度(rad/s)；当前摩擦幅值为 0
    20.0f, 0.2f, 1.0f,         // ESO 带宽(rad/s)、扰动补偿比例、补偿幅值上限(N*m)
    0.0f, 0.0f,                 // ESO 补偿的反馈速度(rad/s)、参考加速度(rad/s^2)门限；0 表示不设门限
    0.0f, 0.0f, 0.0f,          // 力矩偏置积分增益、偏置上限(N*m)、反馈力矩低通系数；当前未启用
    0.0f,                       // 角误差死区(rad)；当前不设置死区
    // 对称软限幅(N*m，0=关闭)、硬限幅下限/上限(N*m)、力矩变化率上限(N*m/s)。
    0.0f, -J4340_TORQUE_LIMIT_NM, J4340_TORQUE_LIMIT_NM, 0.0f,
    1U, 1U, 0U                 // 启用 ESO、启用 ESO 扰动补偿、关闭力矩变化率限制
};
static YawAutoLqrEso_t yaw_auto_ctrl;
// Pitch 视觉角度环外环 PID：视觉模式下使用，输出目标角速度(rad/s)。
ALG::PID::PID pitch_version_angle_imu_pid(17.5f, 0.025f, 0.0f, 2000.0f, 1000.0f, 100.0f);
// Pitch 视觉角度环内环 PID：速度误差转换为力矩(N*m)，再叠加视觉惯量前馈。
ALG::PID::PID pitch_version_speed_imu_pid(4.0f, 0.01f, 0.0f, 5000.0f, 1000.0f, 100.0f);
// Pitch 视觉模式保留有效惯量前馈，输出力矩(N*m)。
Alg::Feedforward::GimbalFullCompensation pitch_vision_dynamics_ff(
    0.01f, 0.001f, 0.0f, 0.0f);
/*********************************************************************** */

using Remote = BSP::REMOTE_CONTROL::RemoteController;


Struct_Gimbal_FSM_Config yaw_gimbal_fsm_config;   // yaw 轴的 FSM 配置结构体(限位、归一化等参数)
Struct_Gimbal_FSM_Config pitch_gimbal_fsm_config;  // pitch 轴的 FSM 配置结构体
Class_Gimbal_FSM yaw_gimbal_fsm;   // 定义 yaw 轴的 FSM 实例
Class_Gimbal_FSM pitch_gimbal_fsm;   // 定义 pitch 轴的 FSM 实例

DM4340_State_t dm4340_state[2]; // 下标 0=Yaw(ID1)，下标 1=Pitch(ID2)，保存电机反馈快照
RemoteData_t RemoteData;
IMU_t ImuData_user; // 仅缓存通过健康检查的 IMU 数据
static ImuFeedbackFSM imu_feedback_fsm;


float yaw_target_angle = 0.0f;   // FSM 给出的目标角(rad)，来源可为遥控、键鼠或视觉
float pitch_target_angle = 300.0f; // 目标角(rad)；此处为原有初值，启动时由 IMU 和 FSM 覆盖
float yaw_current_angle = 0.0f;   // 选定的 Yaw 闭环反馈(rad)：正常用 IMU，降级用编码器
float pitch_current_angle = 0.0f; // 选定的 Pitch 闭环反馈(rad)，两来源零点不同

float yaw_target_speed = 0.0f;   // 目标角速度(rad/s)：普通角度分支由 PID 产生，视觉分支取视觉参考
float pitch_target_speed = 0.0f; // 目标角速度(rad/s)：角度分支由外环 PID 产生
float yaw_current_speed = 0.0f;   // IMU 提供的 Yaw 控制角速度(rad/s)
float pitch_current_speed = 0.0f; // IMU 提供的 Pitch 控制角速度(rad/s)，当前读取陀螺仪通道 0

float filted_yaw = 0.0f; // 滤波后的角度
float filted_gyroz = 0.0f; // 滤波后的陀螺仪 z 轴角速度



uint8_t yaw_mode = GIMBAL_MODE_SPEED;
uint8_t pitch_mode = GIMBAL_MODE_SPEED;



// 2 个云台电机
// GM6020 反馈ID = 0x204 + 拨码值，拨码1→0x205，拨码2→0x206
//BSP::Motor::Dji::GM6020<2> gimbal_motor(0x204, gimbal_motor_idxs, 0x1FF);
BSP::Motor::DM::J4340<2> gimbal_motor(
    0,{7, 8},{7, 8},HAL::FDCAN::FdcanDeviceId::HAL_Fdcan3);

void ControlTask();
static bool UpdateGimbalFeedback(float &yaw_angle, float &yaw_speed, float &pitch_angle, float &pitch_speed);





extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t)
{
    // H7 上原来的 CAN1/CAN2 两个 bxCAN 接收回调合并为 FDCAN 的单个 RxFifo0 回调。
    // 按 Instance 找到对应设备后把该 FIFO 取空；每取到一帧，
    // receive() 内部会触发 register_rx_callback() 注册的解析器。
    HAL::FDCAN::Frame frame;
    HAL::FDCAN::IFdcanDevice *device = nullptr;

    if (hfdcan->Instance == FDCAN1)
    {
        device = &HAL::FDCAN::get_fdcan_bus_instance().get_fdcan1();
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        device = &HAL::FDCAN::get_fdcan_bus_instance().get_fdcan2();
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        device = &HAL::FDCAN::get_fdcan_bus_instance().get_fdcan3();
    }

    if (device != nullptr)
    {
        while (device->receive(frame))
        {
        }
    }
}

/* 本文件使用 H7 FDCAN 接口，不设置 bxCAN 的 CAN_MCR_ABOM。
   下方在线检查按电机反馈有效性停止输出；这里没有单独实现 FDCAN bus-off 恢复流程。 */

float yaw_error = 0.0f; // 目标减反馈(rad)，Yaw 使用连续角直接求差
float pitch_error = 0.0f; // 目标减反馈(rad)，计算后将误差归一化到 [-pi, pi]
float yaw_control_output = 0.0f; // Yaw 合成力矩(N*m)，发送前还需幅值与方向限位
float yaw_speed_pid_output = 0.0f;
float pitch_control_output = 0.0f; // Pitch 合成力矩(N*m)，包含 PID、重力及视觉惯量前馈
volatile uint8_t gimbal_remote_offline = 0; // 遥控数据无效标志：1=本任务判定失联/异常

// 按轴复位控制状态；单轴电机掉线不清除另一轴的 PID、观测器和输出。
static void ResetYawControlState()
{
    yaw_control_output = 0.0f;
    yaw_target_speed = 0.0f;
    yaw_speed_pid_output = 0.0f;
    yaw_angle_imu_pid.reset();
    yaw_angle_encoder_pid.reset();
    yaw_angle_to_speed_imu_pid.reset();
    yaw_angle_to_speed_encoder_pid.reset();
    yaw_keymouse_angle_imu_pid.reset();
    yaw_keymouse_angle_encoder_pid.reset();
    yaw_keymouse_angle_to_speed_imu_pid.reset();
    yaw_keymouse_angle_to_speed_encoder_pid.reset();
    yaw_remote_speed_imu_pid.reset();
    yaw_remote_speed_encoder_pid.reset();
    yaw_version_angle_imu_pid.reset();
    yaw_version_speed_imu_pid.reset();
    YawAutoLqrEso_Reset(&yaw_auto_ctrl, 0.0f, 0.0f);
}

static void ResetPitchControlState()
{
    pitch_control_output = 0.0f;
    pitch_target_speed = 0.0f;
    pitch_angle_imu_pid.reset();
    pitch_angle_encoder_pid.reset();
    pitch_angle_to_speed_imu_pid.reset();
    pitch_angle_to_speed_encoder_pid.reset();
    pitch_keymouse_angle_imu_pid.reset();
    pitch_keymouse_angle_encoder_pid.reset();
    pitch_keymouse_angle_to_speed_imu_pid.reset();
    pitch_keymouse_angle_to_speed_encoder_pid.reset();
    pitch_remote_speed_imu_pid.reset();
    pitch_remote_speed_encoder_pid.reset();
    pitch_version_angle_imu_pid.reset();
    pitch_version_speed_imu_pid.reset();
    pitch_vision_dynamics_ff.ResetState(0.0f);
}

// 遥控或 IMU 故障仍同时复位两轴；目标由恢复分支另行重新锚定。
static void ResetGimbalControlState()
{
    ResetYawControlState();
    ResetPitchControlState();
}

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6);
void vofa_sendN(const float *data, uint8_t count);


extern "C" void can_send_task(void *argument)
{
    // CAN 任务负责初始化总线、注册反馈回调，并周期性运行云台控制。
    // 延时 1000 个 CMSIS-OS tick，给其他任务和通信外设初始化留出时间。
    osDelay(1000);

/*********************************************************************************** */
// ==================== Yaw 状态机参数 ====================
// FSM 负责模式和目标生成，本任务负责闭环计算及发送；所有角度参数在此转换成 rad。
// angle_step 是“每次 FSM 更新、单位摇杆输入”的目标增量，不是独立的每秒速率。
// 摇杆目标累加公式：target_angle += joystick_speed * angle_step（通过死区判断后）。
yaw_gimbal_fsm_config.angle_step = DegreesToRadians(0.10f); // 单位摇杆每次累加 0.10 度；实际速率随更新周期变化
yaw_gimbal_fsm_config.speed_scale = DegreesToRadians(110.0f); // 保留速度分支的单位摇杆目标速度，110 度/s
// mouse_speed_scale 是预留鼠标速度比例(rad/s 每像素)，当前 FSM 尚未读取此成员。
// 当前运行档位为角度模式，键鼠目标实际使用下面的 mouse_angle_scale。
yaw_gimbal_fsm_config.mouse_speed_scale = DegreesToRadians(0.2f);   // 预留值：0.2 度/s 每像素
yaw_gimbal_fsm_config.mouse_angle_scale = DegreesToRadians(0.017f);  // 累计鼠标 X 位移每像素改变目标 0.017 度
// 上下限以电机原始编码器零点为基准；任务传入 getAngleDeg(1) 转换后的弧度。
// 这不是 IMU Yaw 的 +/-60 度目标范围，也不是发送到底盘的带偏置 Yaw 角范围。
yaw_gimbal_fsm_config.min_angle = DegreesToRadians(-60.0f); // 编码器 <= -60 度时禁止继续减小角度
yaw_gimbal_fsm_config.max_angle = DegreesToRadians(60.0f);  // 编码器 >= +60 度时禁止继续增大角度
yaw_gimbal_fsm_config.limit_angle = 1U; // 限位总开关：检查边界处的向外目标和最终向外力矩
yaw_gimbal_fsm_config.limit_target_angle = 0U; // 不夹紧 IMU 目标；其参考系与编码器限位不同
// normalize_angle=1 使视觉目标展开到最接近当前连续 IMU 角的圈数。
// continuous_angle=1 保留累计圈数，普通目标不被归一化回 [-pi, pi)。
yaw_gimbal_fsm_config.normalize_angle = 1U; // 启用视觉目标与连续控制角的圈数对齐
yaw_gimbal_fsm_config.continuous_angle = 1U; // 保持连续 Yaw 目标，避免过 +/-pi 时目标跳变
yaw_gimbal_fsm.Init(yaw_gimbal_fsm_config, GIMBAL_STATUS_STOP); // 复制配置并清状态，初始零力矩 STOP

// ==================== Pitch 状态机参数 ====================
// Pitch 正常闭环用 IMU，降级闭环用编码器；机械限位始终取 ID2 原始编码器。
// 上下限按用户提供的原始编码器角设置，不作为 IMU 目标角的夹紧范围。
pitch_gimbal_fsm_config.angle_step = DegreesToRadians(0.11f); // 单位摇杆每次累加 0.11 度
pitch_gimbal_fsm_config.speed_scale = DegreesToRadians(95.0f); // 保留速度分支的单位摇杆目标速度，95 度/s
pitch_gimbal_fsm_config.mouse_speed_scale = DegreesToRadians(0.2f);   // 预留鼠标速度比例，当前 FSM 未读取
pitch_gimbal_fsm_config.mouse_angle_scale = DegreesToRadians(0.013f);  // 累计鼠标 Y 位移每像素改变目标 0.013 度
pitch_gimbal_fsm_config.min_angle = DegreesToRadians(34.2f);    // 原始编码器 <= 34.2 度时禁止继续减小角度
pitch_gimbal_fsm_config.max_angle = DegreesToRadians(67.9f);    // 原始编码器 >= 67.9 度时禁止继续增大角度
pitch_gimbal_fsm_config.limit_angle = 1U; // 限位总开关；实际到边界后另拦截最终向外力矩
pitch_gimbal_fsm_config.limit_upper_torque = 0U; // 保留上限目标保护，允许正力矩支撑重力/制动下降；下限仍拦负力矩
// 与 Yaw 相同：不夹紧绝对目标角，避免混用 IMU 目标和编码器机械行程。
// 到机械边界后，Update_Angle_Limit 收回向外目标，最终 Limit_Torque 拦截向外力矩。
pitch_gimbal_fsm_config.limit_target_angle = 0U; // 遥控、键鼠、视觉目标均使用边界目标保护
pitch_gimbal_fsm_config.normalize_angle = 0U; // Pitch 不做 Yaw 式圈数对齐/角度归一化
pitch_gimbal_fsm_config.continuous_angle = 0U; // 配合上面的归一化关闭，不使用连续圈数展开
pitch_gimbal_fsm.Init(pitch_gimbal_fsm_config, GIMBAL_STATUS_STOP); // 初始停止，运行时由输入决定状态
/************************************************************************************* */

	static uint32_t can2_tick = 0; // 有效遥控分支中的调度拍数，用于 VOFA 分频，不是毫秒时间戳
static bool remote_was_offline = false; // 遥控曾异常：恢复时用选定反馈角重新建立目标
static bool yaw_motor_was_offline = false; // Yaw 恢复后只重新锚定自身
static bool pitch_motor_was_offline = false; // Pitch 恢复后只重新锚定自身

static uint16_t startup_protect = 0; // 遥控有效且至少一轴在线时累计的启动保护次数

	// --- 在进入循环前初始化 ---
    // 这行代码执行时会触发 HAL_FDCAN_Start 和开启接收中断     
	HAL::FDCAN::get_fdcan_bus_instance(); // 触发 FDCAN bus 初始化：HAL_FDCAN_Start + 激活中断通知
	    auto &fdcan1 = HAL::FDCAN::get_fdcan_bus_instance().get_device(HAL::FDCAN::FdcanDeviceId::HAL_Fdcan1);
    auto &fdcan3 = HAL::FDCAN::get_fdcan_bus_instance().get_device(HAL::FDCAN::FdcanDeviceId::HAL_Fdcan3);
    // FDCAN1：本回调仅将 0x201~0x204 帧交给 friction_motor 的解析器。
    fdcan1.register_rx_callback([](const HAL::FDCAN::Frame &frame) {
    if (frame.id >= 0x201 && frame.id <= 0x204)
    {
        friction_motor.Parse(frame);
    }
    });

    // 裁判转发已改为 USART3，接收 lambda 在 remote_control_task 中注册。

    // FDCAN3：两台 J4340 云台电机反馈，Yaw=1，Pitch=2。
    fdcan3.register_rx_callback([](const HAL::FDCAN::Frame &frame) {
        if (frame.id == 7 || frame.id == 8)
        {
            gimbal_motor.Parse(frame);
        }
    });


// 等待摇杆不再同时为 -1，最多 200 次、每次延时 1 tick；连接状态仍由主循环检查。
for (uint32_t wait = 0; wait < 200; wait++)
{
    vTaskDelay(1);
    if (!(remoteController.get_left_y() == -1
        && remoteController.get_left_x() == -1
        && remoteController.get_right_x() == -1
        && remoteController.get_right_y() == -1))
    {
        break;  // 遥控器数据有效了，提前退出
    }
}

// 初始化视觉 Yaw 控制器并读取当前缓存的电机反馈；在线有效性由主循环另行检查。
YawAutoLqrEso_Init(&yaw_auto_ctrl);
ControlTask();

// 初始目标先用编码器；主循环验证 IMU 后切换来源并重新锚定。
yaw_target_angle = gimbal_motor.getAngleRad(1);
pitch_target_angle = gimbal_motor.getAngleRad(2);

// 最后才使能电机
MotorRecoveryFSM gimbal_recovery_fsm[2]; // 1=Yaw、2=Pitch，独立重试使能
const uint32_t motor_init_tick = HAL_GetTick();
gimbal_recovery_fsm[0].Init(motor_init_tick);
gimbal_recovery_fsm[1].Init(motor_init_tick);
gimbal_motor.On(1, BSP::Motor::DM::MIT); // Yaw, FDCAN3
gimbal_motor.On(2, BSP::Motor::DM::MIT); // Pitch, FDCAN3
vTaskDelay(500); // 等电机上电完成

// 等待 IMU 数据就绪并稳定，防止上电初期 IMU 未收敛导致猛转
// 第一步：等 IMU 板开始发送有效数据（最多等 1000ms）
for (uint32_t i = 0; i < 1000; i++)
{
    if (imu.isConnected()) break;
    vTaskDelay(1);
}

// 第二步：仅检查相邻 Yaw 样本变化，连续 10 次小于 1 度或满 200 次后结束等待。
// 这是变化量稳定判据，不是 IMU 精度校准，也不代表 Pitch 或偏置已经验证。
{
    float prev_yaw = imu.GetAngle(2);
    uint8_t stable_count = 0;
    for (uint32_t i = 0; i < 200; i++)
    {
        vTaskDelay(1);
        float cur_yaw = imu.GetAngle(2);
        float diff = cur_yaw - prev_yaw;
        if (diff > 180.0f)  diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;
        if (diff < 1.0f && diff > -1.0f)
        {
            stable_count++;
            if (stable_count >= 10) break;
        }
        else
        {
            stable_count = 0;
        }
        prev_yaw = cur_yaw;
    }
}

// 等待结束后更新编码器初始目标；未验证的 IMU 不进入连续角计算。
yaw_target_angle = gimbal_motor.getAngleRad(1);
pitch_target_angle = gimbal_motor.getAngleRad(2);

    // 主循环：采集输入、更新状态机、计算控制量并发送 CAN。
    for(;;)
    {
        // 遥控失联期间也检查反馈，连续健康计时不能包含未检查的区间。
        // 先验证原始值，再展开 IMU Yaw，避免 NaN/Inf 进入控制计算。
        const bool feedback_source_changed = UpdateGimbalFeedback(
            yaw_current_angle, yaw_current_speed, pitch_current_angle, pitch_current_speed);
        const bool using_encoder = imu_feedback_fsm.UsingEncoder();
         // 一次性消费此前收到的鼠标累计位移，避免控制任务多次累加同一帧。
         int32_t mouse_delta_x = 0;
         int32_t mouse_delta_y = 0;
         remoteController.ConsumeMouseDelta(mouse_delta_x, mouse_delta_y);

         if(!remoteController.isConnected() ||   // 遥控器失联
            (remoteController.get_left_y() == -1 // 或摇杆数据全-1（解析异常，数据不可信）
         && remoteController.get_left_x() == -1
         && remoteController.get_right_x() == -1
         && remoteController.get_right_y() == -1)
        )
        {
            gimbal_remote_offline = 1;
            // 遥控安全门期间不使能，恢复后的首次正常检查补发 On。
            gimbal_recovery_fsm[0].Force_Enable_On_Next_Check();
            gimbal_recovery_fsm[1].Force_Enable_On_Next_Check();
            remote_was_offline = true;
            RemoteData = {};
            // 遥控失联仍发送完整快照，清零操作量，阻止底盘沿用旧速度或键盘。
            RS485_SendBoardData(0.0f, 0.0f, 0U, 0U,
                                imu.isConnected() && std::isfinite(imu.GetAngle(2)) ? imu.GetAngle(2) : 0.0f,
                                imu.isConnected() && std::isfinite(imu.GetGyro(2)) ? imu.GetGyro(2) : 0.0f, 0U);

            ResetGimbalControlState();
            gimbal_motor.ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // Yaw
            gimbal_motor.ctrl_Mit(2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // Pitch

            vTaskDelay(1);
            continue;
        }
         else {
        gimbal_remote_offline = 0;
                 RemoteData.chassis_vx = remoteController.get_left_y();
        RemoteData.chassis_vy = remoteController.get_left_x();
        RemoteData.gimbal_yaw = -remoteController.get_right_x();
        RemoteData.gimbal_pitch = remoteController.get_right_y();
        RemoteData.s1 = remoteController.get_s1();
        RemoteData.s2 = remoteController.get_s2();

        // [新增] 填充键鼠数据字段
        RemoteData.mouse_x    = remoteController.get_mouseX();
        RemoteData.mouse_y    = remoteController.get_mouseY();
        RemoteData.mouse_z    = remoteController.get_mouseZ();
        RemoteData.mouse_left = remoteController.get_mouseLeft() ? 1 : 0;
        RemoteData.mouse_right = remoteController.get_mouseRight() ? 1 : 0;
        RemoteData.keyboard   = remoteController.get_keyboard();

         // 输入调度 FSM：双中选择键鼠，其余档位选择遥控器；维护按键消抖及视觉右键状态。
         // 云台 FSM 随后结合视觉是否就绪/新鲜决定是否进入视觉控制。
        input_dispatcher.Update(
            RemoteData.s1,
            RemoteData.s2,
            RemoteData.keyboard,
            RemoteData.mouse_left != 0,
            RemoteData.mouse_right != 0
        );

        if (input_dispatcher.GetSource() != InputSource::KeyMouse)
        {
            // 遥控器档丢弃鼠标累计量，避免切到键鼠后继承此前移动。
            mouse_delta_x = 0;
            mouse_delta_y = 0;
        }




        
/******************************************************************** */
// 每轮提交最新快照，收到回复后即可发送；无回复最多等待 2ms，未使用字段归零。
// IMU 原始接口为度和度/秒，不使用上方供控制器计算的连续角(rad)。
const bool chassis_remote_mode = input_dispatcher.GetSource() == InputSource::Remote;
const bool chassis_keymouse_mode = input_dispatcher.GetSource() == InputSource::KeyMouse;
RS485_SendBoardData(chassis_remote_mode ? RemoteData.chassis_vx : 0.0f,
                    chassis_remote_mode ? RemoteData.chassis_vy : 0.0f,
                    RemoteData.s1, RemoteData.s2,
                    imu.isConnected() && std::isfinite(imu.GetAngle(2)) ? imu.GetAngle(2) : 0.0f,
                    imu.isConnected() && std::isfinite(imu.GetGyro(2)) ? imu.GetGyro(2) : 0.0f,
                    chassis_keymouse_mode ? input_dispatcher.GetKeyboardMask() : 0U);

 // tick 用于分频发送和周期任务调度。
 can2_tick++;/************************************************************************************** */

/************************************************************************************** */
       ControlTask(); // 读取2个电机的数据

   // 电机反馈在线状态独立判断；掉线轴停止，在线轴继续控制。
   // isConnected 的第一个参数为逻辑状态槽，第二个参数为蜂鸣器标识，不是另一 CAN ID。
  const bool yaw_motor_online = gimbal_motor.isConnected(1, 1);
  const bool pitch_motor_online = gimbal_motor.isConnected(2, 2);
  // 电机恢复使能：双下停车期间不发 On，退出后重新请求一次。
  const bool motor_recovery_paused =
      input_dispatcher.GetSource() != InputSource::KeyMouse &&
      RemoteData.s1 == Remote::DOWN && RemoteData.s2 == Remote::DOWN;
  if (motor_recovery_paused)
  {
      gimbal_recovery_fsm[0].Force_Enable_On_Next_Check();
      gimbal_recovery_fsm[1].Force_Enable_On_Next_Check();
  }
  else
  {
      // 先检查恢复请求，离线轴也需要周期性重试使能。
      const uint32_t motor_recovery_tick = HAL_GetTick();
      if (gimbal_recovery_fsm[0].Should_Enable(yaw_motor_online, motor_recovery_tick))
          gimbal_motor.On(1, BSP::Motor::DM::MIT);
      if (gimbal_recovery_fsm[1].Should_Enable(pitch_motor_online, motor_recovery_tick))
          gimbal_motor.On(2, BSP::Motor::DM::MIT);
  }
  // 电机掉线按轴处理：只复位掉线轴，后续向该轴传入 STOP 并发送零力矩。
  if (!yaw_motor_online)
  {
      yaw_motor_was_offline = true;
      ResetYawControlState();
  }
  if (!pitch_motor_online)
  {
      pitch_motor_was_offline = true;
      ResetPitchControlState();
  }

  if (feedback_source_changed)
  {
      yaw_gimbal_fsm.ReAnchor(yaw_current_angle);
      pitch_gimbal_fsm.ReAnchor(pitch_current_angle);
      ResetGimbalControlState();
  }

   // 电机恢复按轴处理：按选定反馈锚定恢复轴，保留另一轴的目标和控制状态。
  if (yaw_motor_online && yaw_motor_was_offline)
  {
      yaw_gimbal_fsm.ReAnchor(yaw_current_angle);
      ResetYawControlState();
      yaw_motor_was_offline = false;
  }
  if (pitch_motor_online && pitch_motor_was_offline)
  {
      pitch_gimbal_fsm.ReAnchor(pitch_current_angle);
      ResetPitchControlState();
      pitch_motor_was_offline = false;
  }

   // 遥控恢复：丢弃旧操作对应的目标偏差，锚定到选定来源的最新角。
  if (remote_was_offline)
  {
      yaw_gimbal_fsm.ReAnchor(yaw_current_angle);
      pitch_gimbal_fsm.ReAnchor(pitch_current_angle);
      ResetGimbalControlState();
      remote_was_offline = false;
  }

/************************************************************************************** */
//判断为何种模式

if (startup_protect < 500)
{
    // 前 500 次至少一轴在线的有效更新强制 STOP；两轴均掉线不累计。
    if (yaw_motor_online || pitch_motor_online) startup_protect++;
    Struct_Gimbal_Input stop_input = {};
    stop_input.s1 = Remote::DOWN;
    stop_input.s2 = Remote::DOWN;
    yaw_gimbal_fsm.Update(stop_input, yaw_current_angle);
    pitch_gimbal_fsm.Update(stop_input, pitch_current_angle);
}
else
{
    /************************************************************************************* */
    // FSM 内部自行判断模式：传入原始输入，FSM 根据 s1/s2/键鼠 自行决定
    // 遥控双下为 STOP，双上且视觉有效为 VISION，其他有效运行档位为 ANGLE。
    // 键鼠视觉需输入调度器确认右键视觉状态，并且 vision_ready、vision_fresh 均成立。
    // 角度模式会累加摇杆/累计鼠标位移；视觉模式使用视觉目标，切模式/输入源时重新锚定。
    Struct_Gimbal_Input yaw_input = {};
    yaw_input.s1       = RemoteData.s1;
    yaw_input.s2       = RemoteData.s2;
    yaw_input.is_keymouse = (input_dispatcher.GetSource() == InputSource::KeyMouse);
    yaw_input.joystick_speed = RemoteData.gimbal_yaw;
    yaw_input.mouse_speed = -static_cast<float>(RemoteData.mouse_x);
    yaw_input.mouse_angle_delta = -static_cast<float>(mouse_delta_x); // Yaw 按控制方向取负，再由 FSM 乘像素角度比例
    yaw_input.mouse_dx = RemoteData.mouse_x;
    yaw_input.mouse_dy = RemoteData.mouse_y;
    yaw_input.mouse_right_held = input_dispatcher.IsVisionMode();
    yaw_input.vision_ready = !using_encoder && vision_comm.IsVisionReady();
    yaw_input.vision_fresh = !using_encoder && vision_comm.IsDataFresh();
    yaw_input.vision_angle  = vision_comm.GetYawAngle();
    if (!yaw_motor_online)
    {
        yaw_input = {};
        yaw_input.s1 = Remote::DOWN;
        yaw_input.s2 = Remote::DOWN;
    }
    yaw_gimbal_fsm.Update(yaw_input, yaw_current_angle);

    Struct_Gimbal_Input pitch_input = {};
    pitch_input.s1       = RemoteData.s1;
    pitch_input.s2       = RemoteData.s2;
    pitch_input.is_keymouse = (input_dispatcher.GetSource() == InputSource::KeyMouse);
    pitch_input.joystick_speed = RemoteData.gimbal_pitch;
    pitch_input.mouse_speed = static_cast<float>(RemoteData.mouse_y);
    pitch_input.mouse_angle_delta = static_cast<float>(mouse_delta_y); // Pitch 按现有方向映射使用正 Y 累计位移
    pitch_input.mouse_dx = RemoteData.mouse_x;
    pitch_input.mouse_dy = RemoteData.mouse_y;
    pitch_input.mouse_right_held = input_dispatcher.IsVisionMode();
    pitch_input.vision_ready = !using_encoder && vision_comm.IsVisionReady();
    pitch_input.vision_fresh = !using_encoder && vision_comm.IsDataFresh();
    pitch_input.vision_angle  = vision_comm.GetPitchAngle();
    if (!pitch_motor_online)
    {
        pitch_input = {};
        pitch_input.s1 = Remote::DOWN;
        pitch_input.s2 = Remote::DOWN;
    }
    pitch_gimbal_fsm.Update(pitch_input, pitch_current_angle);
}
yaw_mode   = yaw_gimbal_fsm.Get_Mode_Command();
pitch_mode = pitch_gimbal_fsm.Get_Mode_Command();

// 两轴在线时均用原始编码器更新机械限位，两种闭环下始终有效。
// 第一个角用于与编码器 min/max 比较；第二个角是当前选定闭环来源的反馈角。
// 到机械边界且目标仍向外时，收回到当前控制角；最终再拦截向外力矩。
if (yaw_motor_online)
    yaw_gimbal_fsm.Update_Angle_Limit(DegreesToRadians(gimbal_motor.getAngleDeg(1)), yaw_current_angle);
if (pitch_motor_online)
    pitch_gimbal_fsm.Update_Angle_Limit(DegreesToRadians(gimbal_motor.getAngleDeg(2)), pitch_current_angle);
const uint8_t yaw_mode_changed = yaw_gimbal_fsm.Take_Mode_Changed_Flag();
const uint8_t yaw_limit_reset = yaw_gimbal_fsm.Take_Limit_Reset_Flag();
const uint8_t pitch_mode_changed = pitch_gimbal_fsm.Take_Mode_Changed_Flag();
const uint8_t pitch_limit_reset = pitch_gimbal_fsm.Take_Limit_Reset_Flag();

// Take_* 返回后清除标志：模式/输入源切换或目标保护引起的复位在本周期控制计算前处理。
// 本周期末 Limit_Torque 产生的新标志则留待下一周期处理，避免旧积分和观测状态继续向外推。
if (yaw_mode_changed != 0U || yaw_limit_reset != 0U)
{
    yaw_angle_imu_pid.reset();
    yaw_angle_encoder_pid.reset();
    yaw_angle_to_speed_imu_pid.reset();
    yaw_angle_to_speed_encoder_pid.reset();
    yaw_keymouse_angle_imu_pid.reset();
    yaw_keymouse_angle_encoder_pid.reset();
    yaw_keymouse_angle_to_speed_imu_pid.reset();
    yaw_keymouse_angle_to_speed_encoder_pid.reset();
    yaw_remote_speed_imu_pid.reset();
    yaw_remote_speed_encoder_pid.reset();
    yaw_version_angle_imu_pid.reset();
    yaw_version_speed_imu_pid.reset();
    YawAutoLqrEso_Reset(&yaw_auto_ctrl, yaw_current_angle, yaw_current_speed);
}

if (pitch_mode_changed != 0U || pitch_limit_reset != 0U)
{
    pitch_angle_imu_pid.reset();
    pitch_angle_encoder_pid.reset();
    pitch_angle_to_speed_imu_pid.reset();
    pitch_angle_to_speed_encoder_pid.reset();
    pitch_keymouse_angle_imu_pid.reset();
    pitch_keymouse_angle_encoder_pid.reset();
    pitch_keymouse_angle_to_speed_imu_pid.reset();
    pitch_keymouse_angle_to_speed_encoder_pid.reset();
    pitch_remote_speed_imu_pid.reset();
    pitch_remote_speed_encoder_pid.reset();
    pitch_version_angle_imu_pid.reset();
    pitch_version_speed_imu_pid.reset();
    pitch_vision_dynamics_ff.ResetState(0.0f);
}

// 完成目标保护后再取目标，确保 PID/LQR 不使用本周期已经被 FSM 撤回的向外目标。
yaw_target_angle = yaw_gimbal_fsm.Get_Target_Angle();
pitch_target_angle = pitch_gimbal_fsm.Get_Target_Angle();

 // 统一采用“目标减实际”；切换来源时已经重新锚定，Yaw 直接求差。
 yaw_error = yaw_target_angle - yaw_current_angle;

pitch_error = pitch_target_angle - pitch_current_angle;
// 目标和选定反馈均为弧度，将 Pitch 误差归一化到 [-pi, pi]。
while (pitch_error > kPi)  pitch_error -= kTwoPi;
while (pitch_error < -kPi) pitch_error += kTwoPi;
yaw_speed_pid_output = 0.0f;

/********************************************************************************* */
 // yaw 停止模式：清零输出并复位各控制器状态。
 if (yaw_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_STOP )
{
    yaw_target_speed = 0.0f;
    yaw_control_output = 0.0f;
    yaw_angle_imu_pid.reset();
    yaw_angle_encoder_pid.reset();
    yaw_angle_to_speed_imu_pid.reset();
    yaw_angle_to_speed_encoder_pid.reset();
    yaw_keymouse_angle_imu_pid.reset();
    yaw_keymouse_angle_encoder_pid.reset();
    yaw_keymouse_angle_to_speed_imu_pid.reset();
    yaw_keymouse_angle_to_speed_encoder_pid.reset();
    yaw_remote_speed_imu_pid.reset();
    yaw_remote_speed_encoder_pid.reset();
    yaw_version_angle_imu_pid.reset();
    yaw_version_speed_imu_pid.reset();
}
 // yaw 角度模式：角度环产生目标速度，速度环输出力矩。
 else if (yaw_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_ANGLE)
{
    if (yaw_mode == GIMBAL_MODE_VISION)
    {
        // 视觉角度已由 FSM 对齐到连续 IMU 角；速度(rad/s)、加速度(rad/s^2)使用最近一包。
        // LQR/ESO 直接输出力矩(N*m)，此分支不叠加普通 Yaw PID 和独立前馈对象。
        // 电机反馈力矩用于观测器，feedback_ok=1 的前提是上方电机和 IMU 检查已经通过。
        const YawAutoLqrEsoFeedback_t feedback = {
            yaw_current_angle, yaw_current_speed, gimbal_motor.getTorque(1), 1U
        };
        const YawAutoLqrEsoReference_t reference = {
            yaw_target_angle, vision_comm.GetYawVelocity(),
            vision_comm.GetYawAcceleration()
        };
        YawAutoLqrEsoOutput_t output = {};
        yaw_target_speed = reference.omega_rad_s;
        // 控制器计算周期按固定 0.001 s 传入；任务调度并不是严格的硬件 1 kHz 定时。
        YawAutoLqrEso_Calc(&yaw_auto_ctrl, &yaw_auto_config,
                           &feedback, &reference, 0.001f, &output);
        yaw_control_output = output.tau_cmd_nm;
    }
    else
    {
        // 普通 Yaw：按输入源和反馈来源选择角度/速度双环，速度环输出力矩。
        const bool is_keymouse = (input_dispatcher.GetSource() == InputSource::KeyMouse);
        yaw_target_speed = is_keymouse
            ? (using_encoder ? yaw_keymouse_angle_encoder_pid : yaw_keymouse_angle_imu_pid).UpDate(yaw_error, 0.0f)
            : (using_encoder ? yaw_angle_encoder_pid : yaw_angle_imu_pid).UpDate(yaw_error, 0.0f);
        yaw_speed_pid_output = is_keymouse
            ? (using_encoder ? yaw_keymouse_angle_to_speed_encoder_pid : yaw_keymouse_angle_to_speed_imu_pid).UpDate(yaw_target_speed, yaw_current_speed)
            : (using_encoder ? yaw_angle_to_speed_encoder_pid : yaw_angle_to_speed_imu_pid).UpDate(yaw_target_speed, yaw_current_speed);
        yaw_control_output = yaw_speed_pid_output;
    }
}
 // yaw 保留遥控单速度环；键鼠由 FSM 选择角度或视觉模式。
 else if (yaw_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_SPEED)
{
    yaw_target_speed = yaw_gimbal_fsm.Get_Control_Output();
    yaw_speed_pid_output = (using_encoder ? yaw_remote_speed_encoder_pid : yaw_remote_speed_imu_pid).UpDate(yaw_target_speed, yaw_current_speed);
    yaw_control_output = yaw_speed_pid_output;
}
/****************************************************************************************** */
 // 重力补偿按反馈来源换算水平角：IMU 已是姿态角，编码器扣除实测水平零点。
 // 编码器降级以标定时底盘姿态为基准，无法感知底盘后续倾斜。
 const float pitch_gravity_angle = using_encoder
     ? pitch_current_angle - PITCH_ENCODER_HORIZONTAL_RAD
     : pitch_current_angle;
 pitch_gravity_ff.GravityFeedforward(pitch_gravity_angle);
 const float pitch_gravity_output = pitch_gravity_ff.getFeedforward();
 // Pitch STOP 时不进入角度/速度控制计算。
 if (pitch_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_STOP)
{
    pitch_target_speed = 0.0f;
    pitch_control_output = 0.0f;
    pitch_angle_imu_pid.reset();
    pitch_angle_encoder_pid.reset();
    pitch_angle_to_speed_imu_pid.reset();
    pitch_angle_to_speed_encoder_pid.reset();
    pitch_keymouse_angle_imu_pid.reset();
    pitch_keymouse_angle_encoder_pid.reset();
    pitch_keymouse_angle_to_speed_imu_pid.reset();
    pitch_keymouse_angle_to_speed_encoder_pid.reset();
    pitch_remote_speed_imu_pid.reset();
    pitch_remote_speed_encoder_pid.reset();
    pitch_version_angle_imu_pid.reset();
    pitch_version_speed_imu_pid.reset();
}
 // pitch 角度模式：双环叠加重力补偿，视觉分支另加惯量前馈。
 else if (pitch_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_ANGLE)
{
    if (pitch_mode == GIMBAL_MODE_VISION)
    {
        // 视觉 Pitch 使用独立角度/速度双环，叠加视觉惯量和重力前馈。
        pitch_target_speed = pitch_version_angle_imu_pid.UpDate(pitch_error, 0.0f);
        pitch_vision_dynamics_ff.MomentOfInertiaTuning(pitch_current_speed, pitch_target_speed);
        pitch_control_output = pitch_version_speed_imu_pid.UpDate(
            pitch_target_speed,
            pitch_current_speed
        ) + pitch_vision_dynamics_ff.getTorque()
          + pitch_gravity_output;
    }
    else
    {
        const bool is_keymouse = (input_dispatcher.GetSource() == InputSource::KeyMouse);
        pitch_target_speed = is_keymouse
            ? (using_encoder ? pitch_keymouse_angle_encoder_pid : pitch_keymouse_angle_imu_pid).UpDate(pitch_error, 0.0f)
            : (using_encoder ? pitch_angle_encoder_pid : pitch_angle_imu_pid).UpDate(pitch_error, 0.0f);
        pitch_control_output = is_keymouse
            ? (using_encoder ? pitch_keymouse_angle_to_speed_encoder_pid : pitch_keymouse_angle_to_speed_imu_pid).UpDate(pitch_target_speed, pitch_current_speed)
            : (using_encoder ? pitch_angle_to_speed_encoder_pid : pitch_angle_to_speed_imu_pid).UpDate(pitch_target_speed, pitch_current_speed);
        pitch_control_output += pitch_gravity_output;

    }
}
 // pitch 保留遥控单速度环，并叠加重力补偿。
 else if (pitch_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_SPEED)
{
    pitch_target_speed = pitch_gimbal_fsm.Get_Control_Output();
    pitch_control_output = (using_encoder ? pitch_remote_speed_encoder_pid : pitch_remote_speed_imu_pid).UpDate(pitch_target_speed, pitch_current_speed)
                         + pitch_gravity_output;
}

/********************************************************************************** */
        // 第一层：限制总力矩幅值至 +/-9 N*m，防止 PID、前馈或观测补偿叠加后超过发送范围。
        yaw_control_output = BSP::Motor::DM::ClampMitCommandValue(
            yaw_control_output, -J4340_TORQUE_LIMIT_NM, J4340_TORQUE_LIMIT_NM);
        pitch_control_output = BSP::Motor::DM::ClampMitCommandValue(
            pitch_control_output, -J4340_TORQUE_LIMIT_NM, J4340_TORQUE_LIMIT_NM);

        // 第二层：Yaw 上限拦正、下限拦负；Pitch 只在下限拦负，保留上限正力矩支撑/制动。
        // 等于边界也拦截，朝范围内的制动/回退力矩允许通过；这里不主动规划减速轨迹。
        // 必须放在全部补偿之后，否则后加的惯量或重力前馈可能重新产生向外力矩。
        yaw_control_output = yaw_gimbal_fsm.Limit_Torque(yaw_control_output);
        pitch_control_output = pitch_gimbal_fsm.Limit_Torque(pitch_control_output);

        // 最终发送保护：仅将掉线轴力矩清零，在线轴保留本周期控制结果。
        if (!yaw_motor_online) yaw_control_output = 0.0f;
        if (!pitch_motor_online) pitch_control_output = 0.0f;

            // MIT 参数顺序为电机槽、位置(rad)、速度(rad/s)、Kp、Kd、力矩(N*m)。
            // 位置/速度及 Kp/Kd 均为 0，电机按力矩指令运行，姿态闭环由本任务实现。
            gimbal_motor.ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, yaw_control_output);   // Yaw
            gimbal_motor.ctrl_Mit(2, 0.0f, 0.0f, 0.0f, 0.0f, pitch_control_output); // Pitch
        // UART10 的 VOFA 六通道 JustFloat 帧为 28 字节；当前每 4 个调度拍尝试 DMA 发送一次。
        // 发送发生在有效控制分支；实际频率取决于任务周期及故障跳过，不是固定每 10 ms。
        
        
            if ((can2_tick % 4) == 0)
            {
                vofa_send(
                    pitch_target_angle,       // 通道 1：Pitch 目标角，rad
                    pitch_current_angle,      // 通道 2：Pitch 选定反馈角，rad
                    pitch_target_speed,              // 通道 3：Pitch 目标角速度，rad/s
                    pitch_current_speed,       // 通道 4：Pitch 选定反馈角速度，rad/s
                    pitch_target_angle,      // 通道 5：再次发送 Pitch 目标角，当前与通道 1 重复
                    pitch_control_output      // 通道 6：Pitch 最终发送力矩，N*m
                );
                // // VOFA channels for yaw tuning.
                // float vofa_data[] = {
                //     yaw_target_angle,                              // ch1: yaw angle command
                //     yaw_current_angle,                             // ch2: yaw angle feedback
                //     yaw_target_angle - yaw_current_angle,          // ch3: yaw angle error
                //     yaw_angle_pid_output,                          // ch4: active yaw angle PID output
                //     yaw_angle_ff_output,                           // ch5: active yaw total feedforward
                //     yaw_angle_ff_friction,                         // ch6: active yaw friction feedforward
                //     yaw_control_output,                            // ch8: yaw final torque command
                //     yaw_error,                                     // ch9: yaw angle error
                //     dm4340_state[0].current_a                      // ch10: yaw motor current
                // };
                // vofa_sendN(vofa_data, static_cast<uint8_t>(sizeof(vofa_data) / sizeof(vofa_data[0]))); // send data to VOFA
            }
        

         }



        vTaskDelay(1); // 相对延时 1 个 FreeRTOS tick；实际循环周期还包含本周期执行时间
    }
}







// 辅助安全检查接口：本任务主循环另有按轴零力矩保护，不依赖此函数执行。
// 此函数更新在线/蜂鸣器状态并清除共享 remote 输入，没有直接发送零力矩 CAN 帧。
void SafetyCheck()
{
    // 遍历所有电机
    for (int i = 1; i <= 2; i++)
    {
        // isConnected(逻辑状态槽, 蜂鸣器标识)，本工程两者均使用 1=Yaw、2=Pitch。
        if (!gimbal_motor.isConnected(i, i))
        {
            // 电机 i 掉线了！
            // 蜂鸣器会自动报警

            // 清除共享输入和档位，阻止继续使用此前操作；实际电机零力矩发送在主循环保护中。
			remote.chassis_vx = 0.0f;
			remote.chassis_vy = 0.0f;
			remote.gimbal_yaw = 0.0f;
			remote.gimbal_pitch = 0.0f;
			remote.s1 = 0;
			remote.s2 = 0;
        }
    }

    // 或者直接获取掉线的电机编号
    uint8_t offline_motor = gimbal_motor.getOfflineStatus();
	
    // 返回 0 表示都在线
    // 非零值表示驱动报告的掉线电机；当前只配置两台 J4340，返回值在本函数中未进一步使用。

}

//开vofa软件的justfloat模式
uint8_t send_str2[sizeof(float) * 11]; // 分配11个float空间（44字节，10数据+1帧尾）
// 可变通道 JustFloat 发送：USB CDC-HS 路径，最多 10 个 float，另附 4 字节帧尾。
// USB 未就绪或正在发送时直接跳过；与下方六通道 UART10 DMA 发送接口不同。
void vofa_sendN(const float *data, uint8_t count)
{
    if (data == nullptr || count == 0)
    {
        return;
    }

    if (count > 10)
    {
        count = 10;
    }

    USBD_CDC_HandleTypeDef *hcdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceHS.pClassData);
    if (hcdc == nullptr || hcdc->TxState != 0)
    {
        return;
    }

    memcpy(send_str2, data, sizeof(float) * count);
    *((uint32_t*)&send_str2[sizeof(float) * count]) = 0x7F800000;

    CDC_Transmit_HS(send_str2, static_cast<uint16_t>(sizeof(float) * (count + 1)));
}


// uint8_t send_str2[sizeof(float) * 7]; // 6个float数据 + 4字节帧尾（28字节）

// 开vofa软件的justfloat模式
// 固定六通道 JustFloat 发送：UART10 DMA；6 个小端 float(24 字节)加帧尾(4 字节)。
// 单位由调用方定义；该函数只封装数据，不把 rad 转为度，也不更改通道物理意义。
void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6)
{
    const uint8_t sendSize = sizeof(float); // 单浮点数占4字节

    // 将6个浮点数据写入缓冲区（小端模式）
    *((float*)&send_str2[sendSize * 0]) = x1;
    *((float*)&send_str2[sendSize * 1]) = x2;
    *((float*)&send_str2[sendSize * 2]) = x3;
    *((float*)&send_str2[sendSize * 3]) = x4;
    *((float*)&send_str2[sendSize * 4]) = x5;
    *((float*)&send_str2[sendSize * 5]) = x6;

    // 写入帧尾（协议要求 0x00 0x00 0x80 0x7F）
    *((uint32_t*)&send_str2[sizeof(float) * 6]) = 0x7F800000; // 小端存储为 00 00 80 7F

    HAL::UART::Data vofa_tx_data{send_str2, sizeof(float) * 7};
    HAL::UART::get_uart_bus_instance().get_uart10().transmit_dma(vofa_tx_data);
}

 // 将驱动已解析的反馈缓存复制为电机状态快照，不在这里接收 CAN 或运行姿态 PID。
 // angle_deg/angle_rad 为原始位置；velocity_rpm/velocity_rads 为转速；delta_angle 为累计角(度)。
 // 电机反馈用于状态监测和 IMU 故障降级；两轴限位始终使用原始编码器。
 void ControlTask() {
    for (int i = 0; i < 2; i++) {
        uint8_t motor_id = i + 1; // 电机逻辑 ID 通常从 1 开始
        
        dm4340_state[i].angle_deg   = gimbal_motor.getAngleDeg(motor_id);
        dm4340_state[i].angle_rad   = gimbal_motor.getAngleRad(motor_id);
        dm4340_state[i].velocity_rpm   = gimbal_motor.getVelocityRpm(motor_id);
        dm4340_state[i].velocity_rads  = gimbal_motor.getVelocityRads(motor_id);
        dm4340_state[i].current_a     = gimbal_motor.getCurrent(motor_id);
        dm4340_state[i].temperature        = gimbal_motor.getTemperature(motor_id);
        dm4340_state[i].delta_angle       = gimbal_motor.getAddAngleDeg(motor_id);
    }
}

// ==================== IMU / 编码器反馈选择 ====================
// 四个引用参数输出选定来源的控制角(rad)和速度(rad/s)。
// 健康检查在连续角展开之前执行。返回值只表示来源切换，不是停止控制请求。
static bool UpdateGimbalFeedback(float &yaw_angle, float &yaw_speed,
                                float &pitch_angle, float &pitch_speed)
{
    const ImuFeedbackFSM::Sample sample = {
        DegreesToRadians(imu.GetAngle(2)), DegreesToRadians(imu.GetAngle(1)),
        DegreesToRadians(imu.GetGyro(2)), DegreesToRadians(imu.GetGyro(0))
    };
    const bool source_changed = imu_feedback_fsm.Update(imu.isConnected(), sample, HAL_GetTick());
    if (imu_feedback_fsm.UsingEncoder())
    {
        yaw_angle = gimbal_motor.getAngleRad(1);
        yaw_speed = gimbal_motor.getVelocityRads(1);
        pitch_angle = gimbal_motor.getAngleRad(2);
        pitch_speed = gimbal_motor.getVelocityRads(2);
    }
    else
    {
        // 从编码器切回时重置圈数基准，丢弃 IMU 故障期间的旧连续角历史。
        ImuData_user.yaw = GetContinuousYawAngle(sample.yaw_rad, source_changed);
        ImuData_user.pitch = sample.pitch_rad;
        ImuData_user.gyro_z = sample.yaw_speed_rad_s;
        ImuData_user.gyro_y = sample.pitch_speed_rad_s;
        yaw_angle = ImuData_user.yaw;
        yaw_speed = ImuData_user.gyro_z;
        pitch_angle = ImuData_user.pitch;
        pitch_speed = ImuData_user.gyro_y;
    }
    return source_changed;
}



