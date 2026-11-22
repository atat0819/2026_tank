/**
 * @file can_send_task.cpp
 * @brief 云台通信、输入调度及 Pitch 闭环控制任务。
 *
 * 控制顺序：读取输入和反馈 -> 在线保护 -> 故障恢复锚定 -> Pitch FSM 更新
 *           -> 目标限位检查 -> 控制器复位 -> PID 与补偿
 *           -> +/-27 N*m 幅值限制 -> FSM 向外力矩拦截 -> MIT 指令发送。
 *
 * Pitch 仅使用编码器反馈；角为 rad、速度为 rad/s、力矩为 N*m。
 * Pitch 目标、反馈和机械限位使用同一原始电机编码器参考系。
 * 编码器到边界后向外目标收回到当前控制角，全部补偿叠加后再拦截向外力矩。
 * 本任务更新 Pitch 云台 FSM；摩擦轮、拨弹控制由其他任务中的对应 FSM 负责。
 */
#include "can_send_task.hpp"
#include "watchdog_heartbeat.h"
#include "remote_control_task.hpp"
#include "../communication_between_boards/boards_communication_uart_485.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "gimbal_task.hpp"
#include "gantry_task.hpp"
#include "usart.h"
#include "usbd_cdc_if.h"
#include "../feeder_fsm/gimbal_fsm.hpp"
#include "../feeder_fsm/yaw_fsm.hpp"
#include "../feeder_fsm/motor_recovery_fsm.hpp"
#include "../communication_between_boards/input_dispatcher.hpp"
#include "../user/core/Alg/Feedforward/Feedforward.hpp"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include "../user/core/BSP/Motor/DM/DmMotor.hpp"
#include <algorithm>
#include <cmath>
#include <string.h>

extern "C" USBD_HandleTypeDef hUsbDeviceHS;

namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float DEG_TO_RAD = kPi / 180.0f;
constexpr float J4340_TORQUE_LIMIT_NM = 27.0f; // 与 J4340 Parameters 的 T_MIN/T_MAX 对齐，N*m
constexpr float PITCH_MAX_SPEED_RAD_S = 95.0f * DEG_TO_RAD;
constexpr float PITCH_REMOTE_ANGLE_MIT_KD = 1.3f; // 遥控器角度双环的 MIT 速度反馈增益
constexpr float PITCH_REMOTE_SPEED_MIT_KD = 1.3f; // 遥控器单速度环的 MIT 速度反馈增益
constexpr float PITCH_KEYMOUSE_MIT_KD = 1.3f; // 键鼠 MIT 速度反馈增益，独立调节
constexpr float PITCH_ENCODER_HORIZONTAL_RAD = 46.7f * DEG_TO_RAD; // 实测水平时 ID8 原始编码器角

// 编码器度数和配置中的角度数值，进入控制计算前统一转换为弧度。
float DegreesToRadians(float value)
{
    return value * DEG_TO_RAD;
}




}


/* 注：原 bxCAN 时代的 txDataBuffer / rxDataBuffer0 / rxDataBuffer1 / txHeader /
   rxHeader0 / rxHeader1 已删除 —— H7 的 FDCAN 收发缓冲区由 HAL::FDCAN 内部管理。*/

/******************************************************************************** */
// ============ 主控角度 PID → 目标速度，电机 MIT 速度反馈 → 力矩 ============
// PID 构造参数依次为 Kp、Ki、Kd、输出幅值上限、积分状态上限、积分分离误差阈值。
// 输入角度误差(rad)，输出目标速度(rad/s)；max 直接限制为 95 度/s 对应的弧度值。
ALG::PID::PID pitch_remote_angle_encoder_pid(32.0f,0.015f, 0.0f, PITCH_MAX_SPEED_RAD_S, 1000.0f, 100.0f);


// 键鼠角度 PID 与遥控器独立，均使用编码器反馈。
ALG::PID::PID pitch_keymouse_angle_encoder_pid(26.5f, 0.0f, 0.0f, PITCH_MAX_SPEED_RAD_S, 1000.0f, 100.0f);
// Pitch 重力前馈参数依次为补偿幅值(N*m)、相位(rad)；沿用原值，待实机标定。
Alg::Feedforward::Gravity pitch_gravity_ff(
    0.0f, 0.0f);
/********************************************************************************** */

ALG::PID::PID yaw_center_encoder_pid(26.5f, 0.0f, 0.0f, PITCH_MAX_SPEED_RAD_S, 1000.0f, 100.0f);

ALG::PID::PID yaw_angle_encoder_pid(26.5f, 0.0f, 0.0f, PITCH_MAX_SPEED_RAD_S, 1000.0f, 100.0f);
float yaw_angle_kd = 1.3f;  // Yaw 鼠标微调的 MIT 速度反馈增益
float yaw_center_kd = 1.3f; // Yaw 回中的 MIT 速度反馈增益


using Remote = BSP::REMOTE_CONTROL::RemoteController;


Struct_Gimbal_FSM_Config pitch_gimbal_fsm_config;  // pitch 轴的 FSM 配置结构体
Class_Gimbal_FSM pitch_gimbal_fsm;   // 定义 pitch 轴的 FSM 实例

DM4340_State_t dm4340_state[2]; // 下标 0=Yaw ID7，1=Pitch ID8
RemoteData_t RemoteData;


float pitch_target_angle = 0.0f; // 目标角(rad)；此处为原有初值，启动时由编码器和 FSM 覆盖
float pitch_current_angle = 0.0f; // Pitch 原始编码器反馈角(rad)

float pitch_target_speed = 0.0f; // 目标角速度(rad/s)：角度分支由外环 PID 产生
float pitch_current_speed = 0.0f; // Pitch 电机反馈角速度(rad/s)






BSP::Motor::DM::J4310<1> yaw_motor(
    0,{7},{7},HAL::FDCAN::FdcanDeviceId::HAL_Fdcan1);
static volatile uint8_t yaw_drive_status = 0U;
static Yaw_FSM yaw_fsm;

// Pitch，物理 CAN ID8 对应逻辑槽 1。
BSP::Motor::DM::J4340<1> pitch_motor(
    0,{8},{8},HAL::FDCAN::FdcanDeviceId::HAL_Fdcan1);
static volatile uint8_t pitch_drive_status = 0U;

static void UpdatePitchMotorFeedback();
static void UpdateYawMotorFeedback();





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

float pitch_error = 0.0f; // 目标减反馈(rad)，计算后将误差归一化到 [-pi, pi]
float pitch_control_output = 0.0f; // Pitch 受限估算力矩(N*m)，并非电机实测力矩
volatile uint8_t gimbal_remote_offline = 0; // 遥控数据无效标志：1=本任务判定失联/异常

// 复位 Pitch 控制器和补偿状态。
static void ResetPitchControlState()
{
    pitch_control_output = 0.0f;
    pitch_target_speed = 0.0f;
    pitch_remote_angle_encoder_pid.reset();
    pitch_keymouse_angle_encoder_pid.reset();
}

static void ResetYawControlState()
{
    yaw_fsm.Reset();
    yaw_angle_encoder_pid.reset();
    yaw_center_encoder_pid.reset();
}

static bool UpdateMotorEnable(BSP::Motor::DM::DMMotorBase<1> &motor, MotorRecoveryFSM &recovery,
                              bool online, uint8_t drive_status, uint32_t now_tick, bool &was_ready)
{
    if (!online)
    {
        if (was_ready)
        {
            motor.Off(1, BSP::Motor::DM::MIT);
            recovery.Reset();
        }
        else
            motor.ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // 请求新反馈，等待确认驱动状态。
        was_ready = false;
        return false;
    }

    if (recovery.Update(true, drive_status, now_tick) == MotorRecoveryFSM::Request::On)
        motor.On(1, BSP::Motor::DM::MIT);
    was_ready = drive_status == 1U;
    return was_ready;
}

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6);
void vofa_sendN(const float *data, uint8_t count);


extern "C" void can_send_task(void *argument)
{
    // CAN 任务负责初始化总线、注册反馈回调，并周期性运行云台控制。
    // 延时 1000 个 CMSIS-OS tick，给其他任务和通信外设初始化留出时间。
    osDelay(1000);

/*********************************************************************************** */
// ==================== Pitch 状态机参数 ====================
// Pitch 闭环与机械限位均取 ID8 原始编码器。
// 上下限沿用用户提供的原始编码器角。
pitch_gimbal_fsm_config.angle_step = DegreesToRadians(0.11f); // 单位摇杆每次累加 0.11 度
pitch_gimbal_fsm_config.speed_scale = PITCH_MAX_SPEED_RAD_S; // 单位摇杆对应 95 度/s，内部为 rad/s
pitch_gimbal_fsm_config.mouse_speed_scale = DegreesToRadians(0.2f);   // 预留鼠标速度比例，当前 FSM 未读取
pitch_gimbal_fsm_config.mouse_angle_scale = DegreesToRadians(0.013f);  // 累计鼠标 Y 位移每像素改变目标 0.013 度
pitch_gimbal_fsm_config.min_angle = DegreesToRadians(34.2f);    // 原始编码器 <= 34.2 度时禁止继续减小角度
pitch_gimbal_fsm_config.max_angle = DegreesToRadians(67.9f);    // 原始编码器 >= 67.9 度时禁止继续增大角度
pitch_gimbal_fsm_config.limit_angle = 1U; // 限位总开关；实际到边界后另拦截最终向外力矩
pitch_gimbal_fsm_config.limit_upper_torque = 0U; // 保留上限目标保护，允许正力矩支撑重力/制动下降；下限仍拦负力矩
// 沿用现有限位策略：机械边界收回向外目标，最终发送前拦截向外力矩。
// 到机械边界后，Update_Angle_Limit 收回向外目标，最终 Limit_Torque 拦截向外力矩。
pitch_gimbal_fsm_config.limit_target_angle = 0U; // 遥控、键鼠、视觉目标均使用边界目标保护
pitch_gimbal_fsm_config.normalize_angle = 0U; // Pitch 不做 Yaw 式圈数对齐/角度归一化
pitch_gimbal_fsm_config.continuous_angle = 0U; // 配合上面的归一化关闭，不使用连续圈数展开
pitch_gimbal_fsm.Init(pitch_gimbal_fsm_config, GIMBAL_STATUS_STOP); // 初始停止，运行时由输入决定状态
/************************************************************************************* */

	static uint32_t can2_tick = 0; // 有效遥控分支中的调度拍数，用于 VOFA 分频，不是毫秒时间戳
static bool remote_was_offline = false; // 遥控失联或双下停车后，恢复时用当前反馈重建目标
static bool pitch_motor_was_unready = false; // Pitch 恢复后只重新锚定自身

static uint16_t startup_protect = 0; // 遥控有效且 Pitch 在线时累计的启动保护次数

	// --- 在进入循环前初始化 ---
    // 这行代码执行时会触发 HAL_FDCAN_Start 和开启接收中断     
	HAL::FDCAN::get_fdcan_bus_instance(); // 触发 FDCAN bus 初始化：HAL_FDCAN_Start + 激活中断通知
	    auto &fdcan1 = HAL::FDCAN::get_fdcan_bus_instance().get_device(HAL::FDCAN::FdcanDeviceId::HAL_Fdcan1);
        auto &fdcan2 = HAL::FDCAN::get_fdcan_bus_instance().get_device(HAL::FDCAN::FdcanDeviceId::HAL_Fdcan2);
        auto &fdcan3 = HAL::FDCAN::get_fdcan_bus_instance().get_device(HAL::FDCAN::FdcanDeviceId::HAL_Fdcan3);
    // FDCAN1：Pitch ID8、Yaw ID7；FDCAN2：摩擦轮。


    // FDCAN3：龙门架 3508 ID5/6 和 DM4310 ID9。
    fdcan1.register_rx_callback([](const HAL::FDCAN::Frame &frame) {
        if (frame.id == 8 && frame.dlc == 8 && !frame.is_extended_id &&
            !frame.is_remote_frame && (frame.data[0] & 0x0F) == 8)
        {
            pitch_motor.Parse(frame);
            pitch_drive_status = frame.data[0] >> 4;
        }
        else if (frame.id == 7 && frame.dlc == 8 && !frame.is_extended_id &&
                 !frame.is_remote_frame && (frame.data[0] & 0x0F) == 7)
        {
            yaw_motor.Parse(frame);
            yaw_drive_status = frame.data[0] >> 4;
            Gimbal485_UpdateYawFromISR(yaw_motor.getAngleDeg(1));
        }
    });

    fdcan2.register_rx_callback([](const HAL::FDCAN::Frame &frame) {

            if (frame.id >= 0x201 && frame.id <= 0x204)
            {
                friction_motor.Parse(frame);
            }

    });

    fdcan3.register_rx_callback([](const HAL::FDCAN::Frame &frame) {

         if (frame.id == 0x205 || frame.id == 0x206)
            {
                gantry_3508_motor.Parse(frame);
            }
        else if (frame.dlc == 8 && !frame.is_extended_id && !frame.is_remote_frame)
        {
             if (frame.id == 9 && (frame.data[0] & 0x0F) == 9)
            {
                gantry_dm4310_motor.Parse(frame);
                gantry_watch.dm_status = frame.data[0] >> 4;
            }
        }


    });


// 启动时短暂等待有效的底盘遥控数据；摇杆 -1 是合法值，不用于判断失联。
for (uint32_t wait = 0; wait < 200; wait++)
{
    vTaskDelay(1);
    if (Gimbal485_RemoteOnline())
    {
        break;  // 遥控器数据有效了，提前退出
    }
}

UpdatePitchMotorFeedback();
UpdateYawMotorFeedback();

// 初始目标使用编码器；电机或遥控恢复时按编码器角重新锚定。
pitch_target_angle = pitch_current_angle;

// 两轴启动先失能，进入各自的正常控制分支后再请求使能。
MotorRecoveryFSM pitch_recovery_fsm;
MotorRecoveryFSM yaw_recovery_fsm;
yaw_motor.Off(1, BSP::Motor::DM::MIT);
pitch_motor.Off(1, BSP::Motor::DM::MIT);
vTaskDelay(500); // 等电机上电完成

    bool yaw_wait_for_release = false;
    bool yaw_was_ready = false;
    bool pitch_was_ready = false;
    uint32_t yaw_last_off_tick = 0U;

    // 主循环：采集输入、更新状态机、计算控制量并发送 CAN。
    for(;;)
    {
        UpdateYawMotorFeedback();

        RemoteData_t remote_input{};
        int32_t unused_mouse_delta_x = 0;
        int32_t mouse_delta_y = 0;
        // 同一临界区取出两个轴共用的 DBUS 输入和各自待消费的鼠标位移。
        taskENTER_CRITICAL();
        UpdatePitchMotorFeedback();
        const bool pitch_motor_online = pitch_motor.isConnected(1, 8);
        const uint8_t pitch_status = pitch_drive_status;
        const auto yaw_mouse = Gimbal485_ConsumeYawMouseInput();
        remoteController.ConsumeMouseDelta(unused_mouse_delta_x, mouse_delta_y);
        const bool remote_enabled = yaw_mouse.online && !g_watchdog_rearm_required;
        if (yaw_mouse.online)
        {
            remote_input.chassis_vx = remoteController.get_left_y();
            remote_input.chassis_vy = remoteController.get_left_x();
            remote_input.gimbal_pitch = remoteController.get_right_y();
            remote_input.s1 = remoteController.get_s1();
            remote_input.s2 = remoteController.get_s2();
            remote_input.mouse_x = remoteController.get_mouseX();
            remote_input.mouse_y = remoteController.get_mouseY();
            remote_input.mouse_z = remoteController.get_mouseZ();
            remote_input.mouse_left = remoteController.get_mouseLeft() ? 1 : 0;
            remote_input.mouse_right = remoteController.get_mouseRight() ? 1 : 0;
            remote_input.keyboard = remoteController.get_keyboard();
        }
        const bool valid_switches = remote_input.s1 >= Remote::UP && remote_input.s1 <= Remote::MIDDLE &&
                                    remote_input.s2 >= Remote::UP && remote_input.s2 <= Remote::MIDDLE;
        RemoteData = remote_enabled && valid_switches ? remote_input : RemoteData_t{};
        taskEXIT_CRITICAL();

        const bool double_down = remote_enabled && remote_input.s1 == Remote::DOWN && remote_input.s2 == Remote::DOWN;
         if (!remote_enabled || !valid_switches || double_down)
        {
            gimbal_remote_offline = !remote_enabled || !valid_switches;
            // 安全停车期间没有有效的键鼠释放帧。
            if (yaw_fsm.Is_Target_Valid())
                yaw_wait_for_release = true;
            ResetYawControlState();
            yaw_was_ready = false;
            pitch_was_ready = false;

            const uint32_t now_tick = HAL_GetTick();
            if (yaw_recovery_fsm.Update(false, MotorRecoveryFSM::UNKNOWN_DRIVE_STATUS, now_tick) ==
                MotorRecoveryFSM::Request::Off)
            {
                yaw_motor.Off(1, BSP::Motor::DM::MIT);
                yaw_last_off_tick = now_tick;
            }
            if (pitch_recovery_fsm.Update(false, MotorRecoveryFSM::UNKNOWN_DRIVE_STATUS, now_tick) ==
                MotorRecoveryFSM::Request::Off)
                pitch_motor.Off(1, BSP::Motor::DM::MIT);
            remote_was_offline = true;
            const uint8_t stop_switch = double_down ? static_cast<uint8_t>(Remote::DOWN) : 0U;
            input_dispatcher.Update(stop_switch, stop_switch, 0U, false, false);
            ResetPitchControlState();
        }
         else   //遥控器正常，看门狗正常
         {
        if (remote_was_offline)
        {
            yaw_recovery_fsm.Reset();
            pitch_recovery_fsm.Reset();
        }
        input_dispatcher.Update(
            remote_input.s1,
            remote_input.s2,
            remote_input.keyboard,
            remote_input.mouse_left != 0,
            remote_input.mouse_right != 0
        );
        const InputSource input_source = input_dispatcher.GetSource();
        const bool is_keymouse = input_source == InputSource::KeyMouse;
        // 相机 Yaw：按状态计算速度，并在发送前处理限位和失联。
        {
            Struct_Yaw_Input input{};
            input.is_keymouse = is_keymouse;
            input.enabled = true;
            input.mouse_active = yaw_mouse.mouse_active;
            input.reanchor = yaw_mouse.reanchor;
            input.mouse_delta_x = yaw_mouse.mouse_delta_x;

            taskENTER_CRITICAL();
            const uint32_t now_tick = HAL_GetTick();
            const bool online = yaw_motor.isConnected(1, 7);
            const uint8_t drive_status = yaw_drive_status;
            const float feedback_angle = yaw_motor.getAngleRad(1);
            const float feedback_speed = yaw_motor.getVelocityRads(1);
            taskEXIT_CRITICAL();

            const bool mouse_active = is_keymouse && input.mouse_active;
            // 角度越界由 Yaw FSM 停止输出，驱动保持使能。
            const bool finite_feedback = std::isfinite(feedback_angle) && std::isfinite(feedback_speed);
            const bool ready = online && finite_feedback && drive_status == 1U;

            const bool target_valid = yaw_fsm.Is_Target_Valid();
            const bool interrupted = target_valid && (!ready || yaw_mouse.reset);
            // 失联时的空输入不能当成松手，必须收到一帧有效的键鼠释放。
            if (interrupted && mouse_active)
                yaw_wait_for_release = true;
            else if (is_keymouse && !mouse_active)
                yaw_wait_for_release = false;

            const bool inhibit_enable = (target_valid && yaw_mouse.reset) ||
                                        (online && (!finite_feedback || drive_status > 1U));
            if (inhibit_enable)
            {
                if (yaw_was_ready || yaw_recovery_fsm.HasPendingOn() ||
                    (online && drive_status == 1U &&
                     now_tick - yaw_last_off_tick >= MotorRecoveryFSM::ENABLE_RETRY_MS))
                {
                    yaw_motor.Off(1, BSP::Motor::DM::MIT);
                    yaw_last_off_tick = now_tick;
                }
                yaw_recovery_fsm.Reset();
                yaw_was_ready = false;
            }

            if (inhibit_enable ||
                !UpdateMotorEnable(yaw_motor, yaw_recovery_fsm, online, drive_status, now_tick, yaw_was_ready))
            {
                ResetYawControlState();
            }
            else
            {
                const auto previous_mode = yaw_fsm.Get_Mode();
                // 键鼠档自动使能；失联恢复后等待右键释放期间保持零力矩。
                if (is_keymouse && yaw_wait_for_release)
                    yaw_fsm.Reset();
                else
                    yaw_fsm.Update(input, feedback_angle, now_tick);
                const auto mode = yaw_fsm.Get_Mode();
                if (mode != previous_mode || input.reanchor || mode == Yaw_FSM::Mode::Stop)
                {
                    yaw_angle_encoder_pid.reset();
                    yaw_center_encoder_pid.reset();
                }

                float target_speed = 0.0f;
                float mit_kd = 0.0f;
                const float target_angle = yaw_fsm.Get_Target_Angle();
                if (mode == Yaw_FSM::Mode::Mouse)
                {
                    target_speed = yaw_angle_encoder_pid.UpDate(target_angle, feedback_angle);
                    mit_kd = yaw_angle_kd;
                }
                else if (mode == Yaw_FSM::Mode::Center)
                {
                    const float pid_speed = yaw_center_encoder_pid.UpDate(target_angle, feedback_angle);
                    target_speed = std::clamp(pid_speed, -Yaw_FSM::CENTER_SPEED_RAD_S, Yaw_FSM::CENTER_SPEED_RAD_S);
                    mit_kd = yaw_center_kd;
                }

                float torque_command = 0.0f;
                if (!std::isfinite(mit_kd) || mit_kd <= 0.0f)
                {
                    mit_kd = 0.0f;
                }
                else
                {
                    // 与 Pitch 一样，正常由 MIT 速度反馈闭环，保护触发时改发限幅后的直接力矩。
                    const float estimated_torque = mit_kd * (target_speed - feedback_speed);
                    const float limited_torque = yaw_fsm.Limit_Torque(estimated_torque, feedback_angle);
                    if (Yaw_FSM::Is_Soft_Limit_Active(feedback_angle) || limited_torque != estimated_torque)
                    {
                        mit_kd = 0.0f;
                        torque_command = limited_torque;
                    }
                }
                yaw_motor.ctrl_Mit(1, 0.0f, target_speed, 0.0f, mit_kd, torque_command);
            }
        }

        gimbal_remote_offline = 0;
        if (!is_keymouse)
        {
            // 遥控器档丢弃鼠标累计量，避免切到键鼠后继承此前移动。
            mouse_delta_y = 0;
        }




        
/******************************************************************** */
// 每轮提交最新快照，收到回复后即可发送；无回复最多等待 2ms，未使用字段归零。
// 仅发送操作输入，协议不包含云台 IMU 字段。
// Chassis transmits control data and the gimbal replies with yaw only.
 // tick 用于分频发送和周期任务调度。
 can2_tick++;/************************************************************************************** */

/************************************************************************************** */
  const bool pitch_motor_ready = UpdateMotorEnable(pitch_motor, pitch_recovery_fsm, pitch_motor_online,
                                                   pitch_status, HAL_GetTick(), pitch_was_ready);
  // Pitch 驱动未确认使能时保持 STOP，恢复后从当前角度接管。
  if (!pitch_motor_ready)
  {
      pitch_motor_was_unready = true;
      ResetPitchControlState();
  }
  else if (pitch_motor_was_unready || remote_was_offline)
  {
      pitch_gimbal_fsm.ReAnchor(pitch_current_angle);
      ResetPitchControlState();
      pitch_motor_was_unready = false;
      mouse_delta_y = 0;
  }
  remote_was_offline = false;

/************************************************************************************** */
//判断为何种模式

if (startup_protect < 500)
{
    // 前 500 次 Pitch 在线的有效更新强制 STOP；Pitch 掉线不累计。
    if (pitch_motor_online) startup_protect++;
    Struct_Gimbal_Input stop_input = {};
    stop_input.s1 = Remote::DOWN;
    stop_input.s2 = Remote::DOWN;
    pitch_gimbal_fsm.Update(stop_input, pitch_current_angle);
}
else
{
    /************************************************************************************* */
    // FSM 内部自行判断模式：传入原始输入，FSM 根据 s1/s2/键鼠 自行决定
    // 遥控双下为 STOP；其他档位沿用 FSM 的角度/速度映射。
    // 纯编码器模式禁止视觉接管，防止使用原 IMU 绝对角目标。
    // 角度模式累加摇杆/鼠标位移，切模式或输入源时按编码器角重新锚定。
    Struct_Gimbal_Input pitch_input = {};
    pitch_input.s1       = remote_input.s1;
    pitch_input.s2       = remote_input.s2;
    pitch_input.is_keymouse = is_keymouse;
    pitch_input.joystick_speed = remote_input.gimbal_pitch;
    pitch_input.mouse_speed = static_cast<float>(remote_input.mouse_y);
    pitch_input.mouse_angle_delta = static_cast<float>(mouse_delta_y); // Pitch 按现有方向映射使用正 Y 累计位移
    pitch_input.mouse_dx = remote_input.mouse_x;
    pitch_input.mouse_dy = remote_input.mouse_y;
    pitch_input.mouse_right_held = input_dispatcher.IsVisionMode();
    pitch_input.vision_ready = false;
    pitch_input.vision_fresh = false;
    if (!pitch_motor_ready)
    {
        pitch_input = {};
        pitch_input.s1 = Remote::DOWN;
        pitch_input.s2 = Remote::DOWN;
    }
    pitch_gimbal_fsm.Update(pitch_input, pitch_current_angle);
}

// Pitch 在线时用原始编码器更新机械限位。
// 第一个角用于与 min/max 比较；第二个角用于收回向外目标。
// 到机械边界且目标仍向外时，收回到当前控制角；最终再拦截向外力矩。
if (pitch_motor_online)
    pitch_gimbal_fsm.Update_Angle_Limit(pitch_current_angle, pitch_current_angle);
const uint8_t pitch_mode_changed = pitch_gimbal_fsm.Take_Mode_Changed_Flag();
const uint8_t pitch_limit_reset = pitch_gimbal_fsm.Take_Limit_Reset_Flag();

// Take_* 返回后清除标志：模式/输入源切换或目标保护引起的复位在本周期控制计算前处理。
// 本周期末 Limit_Torque 产生的新标志则留待下一周期处理，避免旧积分和观测状态继续向外推。
if (pitch_mode_changed != 0U || pitch_limit_reset != 0U)
{
    pitch_remote_angle_encoder_pid.reset();
    pitch_keymouse_angle_encoder_pid.reset();
}

// 完成目标保护后再取目标，确保 PID 不使用已被 FSM 撤回的向外目标。
pitch_target_angle = pitch_gimbal_fsm.Get_Target_Angle();

pitch_error = pitch_target_angle - pitch_current_angle;
// 目标和选定反馈均为弧度，将 Pitch 误差归一化到 [-pi, pi]。
while (pitch_error > kPi)  pitch_error -= kTwoPi;
while (pitch_error < -kPi) pitch_error += kTwoPi;

/********************************************************************************* */
 // 重力补偿按编码器换算水平角：扣除实测水平零点。
 // 以标定时底盘姿态为基准，无法感知底盘后续倾斜；保留原补偿参数。
 const float pitch_gravity_angle = pitch_current_angle - PITCH_ENCODER_HORIZONTAL_RAD;
 pitch_gravity_ff.GravityFeedforward(pitch_gravity_angle);
 const float pitch_gravity_output = pitch_gravity_ff.getFeedforward();
 float pitch_mit_kd = 0.0f;
 float pitch_feedforward = 0.0f;
 // Pitch STOP 时不进入角度/速度控制计算。
 if (pitch_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_STOP)
{
    pitch_target_speed = 0.0f;
    pitch_control_output = 0.0f;
    pitch_remote_angle_encoder_pid.reset();
    pitch_keymouse_angle_encoder_pid.reset();
}
 // 角度模式：主控角度 PID 给目标速度，电机 MIT 内部计算速度纠偏力矩。
 else if (pitch_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_ANGLE)
{
    pitch_target_speed = is_keymouse
        ? pitch_keymouse_angle_encoder_pid.UpDate(pitch_error, 0.0f)
        : pitch_remote_angle_encoder_pid.UpDate(pitch_error, 0.0f);
    pitch_mit_kd = is_keymouse ? PITCH_KEYMOUSE_MIT_KD : PITCH_REMOTE_ANGLE_MIT_KD;
    pitch_feedforward = pitch_gravity_output;
}
 // 遥控速度档直接给 MIT 目标速度，保留重力补偿。
 else if (pitch_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_SPEED)
{
    pitch_target_speed = pitch_gimbal_fsm.Get_Control_Output();
    pitch_mit_kd = PITCH_REMOTE_SPEED_MIT_KD;
    pitch_feedforward = pitch_gravity_output;
}

/********************************************************************************** */
        // Kp=0：按本周期反馈估算 MIT 合成力矩，原有保护作用于总量而非仅前馈。
        const float pitch_estimated_torque =
            pitch_mit_kd * (pitch_target_speed - pitch_current_speed) + pitch_feedforward;
        pitch_control_output = pitch_estimated_torque;
        // 第一层：主控 +/-27 N*m 限幅，与用户给定的 J4340 力矩范围对齐。
        pitch_control_output = BSP::Motor::DM::ClampMitCommandValue(
            pitch_control_output, -J4340_TORQUE_LIMIT_NM, J4340_TORQUE_LIMIT_NM);

        // 第二层：Pitch 在下限拦负，保留上限正力矩支撑/制动。
        // 等于边界也拦截，朝范围内的制动/回退力矩允许通过；这里不主动规划减速轨迹。
        // 必须放在全部补偿之后，否则后加的惯量或重力前馈可能重新产生向外力矩。
        pitch_control_output = pitch_gimbal_fsm.Limit_Torque(pitch_control_output);

        // 方向限位生效期间始终关闭 MIT 速度反馈，避免电机随后自行生成向外力矩。
        // 上限仍按原配置允许正力矩支撑；范围内仅在幅值保护改变力矩时切换。
        const bool pitch_torque_limit_active = pitch_gimbal_fsm_config.limit_angle != 0U &&
            (pitch_current_angle <= pitch_gimbal_fsm_config.min_angle ||
             (pitch_gimbal_fsm_config.limit_upper_torque != 0U &&
              pitch_current_angle >= pitch_gimbal_fsm_config.max_angle));
        if (pitch_torque_limit_active || pitch_control_output != pitch_estimated_torque)
        {
            pitch_mit_kd = 0.0f;
            pitch_feedforward = pitch_control_output;
        }

        // 最终发送保护：Pitch 驱动未确认使能时力矩清零。
        if (!pitch_motor_ready)
        {
            pitch_target_speed = 0.0f;
            pitch_control_output = 0.0f;
            pitch_mit_kd = 0.0f;
            pitch_feedforward = 0.0f;
        }

            // MIT 参数顺序为电机槽、位置(rad)、速度(rad/s)、Kp、Kd、力矩(N*m)。
            // 正常由 MIT 速度反馈与前馈产生力矩；STOP 的 Kd/前馈均为 0。
            if (pitch_motor_ready)
                pitch_motor.ctrl_Mit(1, 0.0f, pitch_target_speed, 0.0f, pitch_mit_kd, pitch_feedforward);
        // UART10 的 VOFA 六通道 JustFloat 帧为 28 字节；当前每 4 个调度拍尝试 DMA 发送一次。
        // 发送发生在有效控制分支；实际频率取决于任务周期及故障跳过，不是固定每 10 ms。
        
        
            // if ((can2_tick % 4) == 0)
            // {
            //     vofa_send(
            //         pitch_target_angle,       // 通道 1：Pitch 目标角，rad
            //         pitch_current_angle,      // 通道 2：Pitch 选定反馈角，rad
            //         pitch_target_speed,              // 通道 3：Pitch 目标角速度，rad/s
            //         pitch_current_speed,       // 通道 4：Pitch 选定反馈角速度，rad/s
            //         pitch_target_angle,      // 通道 5：再次发送 Pitch 目标角，当前与通道 1 重复
            //         pitch_control_output      // 通道 6：Pitch 受限估算力矩，N*m（非实测）
            //     );
            // }
        

         }



        Watchdog_RecordHeartbeat(WATCHDOG_TASK_CAN_SEND);
        vTaskDelay(1); // 相对延时 1 个 FreeRTOS tick；实际循环周期还包含本周期执行时间
    }
}







// 辅助安全检查接口：本任务主循环另有按轴零力矩保护，不依赖此函数执行。
// 此函数更新在线/蜂鸣器状态并清除共享 remote 输入，没有直接发送零力矩 CAN 帧。
void SafetyCheck()
{
    // 遍历所有电机
    for (int i = 1; i <= 1; i++)
    {
        // 逻辑槽 1 对应物理 ID8 的 Pitch。
        if (!pitch_motor.isConnected(i, 8))
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
    uint8_t offline_motor = pitch_motor.getOfflineStatus();
	
    // 返回 0 表示都在线
    // 非零值表示驱动报告的掉线电机；当前只配置一台 Pitch J4340，返回值在本函数中未进一步使用。

}

//开vofa软件的justfloat模式
uint8_t send_str2[sizeof(float) * 31]; // 30 个数据和 1 个帧尾，共 124 字节
// 可变通道 JustFloat 发送：USB CDC-HS 路径，最多 30 个 float，另附 4 字节帧尾。
// USB 未就绪或正在发送时直接跳过；与下方六通道 UART10 DMA 发送接口不同。
void vofa_sendN(const float *data, uint8_t count)
{
    if (data == nullptr || count == 0)
    {
        return;
    }

    if (count > 30)
    {
        count = 30;
    }

    USBD_CDC_HandleTypeDef *hcdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceHS.pClassData);
    if (hcdc == nullptr || hcdc->TxState != 0)
    {
        return;
    }

    memcpy(send_str2, data, sizeof(float) * count);
    const uint32_t tail = 0x7F800000;
    memcpy(&send_str2[sizeof(float) * count], &tail, sizeof(tail));

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

// 读取物理 ID8（逻辑槽 1）的反馈缓存，更新状态快照和本周期控制反馈。
static void UpdatePitchMotorFeedback()
{
    dm4340_state[1].angle_deg      = pitch_motor.getAngleDeg(1);
    dm4340_state[1].angle_rad      = pitch_motor.getAngleRad(1);
    dm4340_state[1].velocity_rpm   = pitch_motor.getVelocityRpm(1);
    dm4340_state[1].velocity_rads  = pitch_motor.getVelocityRads(1);
    dm4340_state[1].current_a      = pitch_motor.getCurrent(1);
    dm4340_state[1].torque_nm      = pitch_motor.getTorque(1);
    dm4340_state[1].temperature    = pitch_motor.getTemperature(1);
    dm4340_state[1].delta_angle    = pitch_motor.getAddAngleDeg(1);

    pitch_current_angle = dm4340_state[1].angle_rad;
    pitch_current_speed = dm4340_state[1].velocity_rads;
}

// 读取 Yaw ID7（逻辑槽 1）的反馈缓存供调试观测。
static void UpdateYawMotorFeedback()
{
    dm4340_state[0].angle_deg = yaw_motor.getAngleDeg(1);
    dm4340_state[0].angle_rad = yaw_motor.getAngleRad(1);
    dm4340_state[0].velocity_rpm = yaw_motor.getVelocityRpm(1);
    dm4340_state[0].velocity_rads = yaw_motor.getVelocityRads(1);
    dm4340_state[0].torque_nm = yaw_motor.getTorque(1);
    dm4340_state[0].temperature = yaw_motor.getTemperature(1);
    dm4340_state[0].delta_angle = yaw_motor.getAddAngleDeg(1);
}
