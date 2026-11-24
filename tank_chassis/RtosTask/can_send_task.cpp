#include "can_send_task.hpp"
#include "main.h"
#include "watchdog_heartbeat.h"
#include "remote_task.hpp"
#include "up_stair.hpp"
#include "gantry_task.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "Alg/PID/pid.hpp"
#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "Alg/Filter/Filter.hpp"
#include <algorithm>
#include <math.h>
#include <string.h>
// 必须包含实现类的头文件，否则编译器不知道 FdcanDevice 是什么
#include "HAL/FDCAN/fdcan_hal.hpp"
// 确保包含这个头文件以获取 hfdcan1 / hfdcan2 / hfdcan3 的定义
#include "fdcan.h"
#include "../user/core/Alg/ChassisCalculation/MecanumForceControl.hpp"
#include "../user/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../fsm/chassis_fsm.hpp"
#include "../user/core/Alg/PowerControl-TestVersion/PowerControlTestVersion.hpp"
#include "../user/core/Alg/PowerControl/PowerControl.hpp"
#include "../user/core/Alg/PowerControl/ChassisPowerAllocator.hpp"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include "../communication/super_cupacitor.hpp"
#include "../communication/boards_communication_uart_485.hpp"

#include "../user/core/APP/Referee/RM_RefereeSystem.h"
#include "../user/core/Alg/UtilityFunction/SlopePlanning.hpp"
#include "../fsm/chassis_keyboard_fsm.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// X 轴目标速度上限，m/s；摇杆满量程线性映射至该值。
constexpr float CHASSIS_MAX_VX_M_S = 3.75f;
// Y 轴目标速度上限，m/s；摇杆满量程线性映射至该值。
constexpr float MAX_VY_M_S = 2.5f;
// 旋转目标角速度上限，rad/s；摇杆满量程映射至该值，所有旋转来源统一限幅。
constexpr float MAX_WZ_RAD_S = 2.5f;

QueueHandle_t motorspeedtargetQueue; // 声明一个全局队列句柄，用于在任务之间传递电机转速数据
QueueHandle_t motorCurrentDataQueue; // 声明一个全局队列句柄，用于在任务之间传递电机当前数据
QueueHandle_t chassisCurrentDataQueue; // 声明一个全局队列句柄，用于在任务之间传递底盘当前数据


MotorSpeedTarget_t motorSpeedTarget; // 定义一个全局变量来存储电机速度数据
MotorCurrentData_t motorCurrentData[4]; // 定义一个结构体来存储电机当前数据
chassisCurrentData_t chassisCurrentData; // 定义一个结构体来存储底盘当前数据

float yaw_offset_deg = 0.0f; // 云台回传的偏角，供观测。
ControlInputSnapshot chassis_input_watch{}; // Keil observation copy, written by the drive task.

// UART ISR 已校验完整的 Yaw 回包。
void Chassis485_ApplyData(const Chassis485Data &data)
{
    yaw_offset_deg = data.yaw_offset_deg;
}

ControlInputSnapshot GetControlInputSnapshot()
{
    ControlInputSnapshot input;
    taskENTER_CRITICAL();
    // 取时和读取 ISR 更新时间戳必须在同一临界区，避免新帧比采样时间更新。
    const uint32_t now_tick = HAL_GetTick();
    input.source = SelectControlInputSource(now_tick, local_remote_received,
                                            local_remote_last_tick);
    if (input.source == ControlInputSource::LOCAL) {
        // 本地 DBUS 摇杆是底盘坐标；同一帧提供档位、键盘和失联心跳。
        input.s1 = remoteData.s1;
        input.s2 = remoteData.s2;
        input.keyboard = local_remote_keyboard;
        input.mouse_x = local_remote_mouse_x;
        input.yaw_mouse_active = local_remote_yaw_mouse_active;
        input.keyboard_online = true;
        input.vx = remoteData.vy;
        input.vy = remoteData.vx;
        input.wz = remoteData.wz;
    }
    // 看门狗复位后，只有底盘接收机的新鲜双下帧才能解除停车锁定。
    if (g_watchdog_rearm_required)
    {
        if (input.source != ControlInputSource::NONE && input.s1 == 2U && input.s2 == 2U)
            g_watchdog_rearm_required = 0;
        else
            input = {};
    }
    taskEXIT_CRITICAL();
    return input;
}
StairCommand stair_command;
uint32_t stair_command_generation = 0U;

ChassisKeyboardFSM keyboard_fsm;

// 超级电容通信实例
Communication::SuperCapacitor supercap(500); // 500ms 超时阈值

// 板间回复使用的 42mm 热量诊断值
volatile uint16_t heat_42mm = 0;

using Alg::CalculationBase::MecanumForceControl;
// 三轴车体速度 PI 输出 Fx/Fy/Mz；几何、名义力矩换算和初始增益集中在该头文件。
MecanumForceControl chassis_force;

// 仅键鼠 vx/vy 启动限加速；遥控器直通，w 保持转向响应。
// 1 ms 周期下 0.008 m/s ≈ 8 m/s²；下降步长 100 使减速/松键目标立即到位。
constexpr float KEYBOARD_SPEED_RAMP_STEP = 0.008f;
Alg::Utility::SlopePlanning ramp_vx(KEYBOARD_SPEED_RAMP_STEP, 100.0f);
Alg::Utility::SlopePlanning ramp_vy(KEYBOARD_SPEED_RAMP_STEP, 100.0f);


// 模板参数 <N> 表示电机数量
// 4 个底盘电机
const uint8_t chassis_motor_idxs[4] = {1, 2, 3, 4};
BSP::Motor::Dji::GM3508<4> chassis_motor(0x200, chassis_motor_idxs, 0x200);

extern "C" {
typedef struct
{
    volatile uint32_t ok_count;
    volatile uint32_t fail_count;
    volatile uint32_t consecutive_fail_count;
    volatile uint32_t last_fail_tick;
    volatile uint8_t last_ok;
} ChassisMotorCanTxWatch;

volatile ChassisMotorCanTxWatch chassis_motor_can_tx_watch = {0, 0, 0, 0, 1};
}

static inline void ChassisMotorSendCANChecked()
{
    bool ok = chassis_motor.sendCAN();
    chassis_motor_can_tx_watch.last_ok = ok ? 1 : 0;

    if (ok)
    {
        chassis_motor_can_tx_watch.ok_count++;
        chassis_motor_can_tx_watch.consecutive_fail_count = 0;
    }
    else
    {
        chassis_motor_can_tx_watch.fail_count++;
        chassis_motor_can_tx_watch.consecutive_fail_count++;
        chassis_motor_can_tx_watch.last_fail_tick = HAL_GetTick();
    }
}

void ControlTask();


RemoteData_t ChassisData;



// 创建滤波器
// 参数：滤波系数 α（0-1），越小滤波越强
LPFFilter acc_filter_x(0.2f);
LPFFilter acc_filter_y(0.2f);
LPFFilter acc_filter_z(0.2f);
SecondOrderLPFFilter chassis_vx_filter(230.0f, 0.001f, 0.70710678f);
SecondOrderLPFFilter chassis_vy_filter(230.0f, 0.001f, 0.70710678f);
SecondOrderLPFFilter chassis_wz_filter(150.0f, 0.001f, 0.70710678f); // 旋转角速度反馈低通：截止频率 Hz、周期 s、阻尼比


ALG::PID::PID test_pid = {0.0f, 0.0f, 0.0f, 10000.0f, 5000.0f, 500.0f};

// 底盘状态机实例
Chassis_FSM chassis_fsm(8.0f, 0.0f, 0.0f, 100.0f, 25.0f, 2.5f); // PID 参数可以根据需要调整

float output = 0.0f; // PID 输出变量
float target = 10.0f; // 目标速度（示例值）
// ALG::PID::PID chassis_pid[3] = {
// 	{0.0f, 0.0f, 0.0f, 10000.0f, 5000.0f, 500.0f},    //底盘X轴速度PID
// 	{0.0f, 0.0f, 0.0f, 10000.0f, 5000.0f, 500.0f},    //底盘Y轴速度PID
// 	{0.0f, 0.0f, 0.0f, 10000.0f, 5000.0f, 500.0f}		 //底盘旋转速度PID
// };


void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6);
float motor_target_speed[4];
float motor_output[4];
float motor_output_pre[4];  // 功率控制前的电流，用于VOFA对比
float current_speed_rads[4];
float chassis_velocity_feedback[3] = {}; // 轮速正解，m/s、m/s、rad/s
float phase_comp = 0.0f; // Pitch 固定在车体上，此补偿保持为 0
    constexpr float yaw_offset_rad = 0.0f; // UART5 输入使用底盘坐标。
// 底盘预测功率 (W), 每个控制周期由 post_power 赋值更新 (只赋值不累加!)
// 其他文件读取: extern float chassis_power_pred;
float chassis_power_pred = 0.0f;

// ========== 功率校准扫频测试 ==========
Alg::PowerControlTestVersion::PowerControlTestVersion sweep_tester; // 扫频信号生成器
// Power model: P = K0 + K1*I + K2*abs(w) + K3*I*w + K4*I*I + K5*w*w
// 电机空载新.csv 与功率采集1新.csv 正确接线数据联合拟合，共 89785 条记录。
//   目标函数按组归一化：落地总权重 99.9%，空载总权重 0.1%。
//   P_total = 4*K0 + K1*ΣI + K2*Σabs(w) + K3*Σ(I*w) + K4*Σ(I²) + K5*Σ(w²)
//   落地/空载拟合 RMSE=2.4699/10.6442W，分段验证 RMSE=2.5356/10.7209W。
//   注: K0 已是每电机常数, 无需 -3*K0 修正 (已置 0)
float poly_coeffs[6] = {
     1.075387955765f,  // K0
    -0.231331346558f,  // K1
     0.004604969878f,  // K2, fabsf(w)
     0.018325841773f,  // K3
     0.123775233102f,  // K4
     0.000007258752f   // K5
};

// CorrectionConstant = 0.0f
// CorrectionConstant = 0.0f
// ========== 功率校准扫频测试结束 ==========

// ========== 底盘功率控制 (衰减电流法) ==========
// 临时调试速度 PID：false=直接使用 PID 电流输出，true=恢复原功率限制。
constexpr bool CHASSIS_POWER_LIMIT_ENABLED = true;
// 收腿及到位保持阶段的固定轮组预算，前后组不互借。
constexpr float STAIR_RETRACT_FRONT_POWER_SHARE = 0.75f;
// 超电配置 (根据实际硬件修改总容量)
const float SUPERCAP_TOTAL_CAPACITY = 2000.0f;                        // 超电总容量 (J)
const float ABUNDANCE_LINE          = SUPERCAP_TOTAL_CAPACITY * 0.8f; // 富足线 = 80%总容量
const float POVERTY_LINE            = 250.0f;                         // 贫困线 (生死线, <250J强制80%功率)

// 功率控制器 (衰减电流法执行层)
ALG::PowerControl::ChassisPowerAllocator chassis_power_allocator;
ALG::PowerControl::PowerControl<2> chassis_front_power_ctrl;
ALG::PowerControl::PowerControl<2> chassis_rear_power_ctrl;

// 能量环状态机                                        //富足线，贫困线，最小功率限制(防止功率太小跑不动)
ALG::PowerControl::EnergyRing energy_ring(ABUNDANCE_LINE, POVERTY_LINE, 56.0f);

// 策略层 (数据源仲裁: 裁判系统/超电 在线/离线判断)
ALG::PowerControl::PowerControlStrategy power_strategy(ABUNDANCE_LINE);

// 富足环 PID — 目标: 1600J(80%总容量), 仅energy≥1600J时被EnergyRing使用
//   能量越高 → AbundanceOut越负 → P_max越大 → 释放过剩能量, 防过充
//   能量<1600J → EnergyRing走中间分支, 直接P_ref, 此PID不参与
//   Kp=0.05 → 能量2000J时AbundanceOut≈-20W, P_max≈100W
ALG::PID::PID abundance_energy_pid(4.2361f, 0.0f, 0.0f, 80.0f, 80.0f, 0.0f);

// 贫困环 PID — 目标: 250J (POVERTY_LINE), Shift爆发模式生效
//   能量>>250J → PovertyOut为负 → P_max = P_ref - (负数) = P_ref + 增量 → 超功率！
//   Kp=0.05 → 能量1000J时PovertyOut≈-37.5W, P_max≈117.5W
ALG::PID::PID poverty_energy_pid(2.0754f, 0.0f, 0.0f, 80.0f, 80.0f, 0.0f);
// ========== 功率控制初始化结束 ==========

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t)
{
    HAL::FDCAN::Frame frame;
    HAL::FDCAN::IFdcanDevice *device = nullptr;
    if (hfdcan->Instance == FDCAN1) device = &HAL::FDCAN::get_fdcan_bus_instance().get_fdcan1();
    else if (hfdcan->Instance == FDCAN2) device = &HAL::FDCAN::get_fdcan_bus_instance().get_fdcan2();
    else if (hfdcan->Instance == FDCAN3) device = &HAL::FDCAN::get_fdcan_bus_instance().get_fdcan3();
    if (device) while (device->receive(frame)) {}
}
/******************************************************* */
    float wz_cmd = 0.0f;

extern "C" void can_send_task(void *argument)
{

    // 等待裁判系统就位（约需5秒），确保电机上电前裁判系统已就绪
    osDelay(1000);

    // 触发 FDCAN bus 初始化：HAL_FDCAN_Start + 激活中断通知
    HAL::FDCAN::get_fdcan_bus_instance();

    auto &fdcan1 = HAL::FDCAN::get_fdcan_bus_instance().get_device(HAL::FDCAN::FdcanDeviceId::HAL_Fdcan1);
    auto &fdcan2 = HAL::FDCAN::get_fdcan_bus_instance().get_device(HAL::FDCAN::FdcanDeviceId::HAL_Fdcan2);
    auto &fdcan3 = HAL::FDCAN::get_fdcan_bus_instance().get_device(HAL::FDCAN::FdcanDeviceId::HAL_Fdcan3);
/************************************************************************************** */
/************************************************************************************** */
    fdcan1.register_rx_callback([](const HAL::FDCAN::Frame &frame) {
        if (frame.id >= 0x201 && frame.id <= 0x204)
        {
        // 这是底盘电机的数据，交给 chassis_motor 解析
            chassis_motor.Parse(frame);
        }
        else if (frame.id == 0x777)
        {
       supercap.parse(frame); // 超级电容数据
        }

    });
    fdcan2.register_rx_callback([](const HAL::FDCAN::Frame &frame) {
        if (frame.id >= 0x05 && frame.id <= 0x06) {
            front_4340.Parse(frame);
        }
    });
/************************************************************************************** */
    // 板间输入改为 USART3；解析和心跳更新由 remote_task 的 UART lambda 接入。
/************************************************************************************** */
   fdcan3.register_rx_callback([](const HAL::FDCAN::Frame &frame) {
       if (frame.id >= 0x07 && frame.id <= 0x08) {
           rear_6248.Parse(frame);
       }
       else if ((frame.id == 0x205 || frame.id == 0x206) && frame.dlc == 8 && !frame.is_extended_id && !frame.is_remote_frame) {
           gantry_3508_motor.Parse(frame);
       }
       else if (frame.id == 0x09 && frame.dlc == 8 && !frame.is_extended_id && !frame.is_remote_frame && (frame.data[0] & 0x0f) == 9) {
           gantry_dm4310_motor.Parse(frame);
           gantry_watch.dm_status = frame.data[0] >> 4;
       }
   });
/************************************************************************************** */
    // 通知 FDCAN 接收回调已经准备好；机构电机命令统一由上台阶任务负责。
    dm_motor_control_ready = true;

    MotorCurrentData_t MotorCurrentData[4];
    
    
    float vx_gimbal = 0.0f;
    float vy_gimbal = 0.0f;
    float vx_body = 0.0f;
    float vy_body = 0.0f;

    // 初始化底盘状态机
    chassis_fsm.Init();
    keyboard_fsm.Init();

for (int i = 0; i < 4; i++)
{
    chassis_motor.setCAN((int16_t)0, i + 1);
}
ChassisMotorSendCANChecked();

// 在循环前声明
Enum_Chassis_Mode last_mode = CHASSIS_STOP; //为了不疯车
ControlInputSource last_input_source = ControlInputSource::NONE;
// 原功率辨识 VOFA 的状态，恢复采集时与下方原发送段一起启用。
// uint32_t last_power_meter_sequence = 0U;
// ALG::PowerControl::PowerControlBase<4> power_identification_model;

    //IMU的变量
    IMUData_t IMUData;           // IMU 数据结构体
osDelay(500);
    for (;;)
    {
         // 在输入采样前记录安全代次，防止被机构任务的停车处理抢占后重放旧键。
         taskENTER_CRITICAL();
         const uint32_t stair_sample_generation = stair_command_generation;
         taskEXIT_CRITICAL();
         const uint32_t now_tick = HAL_GetTick();
         float force_target[3] = {};
         float force_feedback[3] = {};
         bool force_control_active = false;
         const ControlInputSnapshot input = GetControlInputSnapshot();
         chassis_input_watch = input;
         const bool source_changed = input.source != last_input_source;
         last_input_source = input.source;
         const bool keyboard_mode =
             (input.s1 == 3U) && (input.s2 == 3U);
         const bool keyboard_online =
             input.keyboard_online && !source_changed;

         keyboard_fsm.Update(
             input.keyboard,
             keyboard_mode,
             keyboard_online,
             now_tick);
        const KeyboardMotionCommand& keyboard_cmd =
            keyboard_fsm.GetCommand();


         const StairModePolicy stair_policy = EvaluateStairModePolicy(
             input.s1, input.s2, input.source != ControlInputSource::NONE,
             keyboard_online);
         // 一次 B 同时发布两个明确目标，任务不会读到前后不一致的半帧命令。
         taskENTER_CRITICAL();
         if (stair_sample_generation == stair_command_generation)
         {
             UpdateStairCommand(stair_command,
                                !stair_policy.zero_all_torque && !source_changed,
                                keyboard_cmd.imu_mode_request,
                                keyboard_cmd.normal_mode_request,
                                keyboard_cmd.stair_toggle);
             if (!keyboard_cmd.valid)
             {
                 stair_command.front_target_requested = false;
             }
         }
         // 拨杆退出双中键鼠或键盘失效时，立即恢复四轮统一缩放。
         const bool stair_retract_power_split =
             keyboard_cmd.valid && stair_command.mode == StairMode::IMU &&
             stair_command.rear_retract_requested;
         taskEXIT_CRITICAL();

         //获取底盘旋转速度
         ChassisData.vx = input.vx * CHASSIS_MAX_VX_M_S;
         ChassisData.vy = input.vy * MAX_VY_M_S;
         ChassisData.wz = 0.0f; // 有效控制周期由状态机输出更新。
         ChassisData.s1 = input.s1;
         ChassisData.s2 = input.s2;

                 //获取底盘旋转速度
         //ChassisData.wz = remoteController.get_right_x() * MAX_WZ_RAD_S;


         // 超级电容在线状态更新
         supercap.updateOnlineStatus();
         // 每周期采集轮轴反馈，双下停车和输入失联时也供 VOFA 显示。
         current_speed_rads[0] = chassis_motor.getVelocityRads(1) / MecanumForceControl::REDUCTION_RATIO;
         current_speed_rads[1] = chassis_motor.getVelocityRads(2) / MecanumForceControl::REDUCTION_RATIO;
         current_speed_rads[2] = chassis_motor.getVelocityRads(3) / MecanumForceControl::REDUCTION_RATIO;
         current_speed_rads[3] = chassis_motor.getVelocityRads(4) / MecanumForceControl::REDUCTION_RATIO;
         bool wheel_feedback_valid = true;
         for (uint8_t i = 0; i < 4; ++i)
         {
             const bool online = chassis_motor.isConnected(i + 1, i + 1);
             wheel_feedback_valid = wheel_feedback_valid && online && isfinite(current_speed_rads[i]);
         }
         if (input.source == ControlInputSource::NONE || source_changed || !wheel_feedback_valid)
    {
        for (int i = 0; i < 4; i++) 
        {
        motor_target_speed[i] = 0;
        motor_output[i] = 0;
        motor_output_pre[i] = 0;
        chassis_motor.setCAN((int16_t)0, i + 1);
        }
         chassis_force.Reset();
         ramp_vx.TIM_Calculate_PeriodElapsedCallback(0.0f, 0.0f);
         ramp_vy.TIM_Calculate_PeriodElapsedCallback(0.0f, 0.0f);
         chassis_fsm.Get_Follow_PID().reset();
         chassis_vx_filter.Reset(0.0f);
         chassis_vy_filter.Reset(0.0f);
         chassis_wz_filter.Reset(0.0f);
 ChassisMotorSendCANChecked();
    }
    else {
             ControlTask(); // 获取电机当前数据并更新全局 变量

            // ==================== 功率校准扫频模式 ====================
            // 遥控器 S1 拨到最上方 → 进入校准模式，电机4自动跑扫频
            // S1 拨到其他位置 → 恢复正常遥控模式
    // if (remoteController.get_s1() == 1)
    // {
    //     // 1. 生成扫频目标转速 (rad/s, 电机转子端)
    //     //    覆盖: 幅值 0~800 rad/s, 频率 1~4 Hz
    //     float sweep_target = sweep_tester.SinExpected(0.001f, 20.0f, 800.0f, 4.0f);

    //     // 2. 电机4: PID 速度闭环跟踪扫频信号
    //     //    getVelocityRads = 转子转速 (rad/s), sweep_target 也是转子转速, 直接对齐
    //     motor_output[3] = motor_pid[3].UpDate(sweep_target,
    //                                           chassis_motor.getVelocityRads(4));
    //     chassis_motor.setCAN((int16_t)motor_output[3], 4);

    //     // 3. 电机1/2/3: 停转
    //     for (int i = 0; i < 3; i++)
    //     {
    //         motor_pid[i].reset();
    //         chassis_motor.setCAN((int16_t)0, i + 1);
    //     }
    //     chassis_motor.sendCAN();

    //     // 4. 采集数据 → VOFA+ (JustFloat 模式)
    //     //    PowerData 来自 UART7 功率计, 在 remote_task.cpp 中断中更新
    //     float I     = chassis_motor.getCurrent(4);           // 电流 (A)
    //     float omega = chassis_motor.getVelocityRads(4);      // 转子转速 (rad/s)
    //     float P_in  = PowerData.power;                       // 功率计功率 (W)
    //     // Power model: P = K0 + K1*I + K2*abs(w) + K3*I*w + K4*I² + K5*w²
    //     float P_est = poly_coeffs[0]
    //                 + poly_coeffs[1] * I
    //                 + poly_coeffs[2] * omega
    //                 + poly_coeffs[3] * I * omega
    //                 + poly_coeffs[4] * I * I
    //                 + poly_coeffs[5] * omega * omega;
    //     // VOFA列: [P_in(功率计), omega, I, sweep_target, P_est(模型), 0]
    //     vofa_send9(P_in, omega, I, sweep_target, P_est, 0.0f, 0.0f, 0.0f, 0.0f);

    //     osDelay(1);
    //                 }
            // ==================== 校准模式结束 ====================


           // 当前来源完整且新鲜才允许退出 STOP；本地输入以车体为坐标系。
           chassis_fsm.StateUpdate(
               input.s1, input.s2, input.source != ControlInputSource::NONE,
               keyboard_cmd.valid);
           wz_cmd = chassis_fsm.Get_wz_cmd(
               yaw_offset_rad, input.wz, input.mouse_x, MAX_WZ_RAD_S, input.yaw_mouse_active);
           ChassisData.wz = wz_cmd;

        // 2. 获取电机当前反馈 (当前轮速)
        phase_comp = 0.0f; // Pitch 固定在车体上，无相对云台旋转的相位补偿。
            // --- A. 运动学逆解算：底盘速度 -> 4个轮子的目标转速 ---
            // 直接解算遥控器给出的目标值
            if (keyboard_cmd.valid)
            {
                vx_gimbal = keyboard_cmd.vx;
                vy_gimbal = keyboard_cmd.vy;
            }
            else
            {
                vx_gimbal = input.vx;
                vy_gimbal = input.vy;
            }
float compensated_angle = yaw_offset_rad + phase_comp;
vx_body = (vx_gimbal * cosf(compensated_angle) + vy_gimbal * sinf(compensated_angle)) * CHASSIS_MAX_VX_M_S;
vy_body = (-vx_gimbal * sinf(compensated_angle) + vy_gimbal * cosf(compensated_angle)) * MAX_VY_M_S;
/**************************************************************** */
if (chassis_fsm.Get_Mode() != last_mode)
{
    // 模式切换：清除三个车体速度环的积分。
    chassis_force.Reset();
    last_mode = chassis_fsm.Get_Mode();
}
/**************************************************************** */
           // STOP 模式：直接清零，跳过后续控制，防疯车
if (chassis_fsm.Get_Mode() == CHASSIS_STOP)
{
   for (int i = 0; i < 4; i++)
   {
       motor_target_speed[i] = 0;
       motor_output[i] = 0;
       motor_output_pre[i] = 0;
       chassis_motor.setCAN((int16_t)0, i + 1);
   }
   chassis_force.Reset();
   ramp_vx.TIM_Calculate_PeriodElapsedCallback(0.0f, 0.0f);
   ramp_vy.TIM_Calculate_PeriodElapsedCallback(0.0f, 0.0f);
   chassis_vx_filter.Reset(0.0f);
   chassis_vy_filter.Reset(0.0f);
   chassis_wz_filter.Reset(0.0f);
   ChassisMotorSendCANChecked();
 }
 else
 {
/**************************************************************** */
          // 四轮车体力控：目标车速 -> Fx/Fy/Mz -> 轮端力矩 -> 电流。
          chassis_force.ForwardKinematics(current_speed_rads, chassis_velocity_feedback);
          force_feedback[0] = chassis_vx_filter.Filter(chassis_velocity_feedback[0]);
          force_feedback[1] = chassis_vy_filter.Filter(chassis_velocity_feedback[1]);
          force_feedback[2] = chassis_wz_filter.Filter(chassis_velocity_feedback[2]);
          // 三轴目标独立限幅，直接使用 m/s、rad/s，不再乘旧操作倍率。
          vx_body = std::clamp(vx_body, -CHASSIS_MAX_VX_M_S, CHASSIS_MAX_VX_M_S);
          vy_body = std::clamp(vy_body, -MAX_VY_M_S, MAX_VY_M_S);
          wz_cmd = std::clamp(wz_cmd, -MAX_WZ_RAD_S, MAX_WZ_RAD_S);
          if (keyboard_cmd.valid)
          {
              // 反向时先撤销旧方向目标，再从零向新方向加速。
              if (vx_body * ramp_vx.GetOut() < 0.0f)
              {
                  ramp_vx.TIM_Calculate_PeriodElapsedCallback(0.0f, 0.0f);
              }
              if (vy_body * ramp_vy.GetOut() < 0.0f)
              {
                  ramp_vy.TIM_Calculate_PeriodElapsedCallback(0.0f, 0.0f);
              }
              // 使用上次规划输出，避免实测反馈同步逻辑跳过启动斜坡。
              ramp_vx.TIM_Calculate_PeriodElapsedCallback(vx_body, ramp_vx.GetOut());
              ramp_vy.TIM_Calculate_PeriodElapsedCallback(vy_body, ramp_vy.GetOut());
              vx_body = ramp_vx.GetOut();
              vy_body = ramp_vy.GetOut();

          }
          else
          {
              // 遥控器直接使用目标；清空键鼠规划，防止再次进入时沿用旧速度。
              ramp_vx.TIM_Calculate_PeriodElapsedCallback(0.0f, 0.0f);
              ramp_vy.TIM_Calculate_PeriodElapsedCallback(0.0f, 0.0f);
          }
          force_target[0] = std::clamp(vx_body, -CHASSIS_MAX_VX_M_S, CHASSIS_MAX_VX_M_S);
          force_target[1] = std::clamp(vy_body, -MAX_VY_M_S, MAX_VY_M_S);
          force_target[2] = wz_cmd;
          // 收腿分组限功率期间关闭同步；当周期清除上一阶段残留，保留车体力控。
          if (stair_retract_power_split)
          {
              for (unsigned i = 0; i < 4; ++i)
              {
                  chassis_force.sync_target_speed[i] = 0.0f;
                  chassis_force.sync_error[i] = 0.0f;
                  chassis_force.sync_current[i] = 0.0f;
              }
          }
          else
          {
               chassis_force.UpdateWheelSynchronization(current_speed_rads, force_target, force_feedback);
          }
          chassis_force.Update(force_target, force_feedback, motor_output,
                                stair_retract_power_split || !CHASSIS_POWER_LIMIT_ENABLED);
          force_control_active = true;
          // 仅保留目标轮速供观察，输出不再经过四个轮速 PID。
          chassis_force.InverseKinematics(force_target, motor_target_speed);
	// 保存功率控制前的电流，用于VOFA对比衰减前后
		memcpy(motor_output_pre, motor_output, sizeof(motor_output_pre));

	// ========== 底盘功率控制 (衰减电流法) ==========
	if (CHASSIS_POWER_LIMIT_ENABLED)
	{
	    // 1. 策略层: 数据源仲裁 (裁判系统 / 超电 在线判断)
	    power_strategy.Update(
	        supercap.isOnline(),                              // 超电在线状态
	        !RM_RefereeSystemDirFlag,                         // 裁判系统在线
	        (float)ext_power_heat_data_0x0201.chassis_power_limit,  // 裁判功率上限 (W)
	        (float)ext_power_heat_data_0x0202.chassis_power_buffer, // 缓冲能量 (J)
	        supercap.getEnergy()                              // 超电剩余能量 (J)
	    );

	    float current_energy = power_strategy.GetInputEnergy();  //超电剩余能量
	    float ref_limit      = power_strategy.GetInputLimit();   //

	    // 2. 富足环 PID — 目标=ABUNDANCE_LINE(1600J), 仅energy≥1600J时被EnergyRing使用
	    //    能量>1600J → AbundanceOut<0 → P_max放大(防过充)
	    //    能量<1600J → EnergyRing走中间分支, 直接P_ref, 此PID不参与
	    // 对能量开根号：E = ½CU²，√E ∝ U，保证全电压区间PID响应一致
	    float abundance_out = abundance_energy_pid.UpDate(sqrtf(ABUNDANCE_LINE), sqrtf(current_energy));

	    // 3. 贫困环 PID — 目标=POVERTY_LINE(250J), Shift爆发模式用
	    //    能量>>250J → PovertyOut为负 → P_max = P_ref - (负数) = 超功率!
	    float poverty_out = poverty_energy_pid.UpDate(sqrtf(POVERTY_LINE), sqrtf(current_energy));

	    // 4. 能量环状态机: 根据能量状态动态计算 PowerMax
	    energy_ring.energyring(
	        abundance_out,      // 富足环输出
	        poverty_out,        // 贫困环输出
	        ref_limit,          // 裁判系统功率上限
	        current_energy,     // 当前能量 (策略层可能伪造)
             keyboard_cmd.valid && keyboard_cmd.shift_pressed,
	        false               // isPower (充电模式暂未实现)
	    );
	    float PowerMax = energy_ring.GetPowerMax();

	    // 5. 转为物理电流 → 调用电流衰减法
	    //    力控输出 [-16384, 16384] 映射到 [-20A, 20A] (GM3508)
	    float I[4], V[4], I_other[4];
	    for (int i = 0; i < 4; i++) {
	        I[i]       = motor_output[i] * (20.0f / 16384.0f);     // raw → 物理电流 (A)
	        V[i]       = chassis_motor.getVelocityRads(i + 1);       // 转子转速 (rad/s)
	        I_other[i] = 0.0f;                                       // 无前馈
	    }

	    if (stair_retract_power_split)  //. 爬台阶收回阶段，采用前后轮分组预算，前75%，后25%，不互借。
	    {
	        // 力控与同步的合成电流按组限幅，每组预算包含本组两轮的基础损耗。
	        chassis_front_power_ctrl.DecayingCurrent(
	            I, V, poly_coeffs, I_other, 0.0f, PowerMax * STAIR_RETRACT_FRONT_POWER_SHARE);
	        chassis_rear_power_ctrl.DecayingCurrent(
	            I + 2, V + 2, poly_coeffs, I_other + 2, 0.0f,
	            PowerMax * (1.0f - STAIR_RETRACT_FRONT_POWER_SHARE));
	        for (int i = 0; i < 2; i++) {
	            motor_output[i] = chassis_front_power_ctrl.getCurrentCalculate(i) * (16384.0f / 20.0f);
	            motor_output[i + 2] = chassis_rear_power_ctrl.getCurrentCalculate(i) * (16384.0f / 20.0f);
	        }
	        // 作为保守的饱和标志反馈 PI；不是四轮实际共用一个缩放系数。
	        chassis_force.ApplyOutputScale(fminf(
	            chassis_front_power_ctrl.getEta(), chassis_rear_power_ctrl.getEta()));
	    }
	    else
	    {
            float translation[4], rotation[4];
            chassis_force.GetCurrentComponents(translation, rotation);
            chassis_power_allocator.Allocate(
                translation, rotation, chassis_force.sync_current, V, poly_coeffs,
                PowerMax, MecanumForceControl::MAX_CURRENT);
            for (unsigned i = 0; i < 4; ++i)
                motor_output[i] = chassis_power_allocator.current[i] * MecanumForceControl::RAW_PER_AMP;
            chassis_force.ApplyAllocationScale(
                chassis_power_allocator.alpha, chassis_power_allocator.beta);
	    }
	}
	// ========== 功率控制结束 ==========
		// 用衰减后的电流重算功率，用于VOFA对比预测功率 vs 衰减后实际功率
		float post_power = 0.0f;   // 局部变量, 每周期清零再累加 (勿改全局, 否则跨周期累积)
		for (int i = 0; i < 4; i++) {
		    float I_post = motor_output[i] * (20.0f / 16384.0f);
		    float w = chassis_motor.getVelocityRads(i + 1);
		    post_power += poly_coeffs[0] + poly_coeffs[1]*I_post + poly_coeffs[2]*fabsf(w)
		                + poly_coeffs[3]*I_post*w + poly_coeffs[4]*I_post*I_post + poly_coeffs[5]*w*w;
		}
		chassis_power_pred = post_power;   // 保存预测功率到全局变量 (赋值, 非累加!)
		// post_power += -3.0f * poly_coeffs[0]; // 新系数已无偏, 不再修正

            for (int i = 0; i < 4; i++) {
    // motor_target_speed[i] = ik.GetMotor(i);
     //motor_output[i] = motor_pid[i].UpDate(motor_target_speed[i], current_speed_rads[i]);
     chassis_motor.setCAN((int16_t)motor_output[i], i + 1);
 }
//		chassis_motor.setCAN((int16_t)motor_output[0],  1);
// 		chassis_motor.setCAN((int16_t)motor_output[1],  2);
//		chassis_motor.setCAN((int16_t)motor_output[2],  3);
//		chassis_motor.setCAN((int16_t)motor_output[3],  4);

 

 ChassisMotorSendCANChecked();  //就是将chassis_motor.sendCAN()打包成可以检查返回值的函数

           
 }
        



       
    }

        // 力控 VOFA：USB CDC / JustFloat，每轮任务尝试发送一次。
        float stopped_velocity[3] = {};
        if (!force_control_active && wheel_feedback_valid)
        {
            // STOP/输入暂停时继续采集当前正解原值。
            chassis_force.ForwardKinematics(current_speed_rads, stopped_velocity);
        }
        const float *raw_velocity = force_control_active ? chassis_velocity_feedback : stopped_velocity;

        float force_vofa_channels[24];
        force_vofa_channels[0] = force_target[0];   // vx 目标 (m/s)
        force_vofa_channels[1] = raw_velocity[0];   // vx 正解原值
        force_vofa_channels[2] = force_feedback[0]; // vx 滤波反馈
        force_vofa_channels[3] = force_target[1];   // vy 目标 (m/s)
        force_vofa_channels[4] = raw_velocity[1];   // vy 正解原值
        force_vofa_channels[5] = force_feedback[1]; // vy 滤波反馈
        force_vofa_channels[6] = force_target[2];   // wz 目标 (rad/s)
        force_vofa_channels[7] = raw_velocity[2];   // wz 正解原值
        force_vofa_channels[8] = force_feedback[2]; // wz 滤波反馈
        force_vofa_channels[9] = chassis_force.force[0];  // Fx 请求 (N)
        force_vofa_channels[10] = chassis_force.force[1]; // Fy 请求 (N)
        force_vofa_channels[11] = chassis_force.force[2]; // Mz 请求 (N·m)
        // 状态：1 力控运行，0 停车或输入暂停，-1 轮反馈无效。
        force_vofa_channels[12] = wheel_feedback_valid ? static_cast<float>(force_control_active) : -1.0f;
        force_vofa_channels[13] = static_cast<float>(now_tick) * 0.001f; // 板端时间 (s)
        force_vofa_channels[14] = chassis_force.sync_current[0]; // 1 号轮同步校正电流 (A)
        force_vofa_channels[15] = motor_output[0] / MecanumForceControl::RAW_PER_AMP; // 1 号轮最终电流指令 (A)

        // 四轮目标和实际角速度均使用带符号的轮端 rad/s。
        force_vofa_channels[16] = motor_target_speed[0]; // 左前目标
        force_vofa_channels[17] = wheel_feedback_valid ? current_speed_rads[0] : 0.0f; // 左前实际
        force_vofa_channels[18] = motor_target_speed[1]; // 右前目标
        force_vofa_channels[19] = wheel_feedback_valid ? current_speed_rads[1] : 0.0f; // 右前实际
        force_vofa_channels[20] = motor_target_speed[2]; // 右后目标
        force_vofa_channels[21] = wheel_feedback_valid ? current_speed_rads[2] : 0.0f; // 右后实际
        force_vofa_channels[22] = motor_target_speed[3]; // 左后目标
        force_vofa_channels[23] = wheel_feedback_valid ? current_speed_rads[3] : 0.0f; // 左后实际
        vofa_sendN(force_vofa_channels, 24);

        // 原功率辨识 VOFA 暂停，保留代码；实际功率限制和功率计接收不受影响。
        /*
        // 功率辨识：每个功率计新帧最多采集一次，STOP 时也保留真实反馈。
        float power_channels[15] = {};
        taskENTER_CRITICAL();
        const uint32_t power_sequence = power_meter_sequence;
        const bool new_power_sample = power_meter_valid &&
            power_sequence != last_power_meter_sequence;
        if (new_power_sample)
        {
            power_channels[0] = PowerData.power;
            for (uint8_t i = 0; i < 4; ++i)
            {
                power_channels[1 + 2 * i] = chassis_motor.getCurrent(i + 1);
                // 此对象减速比为 1，直接取转子 rad/s，不能用除过 15.8 的轮速。
                power_channels[2 + 2 * i] = chassis_motor.getVelocityRads(i + 1);
                if (chassis_motor.isConnected(i + 1, i + 1))
                {
                    power_channels[13] += static_cast<float>(1U << i);
                }
            }
            power_channels[10] = static_cast<float>(power_meter_tick) * 0.001f;
            power_channels[11] = PowerData.voltage;
            power_channels[12] = PowerData.current;
            power_channels[14] = static_cast<float>(power_sequence);
        }
        taskEXIT_CRITICAL();
        if (new_power_sample)
        {
            last_power_meter_sequence = power_sequence;
            for (uint8_t i = 0; i < 4; ++i)
            {
                power_identification_model.SetMotorParameters(
                    i, power_channels[1 + 2 * i], power_channels[2 + 2 * i], poly_coeffs);
                power_channels[9] += power_identification_model.GetPowerCalculate(i);
            }
            vofa_sendN(power_channels, 15);
        }
        */

        // 更新待发送的裁判数据，由 remote_task 调用 Chassis485_Send() 周期发送。
        Chassis485_UpdateReferee(
            ext_power_heat_data_0x0201.shooter_barrel_cooling_value,   // 枪管冷却值
            ext_power_heat_data_0x0201.shooter_barrel_heat_limit,      // 枪管热量上限
            (heat_42mm = ext_power_heat_data_0x0202.shooter_id1_42mm_cooling_heat), // 最新转发热量
            RM_RefereeSystemDirFlag ? 0 : 1                            // 裁判系统在线标志 (1=在线)
        );

        // vofa_send(
        //     chassis_motor.getVelocityRpm(1), chassis_motor.getVelocityRpm(2),
        //     chassis_motor.getVelocityRpm(3), chassis_motor.getVelocityRpm(4),
        //     0,0
        // );


        // 向超级电容发送控制数据
        supercap.sendToSuperCap(
            (float)ext_power_heat_data_0x0201.chassis_power_limit,   // 等级功率 (W)
            0,                                                        // 超电指令: 0=开启
            (float)ext_power_heat_data_0x0202.chassis_power_buffer,   // 缓冲能量 (J)
            supercap.isOnline() ? 1 : 0,                              // 超电在线标志
            RM_RefereeSystemDirFlag ? 0 : 1                           // 裁判系统在线标志
        );

Watchdog_RecordHeartbeat(WATCHDOG_TASK_CAN_SEND);
osDelay(1); 
    }
}



void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6)
{
    // DMA 异步读取，缓冲区需在函数返回后保持有效。
    static uint8_t send_str2[sizeof(float) * 6 + 4];
    auto &vofa_uart = HAL::UART::get_uart_bus_instance().get_uart10();
    // 上一帧未发送完成时跳过本次，避免改写 DMA 正在读取的数据。
    if (vofa_uart.get_handle()->gState != HAL_UART_STATE_READY)
    {
        return;
    }

    const float values[6] = {x1, x2, x3, x4, x5, x6};
    const uint8_t frame_tail[4] = {0x00, 0x00, 0x80, 0x7F};
    memcpy(send_str2, values, sizeof(values));
    memcpy(send_str2 + sizeof(values), frame_tail, sizeof(frame_tail));

    HAL::UART::Data vofa_tx_data{send_str2, sizeof(send_str2)};
    vofa_uart.transmit_dma(vofa_tx_data);
}


void vofa_sendN(const float *data, uint8_t count)
{
    if (data == nullptr || count == 0 || count > 30)
    {
        return;
    }

    // USB 异步发送：缓冲区必须持久有效，且忙时不能改写。
    static uint8_t send_str2[sizeof(float) * 30 + 4];
    const uint8_t frame_tail[4] = {0x00, 0x00, 0x80, 0x7F};
    // USB IRQ 优先级为 5；短临界区防止检查后被断连中断释放 pClassData。
    taskENTER_CRITICAL();
    USBD_CDC_HandleTypeDef *hcdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceHS.pClassData);
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED || hcdc == nullptr || hcdc->TxState != 0)
    {
        taskEXIT_CRITICAL();
        return;
    }

    memcpy(send_str2, data, sizeof(float) * count);
    memcpy(send_str2 + sizeof(float) * count, frame_tail, sizeof(frame_tail));
    CDC_Transmit_HS(send_str2, static_cast<uint16_t>(sizeof(float) * (count + 1)));
    taskEXIT_CRITICAL();
}


 void ControlTask() {
    for (int i = 0; i < 4; i++) {
        uint8_t motor_id = i + 1; // 电机逻辑 ID 通常从 1 开始
        
        motorCurrentData[i].angle_deg   = chassis_motor.getAngleDeg(motor_id);
        motorCurrentData[i].angle_rad   = chassis_motor.getAngleRad(motor_id);
        motorCurrentData[i].last_angle  = chassis_motor.getLastAngleDeg(motor_id);
        motorCurrentData[i].delta_angle = chassis_motor.getAddAngleDeg(motor_id);
        motorCurrentData[i].speed_rpm   = chassis_motor.getVelocityRpm(motor_id);
        motorCurrentData[i].speed_rads  = chassis_motor.getVelocityRads(motor_id);   //角速度，用这个控制电机
        motorCurrentData[i].current     = chassis_motor.getCurrent(motor_id);
        motorCurrentData[i].temp        = chassis_motor.getTemperature(motor_id);
        motorCurrentData[i].torque      = chassis_motor.getTorque(motor_id);
    }
}






