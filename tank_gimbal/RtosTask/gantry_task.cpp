#include "gantry_task.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "remote_control_task.hpp"
#include "../communication_between_boards/input_dispatcher.hpp"
#include "../feeder_fsm/gantry_fsm.hpp"
#include "../feeder_fsm/motor_recovery_fsm.hpp"
#include "../user/core/BSP/switch/switch.hpp"
#include "../user/core/Alg/PID/pid.hpp"
#include <algorithm>
#include <cmath>

// FDCAN1：3508 物理 ID 5、6，反馈 ID 0x205、0x206。
// 0x1FF 命令帧内槽位 1、2 对应物理 ID 5、6；setCAN() 传槽位 1、2。
// 减速比 19：累计角度和速度均使用输出轴端单位。
const uint8_t gantry_3508_motor_idxs[2] = {5, 6};
BSP::Motor::Dji::GM3508<2> gantry_3508_motor(
    0x200, gantry_3508_motor_idxs, 0x1FF, HAL::FDCAN::FdcanDeviceId::HAL_Fdcan1, 19.0);

// FDCAN3：DM4310 控制/反馈 ID 均为 9；对象内部仅有逻辑通道 1。
BSP::Motor::DM::J4310<1> gantry_dm4310_motor(
    0x00, {9}, {9}, HAL::FDCAN::FdcanDeviceId::HAL_Fdcan3);

volatile GantryWatch gantry_watch{};

namespace
{
Class_Gantry_FSM gantry_fsm;
BSP::Switch::MicroSwitch limits[4] = {
    {GPIOE, GPIO_PIN_13}, {GPIOE, GPIO_PIN_9}, {GPIOA, GPIO_PIN_2},
    {GPIOA, GPIO_PIN_3}
};
// 依次为 Kp、Ki、Kd、输出限幅、积分状态限幅、积分分离阈值。
// 3508 输出轴速度误差 rad/s -> 原始电流指令；409.6 对应 0.5 A。
// 以下为空载低速起始参数，尚未实机整定；Ki 按每次调用累加（周期 2 ms）。
// 从转子端换算：速度环 Kp/Ki 乘 19，积分状态限幅除以 19，保持相同实际电流响应。
ALG::PID::PID lift_move_pid[2] = {
    {1900.0f, 0.0f, 0, 16384.0f, 0.0f, 0}, // 3508 ID5 升降运动速度环
    {1900.0f, 0.0f, 0, 16384.0f, 0.0f, 0}  // 3508 ID6 升降运动速度环
};

// 位置保持角度外环：误差 rad -> 目标速度 rad/s；六参数顺序同上。
// 输出限幅直接使用各轴速度上限，让 PID 在限幅时处理积分。
ALG::PID::PID hold_angle_pid[3] = {
    {2.0f, 0.001f, 0, GantryTuning::LIFT_ID5_SPEED_RAD_S, 100.0f / 19.0f, 0}, // 3508 ID5
    {2.0f, 0.001f, 0, GantryTuning::LIFT_ID6_SPEED_RAD_S, 100.0f / 19.0f, 0}, // 3508 ID6
    {1.0f, 0.001f, 0, GantryTuning::DM_SPEED_RAD_S, 10, 0}        // DM ID9
};

ALG::PID::PID lift_hold_pid[2] = {
    {1900.0f, 0.0f, 0, 16384.0f, 0.0f, 0}, // 3508 ID5 位置保持速度内环
    {1900.0f, 0.0f, 0, 16384.0f, 0.0f, 0}  // 3508 ID6 位置保持速度内环
};

bool Gantry_ConfigValid()
{
    using namespace GantryTuning;
    if (!Class_Gantry_FSM::ConfigValid() || DM_SPEED_RAD_S > 30 ||
        !std::isfinite(DM_MOVE_KD) || DM_MOVE_KD <= 0 || DM_MOVE_KD > 5 ||
        !std::isfinite(DM_HOLD_KD) || DM_HOLD_KD <= 0 || DM_HOLD_KD > 5 ||
        !std::isfinite(DM_TORQUE_LIMIT_NM) || DM_TORQUE_LIMIT_NM <= 0 ||
        DM_TORQUE_LIMIT_NM > 3)
        return false;
    return true;
}
}

extern "C" void gantry_task(void *argument)
{
    (void)argument;
    using namespace GantryTuning;
    // 初始化：校验标定和控制参数。
    const bool configured = Gantry_ConfigValid();
    gantry_watch.configured = configured;
    MotorRecoveryFSM dm_recovery_fsm;
    dm_recovery_fsm.Init(HAL_GetTick());
    for(;;)
    {
        // 读取限位、遥控输入和三台电机反馈快照。
        float velocity[3];
        float lift_current_a[2];
        double angle_deg[3];
        bool limit_pressed[4];
        for (unsigned i = 0; i < 4; ++i)
        {
            limits[i].update();
            gantry_watch.limit_raw[i] = limits[i].getRawLevel() == GPIO_PIN_SET;
            limit_pressed[i] = limits[i].getPress();
            gantry_watch.limit[i] = limit_pressed[i];
        }
        // 遥控在线查询可能触发蜂鸣器请求，放在临界区外。
        const bool remote_online = remoteController.isConnected();
        taskENTER_CRITICAL();
        const uint32_t now_ms = HAL_GetTick();
        const bool control_enabled = configured && remote_online &&
            remoteController.get_s1() == BSP::REMOTE_CONTROL::RemoteController::MIDDLE &&
            remoteController.get_s2() == BSP::REMOTE_CONTROL::RemoteController::MIDDLE &&
            input_dispatcher.GetSource() == InputSource::KeyMouse;
        const bool right_raw = remoteController.get_mouseRight();
        const bool right_pressed = input_dispatcher.IsRightButtonHeld();
        bool feedback_ok = true;
        bool lift_feedback_ok = true;
        for (unsigned i = 0; i < 3; ++i)
        {
            angle_deg[i] = i < 2 ? gantry_3508_motor.getAddAngleDeg(i + 1) :
                                     gantry_dm4310_motor.getAddAngleDeg(1);
            velocity[i] = i < 2 ? gantry_3508_motor.getVelocityRads(i + 1) :
                                 gantry_dm4310_motor.getVelocityRads(1);
            // Parse() 在驱动内更新时间戳，isConnected() 判断反馈超时。
            const bool online = i < 2 ? gantry_3508_motor.isConnected(i + 1, i + 5) :
                                       gantry_dm4310_motor.isConnected(1, 9);
            gantry_watch.online[i] = online;
            gantry_watch.angle_deg[i] = angle_deg[i];
            gantry_watch.velocity_rad_s[i] = velocity[i];
            if (i < 2)
            {
                lift_current_a[i] = gantry_3508_motor.getCurrent(i + 1);
                gantry_watch.lift_current_a[i] = lift_current_a[i];
                lift_feedback_ok = lift_feedback_ok && online;
            }
            feedback_ok = feedback_ok && online && std::isfinite(velocity[i]);
        }
        const bool dm_ready = gantry_watch.online[2] && gantry_watch.dm_status == 1;
        feedback_ok = feedback_ok && dm_ready;
        gantry_watch.control_enabled = control_enabled;
        gantry_watch.feedback_ok = feedback_ok;
        gantry_watch.right_raw = right_raw;
        gantry_watch.right_pressed = right_pressed;
        taskEXIT_CRITICAL();

        // 更新动作状态机及调试状态。
        gantry_fsm.Update(
            control_enabled,
            feedback_ok,
            lift_feedback_ok, // 启动原点只依赖两台 3508，不等待 DM 使能
            right_raw,
            right_pressed,
            limit_pressed[0], // PE13：第一次抬升
            limit_pressed[1], // PE9：前伸
            limit_pressed[2], // PA2：第二次抬升
            limit_pressed[3], // PA3：后收
            angle_deg[0],     // 3508 ID5, deg
            angle_deg[1],     // 3508 ID6, deg
            angle_deg[2],     // DM ID9, deg
            lift_current_a[0], // 3508 ID5 实际转矩电流，A
            lift_current_a[1], // 3508 ID6 实际转矩电流，A
            velocity[0],      // 3508 ID5 输出轴速度，rad/s
            velocity[1],      // 3508 ID6 输出轴速度，rad/s
            now_ms);
        gantry_watch.state = gantry_fsm.GetState();
        gantry_watch.fault = gantry_fsm.GetFault();
        gantry_watch.progress = gantry_fsm.GetProgress();
        gantry_watch.scale = gantry_fsm.GetScale();

        // 计算保持速度、升降 PID 和 DM 阻尼力矩，发送 CAN。
        for (unsigned i = 0; i < 2; ++i)
        {
            float current = 0;
            gantry_watch.target_velocity_rad_s[i] = 0;
            if (!gantry_fsm.IsMoving(i) && !gantry_fsm.IsKeeping(i)) // 停机或到底后两台同时零电流
            {
                lift_move_pid[i].reset();
                lift_hold_pid[i].reset();
                hold_angle_pid[i].reset();
            }
            else if (gantry_fsm.IsMoving(i))  // 3508 升降电机在运动状态下使用升降 PID，保持 PID 清零
            {
                const float target = gantry_fsm.GetTargetSpeed(i);
                gantry_watch.target_velocity_rad_s[i] = target;
                lift_hold_pid[i].reset();
                hold_angle_pid[i].reset();
                current = lift_move_pid[i].UpDate(target, velocity[i]);
            }
            else if (gantry_fsm.IsKeeping(i)) // 3508 位置保持双环，升降 PID 清零
            {
                // 累计角度和标定保留 deg，保持外环的角度误差转换为 rad。
                const float hold_error_rad = static_cast<float>(
                    (gantry_fsm.GetHoldAngle(i) - angle_deg[i]) * (3.14159265358979323846 / 180.0));
                const float target = hold_angle_pid[i].UpDate(hold_error_rad, 0);
                gantry_watch.target_velocity_rad_s[i] = target;
                lift_move_pid[i].reset();
                current = lift_hold_pid[i].UpDate(target, velocity[i]);
            }
            if (gantry_fsm.GetState() == GantryState::Lower)
            {
                // C620 指令为原始值，±16384 对应 ±20 A，与减速比无关。
                const float current_limit = LOWER_CURRENT_LIMIT_A * (16384.0f / 20.0f);
                current = std::clamp(current, -current_limit, current_limit);
            }
            const int16_t command = static_cast<int16_t>(current);
            gantry_watch.lift_command[i] = command;
            gantry_3508_motor.setCAN(command, i + 1);
        }
        gantry_3508_motor.sendCAN();
        gantry_watch.target_velocity_rad_s[2] = 0;
        gantry_watch.dm_torque_command_nm = 0;
        if (gantry_fsm.IsActive())   
        {
            float target = 0;
            float kd = 0;
            if (gantry_fsm.IsMoving(2)) // DM 前伸、后收
            {
                hold_angle_pid[2].reset();
                target = gantry_fsm.GetTargetSpeed(2);
                kd = DM_MOVE_KD;
            }
            else if (gantry_fsm.IsKeeping(2)) // DM 位置保持
            {
                const float hold_error_rad = static_cast<float>(
                    (gantry_fsm.GetHoldAngle(2) - angle_deg[2]) * (3.14159265358979323846 / 180.0));
                target = hold_angle_pid[2].UpDate(hold_error_rad, 0);
                kd = DM_HOLD_KD;
            }
            // MIT 的 Kp=0，使用速度阻尼；累计角度保持在外部计算，避免多圈位置折返。
            // 先限制 Kd*(目标速度-实际速度) 的总力矩，再换回发送的速度。
            const float torque = std::clamp(kd * (target - velocity[2]),
                                           -DM_TORQUE_LIMIT_NM, DM_TORQUE_LIMIT_NM);
            gantry_watch.target_velocity_rad_s[2] = target;
            gantry_watch.dm_torque_command_nm = torque;
            gantry_dm4310_motor.ctrl_Mit(1, 0, velocity[2] + torque / kd, 0, kd, 0);
        }
        else  // 非活动状态下清零输出并复位 PID
        {
            hold_angle_pid[2].reset();
            gantry_dm4310_motor.ctrl_Mit(1, 0, 0, 0, 0, 0);
        }

        // 处理 DM 使能重试；停机仅发送上面的零力矩指令，不主动失能。
        const bool enable = control_enabled && gantry_fsm.GetState() != GantryState::Fault;
        // 等待初始反馈时允许请求使能；运行中掉线由 FSM 锁存故障，禁止自动续跑。
        if (enable)
        {
            if (dm_recovery_fsm.Should_Enable(dm_ready, now_ms))
                gantry_dm4310_motor.On(1, BSP::Motor::DM::MIT);
        }
        else
        {
            dm_recovery_fsm.Force_Enable_On_Next_Check();
        }
        osDelay(2);
    }
}
