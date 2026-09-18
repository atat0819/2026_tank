#include "up_stair.hpp"
#include "../fsm/stair_mode_policy.hpp"
#include "../fsm/up_stair_fsm.hpp"
#include "../fsm/up_stair_behind_motor_fsm.hpp"
#include "../fsm/motor_recovery_fsm.hpp"
#include "../fsm/rear_torque_safety.hpp"
#include "can_send_task.hpp"
#include "imu_task.hpp"
#include "FreeRTOS.h"

#include <math.h>

BSP::Motor::DM::J4310<2> front_4340(
    0x00, {0x01, 0x02}, {0x01, 0x02}, HAL::FDCAN::FdcanDeviceId::HAL_Fdcan1);
BSP::Motor::DM::J6248<2> rear_6248(
    0x00, {0x03, 0x04}, {0x03, 0x04}, HAL::FDCAN::FdcanDeviceId::HAL_Fdcan3);

ALG::PID::PID front_4340_left_pid[2] = {
    {15.0f, 0.0f, 0.0f, 45.0f, 0.0f, 0.0f},
    {3.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f},
};
ALG::PID::PID front_4340_right_pid[2] = {
    {8.0f, 0.0f, 0.0f, 45.0f, 0.0f, 0.0f},
    {0.8f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f},
};

// 后 6248 不使用积分项，采用“姿态角环 → 角速度环 → 力矩”的串级控制。
// 具体增益需要结合整车负载、连杆方向和实车响应继续标定。
ALG::PID::PID rear_6248_pitch_pid[2] = {
    {1.5f, 0.0f, 0.0f, 30.0f, 0.0f, 0.0f},
    {0.8f, 0.0f, 0.0f, 40.0f, 0.0f, 0.0f},
};
ALG::PID::PID rear_6248_roll_pid[2] = {
    {1.5f, 0.0f, 0.0f, 30.0f, 0.0f, 0.0f},
    {0.8f, 0.0f, 0.0f, 40.0f, 0.0f, 0.0f},
};

namespace
{
Class_Up_Stair_Behind_Motor_FSM::Config BuildRear6248Config()
{
    Class_Up_Stair_Behind_Motor_FSM::Config config;
    // TODO（机械标定）：将下面的零位、起止角替换为实测安全参数。
    // 起止角相等会使配置无效，标定完成前后 6248 因此保持零力矩。
    config.pitch_zero_deg = 0.0f;
    config.roll_zero_deg = 0.0f;
    config.angle_start_rad[0] = 0.0f;
    config.angle_end_rad[0] = 0.0f;
    config.angle_start_rad[1] = 0.0f;
    config.angle_end_rad[1] = 0.0f;
    config.motor_direction[0] = 1;
    config.motor_direction[1] = 1;
    return config;
}

float SafeMotorAngle(float angle)
{
    return std::isfinite(angle) ? angle : 0.0f;
}

void ResetAllStairPid()
{
    // 进入双下、失联或其他禁用状态时清除四组 PID 的历史误差和积分量。
    front_4340_left_pid[0].reset();
    front_4340_left_pid[1].reset();
    front_4340_right_pid[0].reset();
    front_4340_right_pid[1].reset();
    rear_6248_pitch_pid[0].reset();
    rear_6248_pitch_pid[1].reset();
    rear_6248_roll_pid[0].reset();
    rear_6248_roll_pid[1].reset();
}

void SendAllStairMotorsZero(float front_left_angle, float front_right_angle,
                            float rear_left_angle, float rear_right_angle)
{
    // 双下或链路故障时，四个机构电机统一发送零力矩，但仍带当前安全角度。
    front_4340.ctrl_Mit(1, SafeMotorAngle(front_left_angle), 0.0f, 0.0f,
                        0.0f, 0.0f);
    front_4340.ctrl_Mit(2, SafeMotorAngle(front_right_angle), 0.0f, 0.0f,
                        0.0f, 0.0f);
    rear_6248.ctrl_Mit(1, SafeMotorAngle(rear_left_angle), 0.0f, 0.0f,
                       0.0f, 0.0f);
    rear_6248.ctrl_Mit(2, SafeMotorAngle(rear_right_angle), 0.0f, 0.0f,
                       0.0f, 0.0f);
}
}  // namespace

float front_left_target_velocity = 0.0f;
float front_left_target_torque = 0.0f;
Class_Up_Stair_FSM up_stair_fsm;
Class_Up_Stair_Behind_Motor_FSM up_stair_behind_motor_fsm(BuildRear6248Config());
MotorRecoveryFSM front_recovery_fsm[2];
MotorRecoveryFSM rear_recovery_fsm[2];
volatile bool dm_motor_control_ready = false;

extern "C" void up_stair_task(void *argument)
{
    (void)argument;
    // 等待 CAN 接收回调和电机对象准备完成，避免上电瞬间访问未初始化反馈。
    while (!dm_motor_control_ready)
    {
        osDelay(1U);
    }

    up_stair_fsm.Init(stair_action_sequence);
    const uint32_t init_tick = HAL_GetTick();
    front_recovery_fsm[0].Init(init_tick);
    front_recovery_fsm[1].Init(init_tick);
    rear_recovery_fsm[0].Init(init_tick);
    rear_recovery_fsm[1].Init(init_tick);
    ResetAllStairPid();

    for (;;)
    {
        // 读取四个机构电机的角度、速度和在线状态，后续状态机只使用这些原始数据。
        const float front_left_angle = front_4340.getAngleRad(1);
        const float front_right_angle = front_4340.getAngleRad(2);
        const float front_left_velocity = front_4340.getVelocityRads(1);
        const float front_right_velocity = front_4340.getVelocityRads(2);
        const float rear_left_angle = rear_6248.getAngleRad(1);
        const float rear_right_angle = rear_6248.getAngleRad(2);
        const bool front_left_online = front_4340.isConnected(1, 1);
        const bool front_right_online = front_4340.isConnected(2, 2);
        const bool rear_left_online = rear_6248.isConnected(1, 3);
        const bool rear_right_online = rear_6248.isConnected(2, 4);

        uint8_t switch_s1;
        uint8_t switch_s2;
        uint32_t switch_last_tick;
        bool switch_received;
        uint32_t keyboard_last_tick;
        bool keyboard_received;
        uint32_t now_tick;
        // 在临界区内一次性快照档位、键盘心跳和当前时间，避免回调更新一半时被读取。
        taskENTER_CRITICAL();
        switch_s1 = static_cast<uint8_t>(gimbalChassis_communicate.s1);
        switch_s2 = static_cast<uint8_t>(gimbalChassis_communicate.s2);
        switch_last_tick = gimbal_switch_last_tick;
        switch_received = gimbal_switch_received;
        keyboard_last_tick = gimbal_keyboard_last_tick;
        keyboard_received = gimbal_keyboard_received;
        now_tick = HAL_GetTick();
        taskEXIT_CRITICAL();

        ImuControlSnapshot imu_snapshot;
        GetImuControlSnapshot(imu_snapshot);

        const bool control_link_online =
            switch_received && (now_tick - switch_last_tick < 100U);
        const bool keyboard_online =
            keyboard_received && (now_tick - keyboard_last_tick < 100U);
        const StairModePolicy policy = EvaluateStairModePolicy(
            switch_s1, switch_s2, control_link_online, keyboard_online);

        // 绝对安全分支：双下、档位非法或链路超时都让四个电机输出零力矩。
        // 该分支必须早于状态机、PID 和任何电机 On/恢复请求。
        if (policy.zero_all_torque)
        {
            up_stair_fsm.Update(front_left_angle, front_right_angle, false,
                                false, false, false, stair_action_sequence);
            up_stair_behind_motor_fsm.Update(
                false, false, false, false, 0.0f, 0.0f, 0.0f, 0.0f,
                rear_left_angle, rear_right_angle, now_tick);
            ResetAllStairPid();
            SendAllStairMotorsZero(front_left_angle, front_right_angle,
                                   rear_left_angle, rear_right_angle);
            osDelay(1U);
            continue;
        }

        // 前 4310 的编码器必须同时满足在线和机械角度有效，才允许位置闭环。
        const bool front_left_feedback_valid =
            front_left_online && up_stair_fsm.Is_Angle_Valid(1U, front_left_angle);
        const bool front_right_feedback_valid =
            front_right_online && up_stair_fsm.Is_Angle_Valid(2U, front_right_angle);
        up_stair_fsm.Update(
            front_left_angle, front_right_angle, front_left_feedback_valid,
            front_right_feedback_valid, policy.front_hold_enabled,
            policy.front_stair_command_enabled, stair_action_sequence);

        // 后 6248 只接受完整且不超过 20 ms 的 IMU 快照。
        const bool imu_valid = imu_snapshot.valid &&
                               (now_tick - imu_snapshot.tick < 20U);
        up_stair_behind_motor_fsm.Update(
            policy.rear_attitude_enabled, imu_valid, rear_left_online,
            rear_right_online, imu_snapshot.pitch_deg, imu_snapshot.roll_deg,
            imu_snapshot.pitch_rate_dps, imu_snapshot.roll_rate_dps,
            rear_left_angle, rear_right_angle, now_tick);

        // 电机恢复状态机只负责重新发送 On；实际力矩仍由下面的状态机和 PID 决定。
        if (front_recovery_fsm[0].Should_Enable(front_left_online, now_tick))
            front_4340.On(1, BSP::Motor::DM::Model::MIT);
        if (front_recovery_fsm[1].Should_Enable(front_right_online, now_tick))
            front_4340.On(2, BSP::Motor::DM::Model::MIT);
        if (rear_recovery_fsm[0].Should_Enable(rear_left_online, now_tick))
            rear_6248.On(1, BSP::Motor::DM::Model::MIT);
        if (rear_recovery_fsm[1].Should_Enable(rear_right_online, now_tick))
            rear_6248.On(2, BSP::Motor::DM::Model::MIT);

        // 前左 4310：位置环生成目标速度，速度环生成最终力矩。
        if (front_left_feedback_valid && up_stair_fsm.Is_Enabled())
        {
            front_left_target_velocity = front_4340_left_pid[0].UpDate(
                up_stair_fsm.Get_Target_Angle(1),
                up_stair_fsm.Get_Position_Feedback(1));
            front_left_target_torque = front_4340_left_pid[1].UpDate(
                front_left_target_velocity, front_left_velocity);
            front_4340.ctrl_Mit(1, up_stair_fsm.Get_Current_Angle(1), 0.0f,
                                0.0f, 0.0f, front_left_target_torque);
        }
        else
        {
            front_4340_left_pid[0].reset();
            front_4340_left_pid[1].reset();
            front_4340.ctrl_Mit(1, SafeMotorAngle(front_left_angle), 0.0f,
                                0.0f, 0.0f, 0.0f);
        }

        // 前右 4310 与左侧使用相同的两级闭环，但使用独立 PID 参数。
        if (front_right_feedback_valid && up_stair_fsm.Is_Enabled())
        {
            const float target_velocity = front_4340_right_pid[0].UpDate(
                up_stair_fsm.Get_Target_Angle(2),
                up_stair_fsm.Get_Position_Feedback(2));
            const float target_torque = front_4340_right_pid[1].UpDate(
                target_velocity, front_right_velocity);
            front_4340.ctrl_Mit(2, up_stair_fsm.Get_Current_Angle(2), 0.0f,
                                0.0f, 0.0f, target_torque);
        }
        else
        {
            front_4340_right_pid[0].reset();
            front_4340_right_pid[1].reset();
            front_4340.ctrl_Mit(2, SafeMotorAngle(front_right_angle), 0.0f,
                                0.0f, 0.0f, 0.0f);
        }

        if (up_stair_behind_motor_fsm.Get_State() ==
            UP_STAIR_BEHIND_MOTOR_DISABLED)
        {
            // 后部状态机禁用时，清空姿态 PID 并明确发送零力矩。
            rear_6248_pitch_pid[0].reset();
            rear_6248_pitch_pid[1].reset();
            rear_6248_roll_pid[0].reset();
            rear_6248_roll_pid[1].reset();
            rear_6248.ctrl_Mit(1, SafeMotorAngle(rear_left_angle), 0.0f,
                               0.0f, 0.0f, 0.0f);
            rear_6248.ctrl_Mit(2, SafeMotorAngle(rear_right_angle), 0.0f,
                               0.0f, 0.0f, 0.0f);
        }
        else
        {
            // 后部姿态控制：pitch、roll 各自采用角度环和角速度环串级 PID。
            const float pitch_target_rate = rear_6248_pitch_pid[0].UpDate(
                up_stair_behind_motor_fsm.Get_Target_Pitch_Deg(),
                up_stair_behind_motor_fsm.Get_Feedback_Pitch_Deg());
            const float pitch_torque = rear_6248_pitch_pid[1].UpDate(
                pitch_target_rate, up_stair_behind_motor_fsm.Get_Pitch_Rate_Dps());
            const float roll_target_rate = rear_6248_roll_pid[0].UpDate(
                up_stair_behind_motor_fsm.Get_Target_Roll_Deg(),
                up_stair_behind_motor_fsm.Get_Feedback_Roll_Deg());
            const float roll_torque = rear_6248_roll_pid[1].UpDate(
                roll_target_rate, up_stair_behind_motor_fsm.Get_Roll_Rate_Dps());
            const float left_mixed_torque =
                static_cast<float>(up_stair_behind_motor_fsm.Get_Motor_Direction(1)) *
                (pitch_torque + roll_torque);
            const float right_mixed_torque =
                static_cast<float>(up_stair_behind_motor_fsm.Get_Motor_Direction(2)) *
                (pitch_torque - roll_torque);
            // pitch 对左右同向，roll 对左右反向；先做 J6248 应用层限幅，
            // 再由后部状态机执行反馈、机械角度和恢复比例保护。
            rear_6248.ctrl_Mit(
                1, SafeMotorAngle(rear_left_angle), 0.0f, 0.0f, 0.0f,
                up_stair_behind_motor_fsm.Limit_Torque(
                    1, StairTorqueSafety::ClampJ6248Torque(left_mixed_torque)));
            rear_6248.ctrl_Mit(
                2, SafeMotorAngle(rear_right_angle), 0.0f, 0.0f, 0.0f,
                up_stair_behind_motor_fsm.Limit_Torque(
                    2, StairTorqueSafety::ClampJ6248Torque(right_mixed_torque)));
        }

        osDelay(1U);
    }
}
