#include "up_stair.hpp"
#include "../fsm/stair_mode_policy.hpp"
#include "../fsm/up_stair_fsm.hpp"
#include "../fsm/up_stair_behind_motor_fsm.hpp"
#include "../fsm/motor_recovery_fsm.hpp"
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

// No integral term: angle loop -> rate loop -> torque loop. Tune after testing.
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
    // TODO(calibration): replace equal zero limits with measured safe limits.
    // Equal start/end intentionally disables rear torque until calibration.
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
        const uint32_t now_tick = HAL_GetTick();
        const float front_left_angle = front_4340.getAngleRad(1);
        const float front_right_angle = front_4340.getAngleRad(2);
        const float front_left_velocity = front_4340.getVelocityRads(1);
        const float front_right_velocity = front_4340.getVelocityRads(2);
        const float rear_left_angle = rear_6248.getAngleRad(1);
        const float rear_right_angle = rear_6248.getAngleRad(2);
        const bool front_left_online = front_4340.isConnected(1, 1);
        const bool front_right_online = front_4340.isConnected(2, 2);
        const bool rear_left_online = rear_6248.isConnected(1, 1);
        const bool rear_right_online = rear_6248.isConnected(2, 2);

        uint8_t switch_s1;
        uint8_t switch_s2;
        uint32_t switch_last_tick;
        bool switch_received;
        uint32_t keyboard_last_tick;
        bool keyboard_received;
        taskENTER_CRITICAL();
        switch_s1 = static_cast<uint8_t>(gimbalChassis_communicate.s1);
        switch_s2 = static_cast<uint8_t>(gimbalChassis_communicate.s2);
        switch_last_tick = gimbal_switch_last_tick;
        switch_received = gimbal_switch_received;
        keyboard_last_tick = gimbal_keyboard_last_tick;
        keyboard_received = gimbal_keyboard_received;
        taskEXIT_CRITICAL();

        const bool control_link_online =
            switch_received && (now_tick - switch_last_tick < 100U);
        const bool keyboard_online =
            keyboard_received && (now_tick - keyboard_last_tick < 100U);
        const StairModePolicy policy = EvaluateStairModePolicy(
            switch_s1, switch_s2, control_link_online, keyboard_online);

        // Absolute safety gate: double-down/offline means all four zero torque.
        // This branch intentionally precedes every recovery/enable request.
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

        const bool front_left_feedback_valid =
            front_left_online && up_stair_fsm.Is_Angle_Valid(1U, front_left_angle);
        const bool front_right_feedback_valid =
            front_right_online && up_stair_fsm.Is_Angle_Valid(2U, front_right_angle);
        up_stair_fsm.Update(
            front_left_angle, front_right_angle, front_left_feedback_valid,
            front_right_feedback_valid, policy.front_hold_enabled,
            policy.front_stair_command_enabled, stair_action_sequence);

        const bool imu_valid = bmi088.IsReady();
        up_stair_behind_motor_fsm.Update(
            policy.rear_attitude_enabled, imu_valid, rear_left_online,
            rear_right_online, bmi088.GetPitchAngleDeg(), bmi088.GetRollAngleDeg(),
            bmi088.GetGyroRateYDps(), bmi088.GetGyroRateXDps(), rear_left_angle,
            rear_right_angle, now_tick);

        if (front_recovery_fsm[0].Should_Enable(front_left_online, now_tick))
            front_4340.On(1, BSP::Motor::DM::Model::MIT);
        if (front_recovery_fsm[1].Should_Enable(front_right_online, now_tick))
            front_4340.On(2, BSP::Motor::DM::Model::MIT);
        if (rear_recovery_fsm[0].Should_Enable(rear_left_online, now_tick))
            rear_6248.On(1, BSP::Motor::DM::Model::MIT);
        if (rear_recovery_fsm[1].Should_Enable(rear_right_online, now_tick))
            rear_6248.On(2, BSP::Motor::DM::Model::MIT);

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
            rear_6248.ctrl_Mit(
                1, SafeMotorAngle(rear_left_angle), 0.0f, 0.0f, 0.0f,
                up_stair_behind_motor_fsm.Limit_Torque(1, left_mixed_torque));
            rear_6248.ctrl_Mit(
                2, SafeMotorAngle(rear_right_angle), 0.0f, 0.0f, 0.0f,
                up_stair_behind_motor_fsm.Limit_Torque(2, right_mixed_torque));
        }

        osDelay(1U);
    }
}
