#include "up_stair.hpp"
#include "../fsm/up_stair_fsm.hpp"
#include "../fsm/motor_recovery_fsm.hpp"
#include "can_send_task.hpp"


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

 float front_left_target_velocity =0;
 float front_left_target_torque  =0;

Class_Up_Stair_FSM up_stair_fsm;
MotorRecoveryFSM front_left_recovery_fsm;

volatile bool dm_motor_control_ready = false;

extern "C" void up_stair_task(void *argument)
{
    (void)argument;

    while (!dm_motor_control_ready)
    {
        osDelay(1U);
    }

    // ID1 的使能由 front_left_recovery_fsm 统一管理：首次离线立即发一次，
    // 之后只在未恢复反馈时按周期重试，避免启动阶段与恢复逻辑重复发使能帧。
    front_4340.On(2, BSP::Motor::DM::Model::MIT);

    up_stair_fsm.Init(stair_action_sequence);
    front_left_recovery_fsm.Init(HAL_GetTick());

    for (;;)
    {
        const uint32_t now_tick = HAL_GetTick();
        const float front_left_angle = front_4340.getAngleRad(1);
        const float front_right_angle = front_4340.getAngleRad(2);
        const float front_left_velocity = front_4340.getVelocityRads(1);
        const float front_right_velocity = front_4340.getVelocityRads(2);
        const bool left_online = front_4340.isConnected(1, 1);
        const bool right_online = front_4340.isConnected(2, 2);
        const bool left_feedback_valid = left_online && up_stair_fsm.Is_Angle_Valid(1U, front_left_angle);
        const bool right_feedback_valid = right_online && up_stair_fsm.Is_Angle_Valid(2U, front_right_angle);
        const bool keyboard_online = gimbal_keyboard_received && (now_tick - gimbal_keyboard_last_tick < 100U);
        const bool keyboard_mode = (gimbalChassis_communicate.s1 == 3) && (gimbalChassis_communicate.s2 == 3) && keyboard_online;
        const bool left_enable_requested =
            front_left_recovery_fsm.Should_Enable(left_online, now_tick);

        up_stair_fsm.Update(
            front_left_angle,
            front_right_angle,
            left_feedback_valid,
            right_feedback_valid,
            keyboard_mode,
            stair_action_sequence);

        // The motor can report feedback while still disabled after its power
        // rail recovers. Dispatch the one-shot enable before selecting either
        // the active-control or safe-zero-torque command path.
        if (left_enable_requested)
        {
            front_4340.On(1, BSP::Motor::DM::Model::MIT);
        }

        if (left_feedback_valid && up_stair_fsm.Is_Enabled())
        {
              front_left_target_velocity = front_4340_left_pid[0].UpDate(up_stair_fsm.Get_Target_Angle(1), up_stair_fsm.Get_Position_Feedback(1));
              front_left_target_torque = front_4340_left_pid[1].UpDate(front_left_target_velocity, front_left_velocity);

            front_4340.ctrl_Mit(
                1,
                up_stair_fsm.Get_Current_Angle(1),
                0.0f,
                0.0f,
                0.0f,
                front_left_target_torque);
        }
        else
        {
            front_4340_left_pid[0].reset();
            front_4340_left_pid[1].reset();

            // Keep a powered or newly restored motor responsive even while
            // feedback is offline. With KP, KD and torque all zero this frame
            // cannot command mechanical motion.
            front_4340.ctrl_Mit(1, front_left_angle, 0.0f, 0.0f, 0.0f, 0.0f);
        }

        if (right_feedback_valid && up_stair_fsm.Is_Enabled())
        {
            const float front_right_target_velocity = front_4340_right_pid[0].UpDate(up_stair_fsm.Get_Target_Angle(2), up_stair_fsm.Get_Position_Feedback(2));
            const float front_right_target_torque = front_4340_right_pid[1].UpDate(front_right_target_velocity, front_right_velocity);

            front_4340.ctrl_Mit(
                2,
                up_stair_fsm.Get_Current_Angle(2),
                0.0f,
                0.0f,
                0.0f,
                front_right_target_torque);
        }
        else
        {
            front_4340_right_pid[0].reset();
            front_4340_right_pid[1].reset();
            if (right_online)
            {
                front_4340.ctrl_Mit(2, front_right_angle, 0.0f, 0.0f, 0.0f, 0.0f);
            }
        }

        osDelay(1U);
    }
}
