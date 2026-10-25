#include "fsm/stair_mode_policy.hpp"
#include "fsm/chassis_keyboard_fsm.hpp"
#include "fsm/up_stair_fsm.hpp"
#include "fsm/up_stair_behind_motor_fsm.hpp"
#include <cassert>
#include <cmath>

int main()
{
    StairCommand command;
    ChassisKeyboardFSM keyboard;
    Class_Up_Stair_FSM front;
    Class_Up_Stair_Behind_Motor_FSM rear(
        Class_Up_Stair_Behind_Motor_FSM::Config::WithAngleConstants());
    keyboard.Init();
    front.Init();
    uint32_t now = 0U;
    float left = 13.32f, right = -19.04f;
    bool control_enabled = true, imu_valid = true;
    auto step = [&](uint16_t keys) {
        keyboard.Update(keys, true, true, now);
        const auto &key = keyboard.GetCommand();
        UpdateStairCommand(command, control_enabled, key.imu_mode_request,
                           key.normal_mode_request, key.stair_toggle);
        front.Update(left*0.01745329252f, right*0.01745329252f, true, true,
                     control_enabled, true, command.front_target_requested, now);
        rear.Update(control_enabled && command.mode == StairMode::IMU,
                    imu_valid, true, true, 4.0f, 1.0f, 1.0f, 1.0f, now,
                    command.rear_retract_requested);
    };
    auto press = [&](uint16_t keys) {
        now += 1U; step(keys);
        now += 20U; step(keys);
        now += 1U; step(0U);
        now += 20U; step(0U);
    };
    step(0U); now=20U; step(0U);
    assert(command.mode == StairMode::NORMAL);
    assert(front.Get_State() == UP_STAIR_HOME_HOLD);
    assert(rear.Limit_Torque(1U, 10.0f) == 0.0f);
    press(ChassisKeyboardFSM::KEY_B);
    assert(front.Get_State() == UP_STAIR_HOME_HOLD);
    press(ChassisKeyboardFSM::KEY_V);
    assert(rear.Uses_Attitude_Control());
    press(ChassisKeyboardFSM::KEY_B);
    assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    assert(rear.Uses_Attitude_Control());
    // IMU failure zeros only the rear pair; selected mode/front target persist.
    imu_valid=false; now+=1U; step(0U);
    assert(command.mode == StairMode::IMU);
    assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    assert(rear.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    assert(rear.Limit_Torque(1U, 10.0f) == 0.0f);
    imu_valid=true; now+=1U; step(0U);
    assert(rear.Uses_Attitude_Control());
    left=50; right=-60;
    press(ChassisKeyboardFSM::KEY_B);
    assert(front.Get_State() == UP_STAIR_RETURNING_HOME);
    assert(rear.Uses_Retract_Position_Control());
    press(ChassisKeyboardFSM::KEY_B);
    assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    assert(rear.Uses_Attitude_Control());
    press(ChassisKeyboardFSM::KEY_B);
    assert(rear.Uses_Retract_Position_Control());
    imu_valid=false;
    press(ChassisKeyboardFSM::KEY_C);
    assert(front.Get_State() == UP_STAIR_RETURNING_HOME);
    assert(!rear.Uses_Position_Control() && !rear.Uses_Attitude_Control());
    for (uint8_t id=1; id<=2; ++id)
        assert(rear.Limit_Torque(id, 10.0f) == 0.0f);
    left=13.32f; right=-19.04f; imu_valid=true;
    press(ChassisKeyboardFSM::KEY_V);
    assert(front.Get_State() == UP_STAIR_HOME_HOLD);
    press(ChassisKeyboardFSM::KEY_B);
    assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    control_enabled=false; now+=1U; step(0U);
    assert(command.mode == StairMode::NORMAL);
    assert(!front.Is_Enabled());
    assert(rear.Limit_Torque(1U, 10.0f) == 0.0f);
    control_enabled=true; now+=1U; step(0U);
    assert(front.Get_State() == UP_STAIR_HOME_HOLD);
    assert(!rear.Uses_Attitude_Control());
}
