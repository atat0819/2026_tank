#include "fsm/up_stair_behind_motor_fsm.hpp"
#include <cassert>
#include <cmath>

int main()
{
    using Rear = Class_Up_Stair_Behind_Motor_FSM;
    const auto config = Rear::Config::WithAngleConstants();
    Rear rear(config);
    // Normal IMU entry and B transitions use full output immediately.
    rear.Update(true, true, true, true, 4.0f, 0.0f, 0.8f, 0.8f, 100U);
    assert(rear.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    assert(rear.Get_Output_Scale() == 1.0f);
    rear.Update(true, true, true, true, 4.0f, 0.0f, 0.8f, 0.8f, 101U, true);
    assert(rear.Uses_Retract_Position_Control());
    assert(rear.Get_Output_Scale() == 1.0f);
    // The first retract cycle exposes the final targets, without a position ramp.
    for (uint8_t id = 1U; id <= 2U; ++id)
        assert(std::fabs(rear.Get_Position_Target_Angle(id) - config.retract_target_rad[id - 1U]) < 1e-5f);
    rear.Update(true, true, true, true, 4.0f, 0.0f, 0.8f, 0.8f, 201U, true);
    assert(rear.Get_Position_Target_Angle(1U) > 0.8f);
    // Another B reverses immediately, without waiting for the retract target.
    rear.Update(true, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 202U, false);
    assert(rear.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    assert(rear.Get_Output_Scale() == 1.0f);
    rear.Update(true, false, true, true, NAN, NAN, 1.0f, 1.0f, 203U, true);
    assert(rear.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    // C overrides retraction and IMU failure: no basic-angle torque in NORMAL.
    rear.Update(false, false, true, true, NAN, NAN, 1.0f, 1.0f, 204U, false);
    assert(rear.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    assert(rear.Limit_Torque(1U, 10.0f) == 0.0f);
    assert(rear.Limit_Torque(2U, -10.0f) == 0.0f);

    // A held retract request must not bypass soft start after feedback returns.
    rear.Update(true, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 205U, true);
    assert(rear.Uses_Retract_Position_Control());
    assert(rear.Get_Output_Scale() == 0.0f);
    assert(std::fabs(rear.Get_Position_Target_Angle(1U) - config.retract_target_rad[0]) < 1e-5f);
    assert(std::fabs(rear.Get_Position_Target_Angle(2U) - config.retract_target_rad[1]) < 1e-5f);
    rear.Update(true, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 1205U, true);
    assert(std::fabs(rear.Get_Output_Scale() - 0.5f) < 1e-5f);
    // Changing between retract and attitude must not bypass fault recovery.
    rear.Update(true, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 1206U, false);
    assert(rear.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(std::fabs(rear.Get_Output_Scale() - 0.5005f) < 1e-5f);
    rear.Update(true, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 1207U, true);
    assert(std::fabs(rear.Get_Output_Scale() - 0.501f) < 1e-5f);
    rear.Update(false, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 1208U);
    rear.Update(true, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 1209U, true);
    assert(rear.Get_Output_Scale() == 0.0f);
    rear.Update(true, true, true, false, 4.0f, 0.0f, 1.0f, 1.0f, 2206U, true);
    assert(rear.Limit_Torque(1U, 10.0f) == 0.0f);
    rear.Update(true, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 2207U, true);
    assert(rear.Uses_Retract_Position_Control());
    assert(rear.Get_Output_Scale() == 0.0f);
    rear.Update(true, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 6207U, true);
    assert(rear.Uses_Retract_Position_Control());
    assert(rear.Get_Output_Scale() == 1.0f);

    rear.Update(true, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 6208U, false);
    rear.Update(true, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 10208U, false);
    assert(rear.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    rear.Update(true, true, true, false, 4.0f, 0.0f, 1.0f, 1.0f, 10209U, false);
    // Feedback recovery and B2 in the same frame must also restart the ramp.
    rear.Update(true, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 10210U, true);
    assert(rear.Uses_Retract_Position_Control());
    assert(rear.Get_Output_Scale() == 0.0f);

    // Enter HOLD only after both encoders reach their final targets.
    rear.Update(true, true, true, true, 4.0f, 0.0f,
                config.retract_target_rad[0], 1.0f, 10211U, true);
    assert(rear.Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTING);
    rear.Update(true, true, true, true, 4.0f, 0.0f,
                config.retract_target_rad[0], config.retract_target_rad[1], 10212U, true);
    assert(rear.Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD);
    assert(rear.Uses_Position_Control());
    // A motor fault still wins when recovery coincides with cancelling retract.
    rear.Update(true, true, true, false, 4.0f, 0.0f, 1.0f, 1.0f, 10213U, true);
    rear.Update(true, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 10214U, false);
    assert(rear.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(rear.Get_Output_Scale() == 0.0f);

    // Normal C/V cycling does not turn feedback edges into a false recovery.
    Rear normal(config);
    normal.Update(false, false, false, false, 0.0f, 0.0f, 1.0f, 1.0f, 0U);
    normal.Update(true, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 0U);
    normal.Update(false, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 1U);
    normal.Update(true, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 2U);
    assert(normal.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    assert(normal.Get_Output_Scale() == 1.0f);
    // Staggered motor feedback recovery waits for both channels before ramping.
    normal.Update(true, true, false, false, 4.0f, 0.0f, 1.0f, 1.0f, 3U);
    normal.Update(true, true, true, false, 4.0f, 0.0f, 1.0f, 1.0f, 4U);
    normal.Update(true, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 5U);
    assert(normal.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(normal.Get_Output_Scale() == 0.0f);

    // Direct targets retain the existing continuous-angle mapping at +/-pi.
    auto wrapped_config = config;
    const float pi = 3.14159265358979323846f;
    wrapped_config.angle_start_rad[1] = 150.0f * pi / 180.0f;
    wrapped_config.angle_end_rad[1] = -150.0f * pi / 180.0f;
    wrapped_config.retract_target_rad[1] = -170.0f * pi / 180.0f;
    Rear wrapped(wrapped_config);
    wrapped.Update(true, true, true, true, 4.0f, 0.0f,
                   1.0f, 170.0f * pi / 180.0f, 0U, true);
    assert(wrapped.Uses_Position_Control());
    assert(std::fabs(wrapped.Get_Position_Target_Angle(2U) - 190.0f * pi / 180.0f) < 1e-5f);

    // Speed soft start changes only the transient, never the final angle targets.
    Rear speed(config);
    const uint32_t ramp_time = Rear::RETRACT_SPEED_RAMP_TIME_MS;
    speed.Update(true, true, true, true, 4, 0, 1, 1, 0U, true);
    assert(speed.Limit_Velocity(1, 2, 2.5f) == 0);
    assert(speed.Limit_Velocity(2, -2, 2.5f) == 0);
    speed.Update(true, true, true, true, 4, 0, 1, 1, ramp_time / 4U, true);
    assert(std::fabs(speed.Limit_Velocity(1, 2, 2.5f) - 0.5f) < 1e-5f);
    speed.Update(true, true, true, true, 4, 0, 1, 1, ramp_time / 2U, true, 0.7f, -0.7f);
    assert(std::fabs(speed.Limit_Velocity(1, 2, 2.5f) - 1) < 1e-5f);
    assert(std::fabs(speed.Limit_Velocity(2, -2, 2.5f) + 1) < 1e-5f);
    // Repeating a tick must not advance the ramp or restart it on reaching HOLD.
    speed.Update(true, true, true, true, 4, 0,
                 config.retract_target_rad[0], config.retract_target_rad[1], ramp_time / 2U, true);
    assert(speed.Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD);
    assert(std::fabs(speed.Limit_Velocity(1, 2, 2.5f) - 1) < 1e-5f);
    speed.Update(true, true, true, true, 4, 0, 1, 1, ramp_time, true);
    assert(std::fabs(speed.Limit_Velocity(1, 2, 2.5f) - 2) < 1e-5f);
    for (uint8_t id = 1U; id <= 2U; ++id)
        assert(std::fabs(speed.Get_Position_Target_Angle(id) - config.retract_target_rad[id - 1U]) < 1e-5f);
    speed.Update(true, true, true, true, 4, 0, 1, 1, ramp_time + 1U, false);
    assert(std::fabs(speed.Limit_Velocity(1, 2, 2.5f) - 2) < 1e-5f);
    speed.Update(true, true, true, true, 4, 0, 1, 1, ramp_time + 2U, true);
    assert(speed.Limit_Velocity(1, 2, 2.5f) == 0);
    speed.Update(true, true, true, true, 4, 0, 1, 1, 2U * ramp_time + 2U, true);
    assert(std::fabs(speed.Limit_Velocity(1, 2, 2.5f) - 2) < 1e-5f);
    // Millisecond tick rollover retains the same elapsed-time ramp.
    Rear rollover(config);
    const uint32_t start_tick = 0xFFFFFF00U;
    rollover.Update(true, true, true, true, 4, 0, 1, 1, start_tick, true);
    rollover.Update(true, true, true, true, 4, 0, 1, 1, start_tick + ramp_time / 2U, true);
    assert(std::fabs(rollover.Limit_Velocity(1, 2, 2.5f) - 1) < 1e-5f);
}
