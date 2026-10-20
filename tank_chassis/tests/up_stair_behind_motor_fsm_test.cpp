#include "../fsm/up_stair_behind_motor_fsm.hpp"
#include <assert.h>
#include <math.h>
#include <limits>

namespace
{
const float PI = 3.14159265358979323846f;

float deg(float value)
{
    return value * PI / 180.0f;
}

bool near(float actual, float expected)
{
    return fabsf(actual - expected) < 1.0e-5f;
}

Class_Up_Stair_Behind_Motor_FSM::Config valid_config()
{
    Class_Up_Stair_Behind_Motor_FSM::Config config;
    config.pitch_target_deg = 2.5f;
    config.angle_start_rad[0] = deg(10.0f);
    config.angle_end_rad[0] = deg(170.0f);
    config.angle_start_rad[1] = deg(-60.0f);
    config.angle_end_rad[1] = deg(60.0f);
    config.motor_direction[0] = 1;
    config.motor_direction[1] = -1;
    config.retract_target_rad[0] = 2.0f;
    config.retract_target_rad[1] = deg(-30.0f);
    config.retract_position_tolerance_rad = 0.02f;
    return config;
}

void update_valid(Class_Up_Stair_Behind_Motor_FSM &fsm, uint32_t tick)
{
    fsm.Update(true, true, true, true, 4.5f, 12.0f,
               deg(90.0f), deg(-10.0f), tick);
}

void enter_attitude_hold(Class_Up_Stair_Behind_Motor_FSM &fsm,
                         uint32_t tick)
{
    // Finish recovery at tick; later action samples must advance from this time.
    update_valid(fsm, tick - 4000U);
    fsm.Update(true, true, true, true, 4.5f, 12.0f,
               deg(90.0f), deg(-10.0f), tick);
    assert(fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
}

}

int main()
{
    // Two-second fault recovery: check intermediate effort, completion and both sides.
    Class_Up_Stair_Behind_Motor_FSM slow_start(valid_config());
    slow_start.Update(true, false, true, true, 0, 0, deg(90), deg(-10), 0U);
    const uint32_t ramp_ticks[] = {0U, 500U, 1000U, 1500U, 1999U, 2000U};
    for (uint32_t tick : ramp_ticks)
    {
        update_valid(slow_start, tick);
        const float expected = static_cast<float>(tick) / 2000.0f;
        assert(near(slow_start.Get_Output_Scale(), expected));
        assert(near(slow_start.Limit_Torque(1U, 2.0f), 2.0f * expected));
        assert(near(slow_start.Limit_Torque(2U, -2.0f), -2.0f * expected));
        assert(slow_start.Get_State() == (tick < 2000U
                   ? UP_STAIR_BEHIND_MOTOR_RECOVERING
                   : UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD));
    }

    // The default/unmeasured configuration must remain safely disabled.
    Class_Up_Stair_Behind_Motor_FSM default_fsm;
    default_fsm.Update(true, true, true, true, 0.0f, 0.0f,
                       deg(90.0f), deg(-10.0f), 0U);
    assert(default_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    assert(near(default_fsm.Get_Output_Scale(), 0.0f));
    assert(near(default_fsm.Limit_Torque(1U, 2.0f), 0.0f));

    const Class_Up_Stair_Behind_Motor_FSM::Config header_angles =
        Class_Up_Stair_Behind_Motor_FSM::Config::WithAngleConstants();
    assert(near(header_angles.angle_end_rad[0], deg(94.7f)));
    assert(near(header_angles.angle_start_rad[1], deg(4.66f)));
    assert(near(header_angles.pitch_target_deg, Class_Up_Stair_Behind_Motor_FSM::IMU_PITCH_TARGET_DEG));
    for (int i = 0; i < 2; ++i)
    {
        assert(near(header_angles.angle_start_rad[i], Class_Up_Stair_Behind_Motor_FSM::ANGLE_START_RAD[i]));
        assert(near(header_angles.angle_end_rad[i], Class_Up_Stair_Behind_Motor_FSM::ANGLE_END_RAD[i]));
        assert(header_angles.motor_direction[i] == Class_Up_Stair_Behind_Motor_FSM::MOTOR_DIRECTION[i]);
        assert(near(header_angles.motor_torque_gain[i], Class_Up_Stair_Behind_Motor_FSM::MOTOR_TORQUE_GAIN[i]));
        assert(near(header_angles.retract_target_rad[i], Class_Up_Stair_Behind_Motor_FSM::RETRACT_TARGET_RAD[i]));
        }
    assert(near(header_angles.retract_position_tolerance_rad,
                Class_Up_Stair_Behind_Motor_FSM::RETRACT_POSITION_TOLERANCE_RAD));
    assert(Class_Up_Stair_Behind_Motor_FSM(header_angles).Is_Config_Valid());

    // The latest measured flat-chassis stops replace the earlier calibration.
    Class_Up_Stair_Behind_Motor_FSM calibrated(header_angles);
    // Both flat-chassis readings must be valid before either leg can move.
    assert(calibrated.Is_Angle_Valid(1U, deg(94.7f)));
    assert(calibrated.Is_Angle_Valid(2U, deg(4.47f)));
    assert(calibrated.Is_Angle_Valid(1U, deg(28.6f)));
    assert(calibrated.Is_Angle_Valid(1U, deg(91.4f)));
    assert(calibrated.Is_Angle_Valid(2U, deg(5.6f)));
    assert(calibrated.Is_Angle_Valid(2U, deg(68.4f)));
    assert(!calibrated.Is_Angle_Valid(1U, deg(28.4f)));
    assert(calibrated.Is_Angle_Valid(1U, deg(91.6f)));
    assert(calibrated.Is_Angle_Valid(2U, deg(5.4f)));
    assert(!calibrated.Is_Angle_Valid(2U, deg(68.6f)));
    assert(!calibrated.Is_Angle_Valid(1U, deg(95.3f)));
    assert(!calibrated.Is_Angle_Valid(2U, deg(3.9f)));
    calibrated.Update(true, true, true, true, 0, 0, deg(60), deg(40), 0U);
    calibrated.Update(true, true, true, true, 0, 0, deg(60), deg(40), 4000U);
    // Inside the former return buffer, normal effort is no longer reduced.
    calibrated.Update(true, true, true, true, 0, 0, deg(93.7f), deg(5.47f), 4001U);
    assert(near(calibrated.Limit_Torque(1U, 3), 3));
    assert(near(calibrated.Limit_Torque(2U, -3), -3));
    // Near each endpoint, suppress outward effort while allowing return effort.
    calibrated.Update(true, true, true, true, 0, 0, deg(94.3f), deg(4.87f), 4002U);
    assert(near(calibrated.Limit_Torque(1U, 3), 0));
    assert(near(calibrated.Limit_Torque(1U, -3), -3));
    assert(near(calibrated.Limit_Torque(2U, -3), 0));
    assert(near(calibrated.Limit_Torque(2U, 3), 3));
    calibrated.Update(true, true, true, true, 0, 0,
                      deg(94.7f), deg(4.47f), 4002U);
    assert(calibrated.Is_Motor_Controllable(1U));
    assert(calibrated.Is_Motor_Controllable(2U));
    assert(near(calibrated.Limit_Torque(1U, 3), 0));
    assert(near(calibrated.Limit_Torque(1U, -3), -3));
    assert(near(calibrated.Limit_Torque(2U, -3), 0));
    assert(near(calibrated.Limit_Torque(2U, 3), 3));
    calibrated.Update(true, true, true, true, 0, 0,
                      header_angles.angle_end_rad[0], header_angles.angle_start_rad[1], 4002U);
    assert(calibrated.Is_Motor_Controllable(1U));
    assert(calibrated.Is_Motor_Controllable(2U));
    assert(near(calibrated.Limit_Torque(1U, 3), 0));
    assert(near(calibrated.Limit_Torque(1U, -3), -3));
    assert(near(calibrated.Limit_Torque(2U, -3), 0));
    assert(near(calibrated.Limit_Torque(2U, 3), 3));
    calibrated.Update(true, true, true, true, 0, 0, deg(93.26f), deg(5.6f), 4002U);
    assert(calibrated.Is_Motor_Controllable(1U));
    assert(calibrated.Is_Motor_Controllable(2U));
    assert(near(calibrated.Limit_Torque(1U, 3), 3));
    assert(near(calibrated.Limit_Torque(1U, -3), -3));
    assert(near(calibrated.Limit_Torque(2U, -3), -3));
    assert(near(calibrated.Limit_Torque(2U, 3), 3));
    // The fully extended ends also block outward effort and allow departure.
    calibrated.Update(true, true, true, true, 0, 0,
                      header_angles.angle_start_rad[0], header_angles.angle_end_rad[1], 4003U);
    assert(near(calibrated.Limit_Torque(1U, -3), 0));
    assert(near(calibrated.Limit_Torque(1U, 3), 3));
    assert(near(calibrated.Limit_Torque(2U, 3), 0));
    assert(near(calibrated.Limit_Torque(2U, -3), -3));
    calibrated.Update(true, true, true, true, 0, 0, deg(28.6f), deg(68.4f), 4003U);
    assert(calibrated.Is_Motor_Controllable(1U));
    assert(calibrated.Is_Motor_Controllable(2U));
    assert(near(calibrated.Limit_Torque(1U, -3), 0));
    assert(near(calibrated.Limit_Torque(1U, 3), 3));
    assert(near(calibrated.Limit_Torque(2U, 3), 0));
    assert(near(calibrated.Limit_Torque(2U, -3), -3));
    calibrated.Update(true, true, true, true, 0, 0,
                      header_angles.angle_start_rad[0], header_angles.angle_end_rad[1], 4004U, true);
    calibrated.Update(true, true, true, true, 0, 0,
                      header_angles.angle_start_rad[0], header_angles.angle_end_rad[1], 4104U, true);
    assert(calibrated.Get_Position_Target_Angle(1U) > header_angles.angle_start_rad[0]);
    assert(calibrated.Get_Position_Target_Angle(2U) < header_angles.angle_end_rad[1]);
    calibrated.Update(true, true, true, true, 0, 0,
                      header_angles.retract_target_rad[0], header_angles.retract_target_rad[1], 6004U, true);
    assert(calibrated.Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD);
    assert(near(calibrated.Get_Position_Target_Angle(1U), header_angles.retract_target_rad[0]));
    assert(near(calibrated.Get_Position_Target_Angle(2U), header_angles.retract_target_rad[1]));
    // One out-of-range rear angle disables the entire retract operation.
    calibrated.Update(true, true, true, true, 0, 0, deg(95.3f), deg(6), 6005U, true);
    assert(near(calibrated.Limit_Torque(1U, 3), 0));
    assert(near(calibrated.Limit_Torque(2U, 3), 0));

    // Feedback tolerance must not permit a position target past a hard stop.
    Class_Up_Stair_Behind_Motor_FSM::Config outside_target = header_angles;
    outside_target.retract_target_rad[0] = deg(94.9f);
    Class_Up_Stair_Behind_Motor_FSM outside_target_fsm(outside_target);
    assert(outside_target_fsm.Is_Angle_Valid(1U, deg(94.9f)));
    outside_target_fsm.Update(true, true, true, true, 0, 0, deg(60), deg(40), 0U, true);
    assert(outside_target_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);

    // 左右电机的最终力矩增益可独立校准，默认值 1.0 不改变原有输出。
    Class_Up_Stair_Behind_Motor_FSM::Config gain_config = valid_config();
    assert(near(gain_config.motor_torque_gain[0], 1.0f));
    assert(near(gain_config.motor_torque_gain[1], 1.0f));
    gain_config.motor_torque_gain[0] = 1.25f;
    gain_config.motor_torque_gain[1] = 0.80f;
    Class_Up_Stair_Behind_Motor_FSM gain_fsm(gain_config);
    update_valid(gain_fsm, 10000U);
    update_valid(gain_fsm, 14000U);
    assert(near(gain_fsm.Limit_Torque(1U, 2.0f), 2.50f));
    assert(near(gain_fsm.Limit_Torque(2U, 2.0f), 1.60f));

    Class_Up_Stair_Behind_Motor_FSM fsm(valid_config());
    assert(!fsm.Is_Motor_Controllable(1U));
    assert(!fsm.Is_Motor_Controllable(2U));

    // Valid feedback after an IMU fault ramps for 2000 ms.
    fsm.Update(true, false, true, true, 0, 0, deg(90), deg(-10), 999U);
    update_valid(fsm, 1000U);
    assert(fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(fsm.Get_Output_Scale(), 0.0f));
    assert(near(fsm.Limit_Torque(1U, 2.0f), 0.0f));
    fsm.Update(true, true, true, true, 4.5f, 12.0f,
               deg(90.0f), deg(-10.0f), 2000U);
    assert(near(fsm.Get_Output_Scale(), 0.5f));
    assert(fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(fsm.Limit_Torque(1U, 2.0f), 1.0f));
    fsm.Update(true, true, true, true, 4.5f, 12.0f,
               deg(90.0f), deg(-10.0f), 5000U);
    assert(fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    assert(near(fsm.Get_Output_Scale(), 1.0f));
    assert(near(fsm.Limit_Torque(1U, 2.0f), 2.0f));

    // Target and feedback use raw HI14 coordinates; the previous error is preserved.
    assert(near(fsm.Get_Target_Pitch_Deg(), 2.5f));
    assert(near(fsm.Get_Feedback_Pitch_Deg(), 4.5f));
    assert(near(fsm.Get_Pitch_Rate_Dps(), 12.0f));

    // Invalid IMU data disables both rear motors; a later valid
    // sample restarts attitude recovery from zero at its new tick.
    Class_Up_Stair_Behind_Motor_FSM imu_fsm(valid_config());
    update_valid(imu_fsm, 3000U);
    imu_fsm.Update(true, false, true, true, 4.5f, 12.0f,
                   deg(90.0f), deg(-10.0f), 3010U);
    assert(imu_fsm.Get_State() ==
           UP_STAIR_BEHIND_MOTOR_DISABLED);
    assert(!imu_fsm.Uses_Position_Control());

    // Nonfinite attitude/rate samples also disable both rear motors and
    // clear stored attitude feedback so stale IMU data cannot leak through.
    const float nan_value = std::numeric_limits<float>::quiet_NaN();
    const float inf_value = std::numeric_limits<float>::infinity();
    Class_Up_Stair_Behind_Motor_FSM numerical_fsm(valid_config());
    update_valid(numerical_fsm, 5000U);
    numerical_fsm.Update(true, true, true, true, nan_value, 12.0f, deg(90.0f), deg(-10.0f), 5300U);
    assert(numerical_fsm.Get_State() ==
           UP_STAIR_BEHIND_MOTOR_DISABLED);
    assert(near(numerical_fsm.Get_Feedback_Pitch_Deg(), 0.0f));
    assert(near(numerical_fsm.Get_Pitch_Rate_Dps(), 0.0f));
    assert(near(numerical_fsm.Limit_Torque(1U, 2.0f), 0.0f));
    update_valid(numerical_fsm, 6000U);
    assert(numerical_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(numerical_fsm.Get_Output_Scale(), 0.0f));
    numerical_fsm.Update(true, true, true, true, 4.5f, inf_value,
                         deg(90.0f), deg(-10.0f), 6300U);
    assert(numerical_fsm.Get_State() ==
           UP_STAIR_BEHIND_MOTOR_DISABLED);
    assert(near(numerical_fsm.Get_Feedback_Pitch_Deg(), 0.0f));
    assert(near(numerical_fsm.Get_Pitch_Rate_Dps(), 0.0f));
    assert(near(numerical_fsm.Limit_Torque(1U, 2.0f), 0.0f));
    assert(near(imu_fsm.Limit_Torque(1U, 2.0f), 0.0f));
    update_valid(imu_fsm, 4000U);
    assert(imu_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(imu_fsm.Get_Output_Scale(), 0.0f));

    // One feedback channel failing only removes that motor's permission.
    fsm.Update(true, true, false, true, 4.5f, 12.0f,
               deg(90.0f), deg(-10.0f), 5001U);
    assert(!fsm.Is_Motor_Controllable(1U));
    assert(fsm.Is_Motor_Controllable(2U));
    assert(near(fsm.Limit_Torque(1U, 1.0f), 0.0f));
    assert(!near(fsm.Limit_Torque(2U, 1.0f), 0.0f));

    // Disable resets recovery and output permission.
    fsm.Update(false, true, true, true, 4.5f, 12.0f,
               deg(90.0f), deg(-10.0f), 5100U);
    assert(fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    assert(near(fsm.Get_Output_Scale(), 0.0f));
    assert(near(fsm.Limit_Torque(2U, 1.0f), 0.0f));
    update_valid(fsm, 6000U);
    assert(fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(fsm.Get_Output_Scale(), 0.0f));

    // Normal interval: lower boundary blocks negative torque and upper blocks
    // positive torque, while escape torque remains available.
    fsm.Update(true, true, true, true, 4.5f, 12.0f,
               deg(10.0f), deg(-10.0f), 10000U);
    assert(near(fsm.Get_Output_Scale(), 1.0f));
    assert(near(fsm.Limit_Torque(1U, -3.0f), 0.0f));
    assert(near(fsm.Limit_Torque(1U, 3.0f), 3.0f));
    fsm.Update(true, true, true, true, 4.5f, 12.0f,
               deg(170.0f), deg(-10.0f), 10001U);
    assert(near(fsm.Limit_Torque(1U, 3.0f), 0.0f));
    assert(near(fsm.Limit_Torque(1U, -3.0f), -3.0f));

    // Signed interval protects both physical ends of the right motor range.
    assert(fsm.Is_Angle_Valid(2U, deg(-10.0f)));
    assert(fsm.Is_Angle_Valid(2U, deg(10.0f)));
    assert(!fsm.Is_Angle_Valid(2U, deg(120.0f)));
    fsm.Update(true, true, true, true, 4.5f, 12.0f,
               deg(90.0f), deg(-60.0f), 10002U);
    assert(near(fsm.Limit_Torque(2U, -4.0f), 0.0f));
    assert(near(fsm.Limit_Torque(2U, 4.0f), 4.0f));
    fsm.Update(true, true, true, true, 4.5f, 12.0f,
               deg(90.0f), deg(60.0f), 10003U);
    assert(near(fsm.Limit_Torque(2U, 4.0f), 0.0f));
    assert(near(fsm.Limit_Torque(2U, -4.0f), -4.0f));

    // 调试用全角度配置：两侧在 ±π 接缝及原限位缓冲区内都允许双向力矩。
    Class_Up_Stair_Behind_Motor_FSM::Config full_angle_config = valid_config();
    full_angle_config.angle_start_rad[0] = -PI;
    full_angle_config.angle_end_rad[0] = PI;
    full_angle_config.angle_start_rad[1] = -PI;
    full_angle_config.angle_end_rad[1] = PI;
    Class_Up_Stair_Behind_Motor_FSM full_angle_fsm(full_angle_config);
    const float debug_angles[] = {-PI, deg(-178.0f), 0.0f,
                                  deg(178.0f), PI};
    full_angle_fsm.Update(true, true, true, true, 4.5f,
                          12.0f, 0.0f, 0.0f, 5000U);
    for (unsigned i = 0U; i < sizeof(debug_angles) / sizeof(debug_angles[0]); ++i)
    {
        full_angle_fsm.Update(true, true, true, true, 4.5f,
                              12.0f, debug_angles[i], debug_angles[i],
                              9000U + i);
        for (uint8_t id = 1U; id <= 2U; ++id)
        {
            assert(full_angle_fsm.Is_Angle_Valid(id, debug_angles[i]));
            assert(near(full_angle_fsm.Limit_Torque(id, 3.0f), 3.0f));
            assert(near(full_angle_fsm.Limit_Torque(id, -3.0f), -3.0f));
        }
    }

    // Both interval types reject angles outside the encoder's signed domain.
    const float below_raw_domain = -PI - deg(1.0f);
    const float above_raw_domain = PI + deg(1.0f);
    assert(!fsm.Is_Angle_Valid(1U, below_raw_domain));
    assert(!fsm.Is_Angle_Valid(1U, above_raw_domain));
    assert(!fsm.Is_Angle_Valid(2U, below_raw_domain));
    assert(!fsm.Is_Angle_Valid(2U, above_raw_domain));
    fsm.Update(true, true, true, true, 4.5f, 12.0f,
               below_raw_domain, deg(-10.0f), 10004U);
    assert(!fsm.Is_Motor_Controllable(1U));
    assert(near(fsm.Limit_Torque(1U, 2.0f), 0.0f));
    fsm.Update(true, true, true, true, 4.5f, 12.0f,
               deg(90.0f), above_raw_domain, 10005U);
    assert(!fsm.Is_Motor_Controllable(2U));
    assert(near(fsm.Limit_Torque(2U, 2.0f), 0.0f));

    // Invalid torque requests and public IDs are always rejected.
    assert(near(fsm.Limit_Torque(1U, nan_value), 0.0f));
    assert(near(fsm.Limit_Torque(1U, inf_value), 0.0f));
    assert(near(fsm.Limit_Torque(0U, 1.0f), 0.0f));
    assert(near(fsm.Limit_Torque(3U, 1.0f), 0.0f));

    // Losing one feedback suppresses only that motor. Once both feedbacks
    // return, the shared FSM restarts its 2000 ms recovery ramp.
    Class_Up_Stair_Behind_Motor_FSM feedback_fsm(valid_config());
    update_valid(feedback_fsm, 7000U);
    feedback_fsm.Update(true, true, true, true, 4.5f, 12.0f, deg(90.0f), deg(-10.0f), 11000U);
    assert(feedback_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    feedback_fsm.Update(true, true, false, true, 4.5f, 12.0f, deg(90.0f), deg(-10.0f), 11001U);
    assert(feedback_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    assert(!feedback_fsm.Is_Motor_Controllable(1U));
    assert(feedback_fsm.Is_Motor_Controllable(2U));
    assert(near(feedback_fsm.Limit_Torque(2U, 2.0f), 2.0f));
    feedback_fsm.Update(true, true, true, true, 4.5f, 12.0f, deg(90.0f), deg(-10.0f), 11002U);
    assert(feedback_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(feedback_fsm.Get_Output_Scale(), 0.0f));
    assert(near(feedback_fsm.Limit_Torque(1U, 2.0f), 0.0f));
    feedback_fsm.Update(true, true, true, true, 4.5f, 12.0f, deg(90.0f), deg(-10.0f), 12002U);
    assert(near(feedback_fsm.Get_Output_Scale(), 0.5f));
    feedback_fsm.Update(true, true, true, true, 4.5f, 12.0f, deg(90.0f), deg(-10.0f), 15002U);
    assert(feedback_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    assert(near(feedback_fsm.Get_Output_Scale(), 1.0f));

    // Staggered return after both feedbacks are lost restarts recovery for
    // each valid transition; no returning motor gets immediate torque.
    Class_Up_Stair_Behind_Motor_FSM staggered_fsm(valid_config());
    update_valid(staggered_fsm, 8000U);
    staggered_fsm.Update(true, true, true, true, 4.5f, 12.0f, deg(90.0f), deg(-10.0f), 12000U);
    staggered_fsm.Update(true, true, false, false, 4.5f, 12.0f, deg(90.0f), deg(-10.0f), 12001U);
    assert(!staggered_fsm.Is_Motor_Controllable(1U));
    assert(!staggered_fsm.Is_Motor_Controllable(2U));
    staggered_fsm.Update(true, true, true, false, 4.5f, 12.0f, deg(90.0f), deg(-10.0f), 12002U);
    assert(staggered_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(staggered_fsm.Get_Output_Scale(), 0.0f));
    assert(staggered_fsm.Is_Motor_Controllable(1U));
    assert(!staggered_fsm.Is_Motor_Controllable(2U));
    assert(near(staggered_fsm.Limit_Torque(1U, 2.0f), 0.0f));
    assert(near(staggered_fsm.Limit_Torque(2U, 2.0f), 0.0f));
    staggered_fsm.Update(true, true, true, true, 4.5f, 12.0f, deg(90.0f), deg(-10.0f), 12003U);
    assert(staggered_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(staggered_fsm.Get_Output_Scale(), 0.0f));
    assert(near(staggered_fsm.Limit_Torque(1U, 2.0f), 0.0f));
    assert(near(staggered_fsm.Limit_Torque(2U, 2.0f), 0.0f));
    staggered_fsm.Update(true, true, true, true, 4.5f, 12.0f, deg(90.0f), deg(-10.0f), 13003U);
    assert(near(staggered_fsm.Get_Output_Scale(), 0.5f));
    staggered_fsm.Update(true, true, true, true, 4.5f, 12.0f, deg(90.0f), deg(-10.0f), 16003U);
    assert(staggered_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    assert(near(staggered_fsm.Limit_Torque(1U, 2.0f), 2.0f));

    // Finite target and feedback can still overflow their difference.
    Class_Up_Stair_Behind_Motor_FSM::Config overflow_config = valid_config();
    overflow_config.pitch_target_deg = -std::numeric_limits<float>::max();
    Class_Up_Stair_Behind_Motor_FSM overflow_fsm(overflow_config);
    overflow_fsm.Update(true, true, true, true,
                        std::numeric_limits<float>::max(), 12.0f, deg(90.0f), deg(-10.0f), 9000U);
    assert(overflow_fsm.Get_State() ==
           UP_STAIR_BEHIND_MOTOR_DISABLED);
    assert(near(overflow_fsm.Get_Feedback_Pitch_Deg(), 0.0f));
    assert(near(overflow_fsm.Limit_Torque(1U, 2.0f), 0.0f));

    // Unsigned tick subtraction keeps recovery deterministic across wrap.
    Class_Up_Stair_Behind_Motor_FSM wrap_fsm(valid_config());
    wrap_fsm.Update(true, false, true, true, 0, 0, deg(90), deg(-10), 0xFFFFFEFFU);
    update_valid(wrap_fsm, 0xFFFFFF00U);
    wrap_fsm.Update(true, true, true, true, 4.5f, 12.0f,
                    deg(90.0f), deg(-10.0f), 0x00000000U);
    assert(wrap_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(wrap_fsm.Get_Output_Scale(), 256.0f / 2000.0f));
    wrap_fsm.Update(true, true, true, true, 4.5f, 12.0f,
                    deg(90.0f), deg(-10.0f), 1744U);
    assert(wrap_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    assert(near(wrap_fsm.Get_Output_Scale(), 1.0f));

    assert(fsm.Get_Motor_Direction(1U) == 1);
    assert(fsm.Get_Motor_Direction(2U) == -1);
    assert(fsm.Get_Motor_Direction(0U) == 0);

    // Equal limits and any direction other than exactly +/-1 are invalid.
    Class_Up_Stair_Behind_Motor_FSM::Config bad = valid_config();
    bad.angle_end_rad[0] = bad.angle_start_rad[0];
    Class_Up_Stair_Behind_Motor_FSM bad_range(bad);
    update_valid(bad_range, 0U);
    assert(bad_range.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    bad = valid_config();
    bad.motor_direction[1] = 0;
    Class_Up_Stair_Behind_Motor_FSM bad_direction(bad);
    update_valid(bad_direction, 0U);
    assert(bad_direction.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    // 最终增益必须是正的有限数，保证不会反向或产生无效输出。
    bad = valid_config();
    bad.motor_torque_gain[0] = 0.0f;
    Class_Up_Stair_Behind_Motor_FSM bad_zero_gain(bad);
    update_valid(bad_zero_gain, 0U);
    assert(bad_zero_gain.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    bad = valid_config();
    bad.motor_torque_gain[1] = nan_value;
    Class_Up_Stair_Behind_Motor_FSM bad_nan_gain(bad);
    update_valid(bad_nan_gain, 0U);
    assert(bad_nan_gain.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    bad = valid_config();
    bad.pitch_target_deg = nan_value;
    Class_Up_Stair_Behind_Motor_FSM bad_offset(bad);
    update_valid(bad_offset, 0U);
    assert(bad_offset.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    bad = valid_config();
    bad.angle_start_rad[0] = -PI - 0.01f;
    Class_Up_Stair_Behind_Motor_FSM bad_negative_limit(bad);
    update_valid(bad_negative_limit, 0U);
    assert(bad_negative_limit.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    bad = valid_config();
    bad.angle_end_rad[0] = PI + 0.01f;
    Class_Up_Stair_Behind_Motor_FSM bad_large_limit(bad);
    update_valid(bad_large_limit, 0U);
    assert(bad_large_limit.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    bad = valid_config();
    bad.angle_start_rad[1] = PI;
    bad.angle_end_rad[1] = -PI;
    Class_Up_Stair_Behind_Motor_FSM bad_alias_limit(bad);
    update_valid(bad_alias_limit, 0U);
    assert(bad_alias_limit.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);

    // A persistent retract request switches from attitude control to encoder-only
    // retract control and exposes continuous position targets/feedback.
    Class_Up_Stair_Behind_Motor_FSM retract_fsm(valid_config());
    enter_attitude_hold(retract_fsm, 10000U);
    retract_fsm.Update(true, true, true, true, 4.5f, 12.0f,
                       deg(90.0f), deg(-10.0f), 10300U, true);
    assert(retract_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTING);
    assert(!retract_fsm.Uses_Attitude_Control());
    assert(retract_fsm.Uses_Retract_Position_Control());
    assert(retract_fsm.Uses_Position_Control());
    assert(near(retract_fsm.Get_Position_Target_Angle(1U), 2.0f));
    assert(near(retract_fsm.Get_Position_Target_Angle(2U), deg(-30.0f)));
    assert(near(retract_fsm.Get_Retract_Target_Angle(1U), 2.0f));
    assert(near(retract_fsm.Get_Retract_Target_Angle(2U), deg(-30.0f)));
    assert(near(retract_fsm.Get_Position_Feedback(1U), deg(90.0f)));
    assert(near(retract_fsm.Get_Position_Feedback(2U), deg(-10.0f)));

    // Valid IMU allows retracting and retracted hold.
    retract_fsm.Update(true, true, true, true, 4.5f,
                       4.5f, deg(90.0f), deg(-10.0f),
                       10350U, true);
    assert(retract_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTING);
    retract_fsm.Update(true, true, true, true, 4.5f,
                       4.5f, 2.0f, deg(-30.0f), 11350U, true);
    assert(retract_fsm.Get_State() ==
           UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD);
    assert(retract_fsm.Uses_Retract_Position_Control());
    assert(!retract_fsm.Uses_Attitude_Control());
    assert(near(retract_fsm.Get_Position_Feedback(1U), 2.0f));
    assert(near(retract_fsm.Get_Position_Feedback(2U), deg(-30.0f)));
    assert(near(retract_fsm.Limit_Torque(1U, 2.0f), 2.0f));

    // Retracted hold continues holding both targets with valid IMU.
    retract_fsm.Update(true, true, true, true, 4.5f,
                       4.5f, 2.0f, deg(-30.0f), 11400U, true);
    assert(retract_fsm.Get_State() ==
           UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD);
    assert(near(retract_fsm.Get_Retract_Target_Angle(1U), 2.0f));
    assert(near(retract_fsm.Get_Retract_Target_Angle(2U), deg(-30.0f)));

    // Normal cancellation of retract resumes attitude at full output.
    retract_fsm.Update(true, true, true, true, 4.5f, 12.0f,
                       2.0f, deg(-30.0f), 11500U, false);
    assert(retract_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    assert(near(retract_fsm.Get_Output_Scale(), 1.0f));
    retract_fsm.Update(true, true, true, true, 4.5f, 12.0f,
                       2.0f, deg(-30.0f), 15500U, false);
    assert(retract_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);

    // Clearing the request is immediate even before retraction completes.
    Class_Up_Stair_Behind_Motor_FSM retract_resume_fsm(valid_config());
    enter_attitude_hold(retract_resume_fsm, 25000U);
    retract_resume_fsm.Update(true, true, true, true, 4.5f, 12.0f, deg(90.0f), deg(-10.0f), 25300U,
                              true);
    retract_resume_fsm.Update(true, true, true, true, 4.5f, 12.0f, deg(90.0f), deg(-10.0f), 25301U,
                              false);
    assert(retract_resume_fsm.Get_State() ==
           UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    assert(near(retract_resume_fsm.Get_Output_Scale(), 1.0f));

    // Any invalid retract feedback disables both legs and zeros both torque
    // paths instead of partially controlling the healthy side.
    Class_Up_Stair_Behind_Motor_FSM fault_fsm(valid_config());
    enter_attitude_hold(fault_fsm, 14000U);
    fault_fsm.Update(true, true, true, true, 4.5f, 12.0f,
                     deg(90.0f), deg(-10.0f), 14300U, true);
    fault_fsm.Update(true, false, true, false, nan_value,
                     nan_value, 2.0f, deg(-30.0f), 14301U, true);
    assert(fault_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    assert(!fault_fsm.Is_Motor_Controllable(1U));
    assert(!fault_fsm.Is_Motor_Controllable(2U));
    assert(near(fault_fsm.Limit_Torque(1U, 2.0f), 0.0f));
    assert(near(fault_fsm.Limit_Torque(2U, 2.0f), 0.0f));

    // Zero/un-calibrated retract fields remain fail-safe even with valid
    // normal attitude calibration.
    Class_Up_Stair_Behind_Motor_FSM::Config no_retract = valid_config();
    no_retract.retract_target_rad[0] = 0.0f;
    no_retract.retract_target_rad[1] = 0.0f;
    no_retract.retract_position_tolerance_rad = 0.0f;
    Class_Up_Stair_Behind_Motor_FSM no_retract_fsm(no_retract);
    enter_attitude_hold(no_retract_fsm, 16000U);
    no_retract_fsm.Update(true, true, true, true, 4.5f, 12.0f, deg(90.0f), deg(-10.0f), 16300U, true);
    assert(no_retract_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);

    Class_Up_Stair_Behind_Motor_FSM::Config zero_tolerance = valid_config();
    zero_tolerance.retract_position_tolerance_rad = 0.0f;
    Class_Up_Stair_Behind_Motor_FSM zero_tolerance_fsm(zero_tolerance);
    enter_attitude_hold(zero_tolerance_fsm, 17000U);
    zero_tolerance_fsm.Update(true, true, true, true, 4.5f,
                              12.0f, deg(90.0f), deg(-10.0f),
                              17300U, true);
    assert(zero_tolerance_fsm.Get_State() ==
           UP_STAIR_BEHIND_MOTOR_DISABLED);

    // A wrap-around safe interval accepts a raw retract target below its
    // start angle and exposes the equivalent continuous target.
    Class_Up_Stair_Behind_Motor_FSM::Config wrapped_target = valid_config();
    wrapped_target.angle_start_rad[1] = deg(150.0f);
    wrapped_target.angle_end_rad[1] = deg(-150.0f);
    wrapped_target.retract_target_rad[1] = deg(-170.0f);
    Class_Up_Stair_Behind_Motor_FSM wrapped_target_fsm(wrapped_target);
    wrapped_target_fsm.Update(true, true, true, true, 4.5f,
                              12.0f, deg(90.0f), deg(170.0f),
                              18000U);
    wrapped_target_fsm.Update(true, true, true, true, 4.5f,
                              12.0f, deg(90.0f), deg(170.0f),
                              22000U);
    assert(wrapped_target_fsm.Get_State() ==
           UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    wrapped_target_fsm.Update(true, true, true, true, 4.5f,
                               12.0f, deg(90.0f), deg(170.0f),
                               22001U, true);
    assert(wrapped_target_fsm.Get_State() ==
           UP_STAIR_BEHIND_MOTOR_RETRACTING);
    assert(near(wrapped_target_fsm.Get_Retract_Target_Angle(2U),
                deg(190.0f)));
    wrapped_target_fsm.Update(true, true, true, true, 4.5f,
                               4.5f, 2.0f, deg(-170.0f),
                               23301U, true);
    assert(wrapped_target_fsm.Get_State() ==
           UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD);
    assert(near(wrapped_target_fsm.Get_Retract_Target_Angle(2U),
                deg(190.0f)));
    assert(near(wrapped_target_fsm.Get_Position_Feedback(2U), deg(190.0f)));

    // Wrapped interval tolerance must not unfold the lower fringe by a turn.
    Class_Up_Stair_Behind_Motor_FSM wrapped_margin_fsm(wrapped_target);
    wrapped_margin_fsm.Update(true, true, true, true, 0, 0, deg(90), deg(149.8f), 0U);
    wrapped_margin_fsm.Update(true, true, true, true, 0, 0, deg(90), deg(149.8f), 4000U);
    assert(wrapped_margin_fsm.Is_Motor_Controllable(2U));
    assert(near(wrapped_margin_fsm.Get_Position_Feedback(2U), deg(149.8f)));
    assert(near(wrapped_margin_fsm.Limit_Torque(2U, -4), 0));
    assert(near(wrapped_margin_fsm.Limit_Torque(2U, 4), 4));
    wrapped_margin_fsm.Update(true, true, true, true, 0, 0, deg(90), deg(-149.8f), 4001U);
    assert(near(wrapped_margin_fsm.Get_Position_Feedback(2U), deg(210.2f)));
    assert(near(wrapped_margin_fsm.Limit_Torque(2U, 4), 0));
    assert(near(wrapped_margin_fsm.Limit_Torque(2U, -4), -4));

    // A target outside [-π, +π] must not be accepted even when its unwrapped
    // value lies inside the mechanical interval.
    Class_Up_Stair_Behind_Motor_FSM::Config invalid_wrapped_target =
        wrapped_target;
    invalid_wrapped_target.retract_target_rad[1] = PI + deg(10.0f);
    Class_Up_Stair_Behind_Motor_FSM invalid_wrapped_fsm(
        invalid_wrapped_target);
    invalid_wrapped_fsm.Update(true, true, true, true, 4.5f,
                               12.0f, deg(90.0f), deg(170.0f),
                               18000U);
    invalid_wrapped_fsm.Update(true, true, true, true, 4.5f,
                               12.0f, deg(90.0f), deg(170.0f),
                               22000U);
    invalid_wrapped_fsm.Update(true, true, true, true, 4.5f,
                               12.0f, deg(90.0f), deg(170.0f),
                               22001U, true);
    assert(invalid_wrapped_fsm.Get_State() ==
           UP_STAIR_BEHIND_MOTOR_DISABLED);
    return 0;
}
