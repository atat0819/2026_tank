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
    config.pitch_zero_deg = 2.5f;
    config.roll_zero_deg = -1.5f;
    config.angle_start_rad[0] = deg(10.0f);
    config.angle_end_rad[0] = deg(170.0f);
    config.angle_start_rad[1] = deg(300.0f);
    config.angle_end_rad[1] = deg(60.0f);
    config.motor_direction[0] = 1;
    config.motor_direction[1] = -1;
    return config;
}

void update_valid(Class_Up_Stair_Behind_Motor_FSM &fsm, uint32_t tick)
{
    fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f, -7.0f,
               deg(90.0f), deg(350.0f), tick);
}
}

int main()
{
    // The default/unmeasured configuration must remain safely disabled.
    Class_Up_Stair_Behind_Motor_FSM default_fsm;
    default_fsm.Update(true, true, true, true, 0.0f, 0.0f, 0.0f, 0.0f,
                       deg(90.0f), deg(350.0f), 0U);
    assert(default_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    assert(near(default_fsm.Get_Output_Scale(), 0.0f));
    assert(near(default_fsm.Limit_Torque(1U, 2.0f), 0.0f));

    // 左右电机的最终力矩增益可独立校准，默认值 1.0 不改变原有输出。
    Class_Up_Stair_Behind_Motor_FSM::Config gain_config = valid_config();
    assert(near(gain_config.motor_torque_gain[0], 1.0f));
    assert(near(gain_config.motor_torque_gain[1], 1.0f));
    gain_config.motor_torque_gain[0] = 1.25f;
    gain_config.motor_torque_gain[1] = 0.80f;
    Class_Up_Stair_Behind_Motor_FSM gain_fsm(gain_config);
    update_valid(gain_fsm, 10000U);
    update_valid(gain_fsm, 10300U);
    assert(near(gain_fsm.Limit_Torque(1U, 2.0f), 2.50f));
    assert(near(gain_fsm.Limit_Torque(2U, 2.0f), 1.60f));

    Class_Up_Stair_Behind_Motor_FSM fsm(valid_config());
    assert(!fsm.Is_Motor_Controllable(1U));
    assert(!fsm.Is_Motor_Controllable(2U));

    // A valid update enters recovery at the current tick and ramps for 300 ms.
    update_valid(fsm, 1000U);
    assert(fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(fsm.Get_Output_Scale(), 0.0f));
    assert(near(fsm.Limit_Torque(1U, 2.0f), 0.0f));
    fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f, -7.0f,
               deg(90.0f), deg(350.0f), 1150U);
    assert(near(fsm.Get_Output_Scale(), 0.5f));
    assert(fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(fsm.Limit_Torque(1U, 2.0f), 1.0f));
    fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f, -7.0f,
               deg(90.0f), deg(350.0f), 1300U);
    assert(fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    assert(near(fsm.Get_Output_Scale(), 1.0f));
    assert(near(fsm.Limit_Torque(1U, 2.0f), 2.0f));

    // Calibration is applied by the FSM; rates are passed through unchanged.
    assert(near(fsm.Get_Target_Pitch_Deg(), 0.0f));
    assert(near(fsm.Get_Target_Roll_Deg(), 0.0f));
    assert(near(fsm.Get_Feedback_Pitch_Deg(), 2.0f));
    assert(near(fsm.Get_Feedback_Roll_Deg(), -2.0f));
    assert(near(fsm.Get_Pitch_Rate_Dps(), 12.0f));
    assert(near(fsm.Get_Roll_Rate_Dps(), -7.0f));

    // Invalid IMU data disables output and a later valid sample restarts
    // recovery from zero at its new tick.
    Class_Up_Stair_Behind_Motor_FSM imu_fsm(valid_config());
    update_valid(imu_fsm, 3000U);
    imu_fsm.Update(true, false, true, true, 4.5f, -3.5f, 12.0f, -7.0f,
                   deg(90.0f), deg(350.0f), 3010U);
    assert(imu_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    assert(near(imu_fsm.Get_Output_Scale(), 0.0f));

    // Nonfinite attitude/rate samples disable safely and clear all stored
    // feedback, so stale active data cannot leak to control or torque output.
    const float nan_value = std::numeric_limits<float>::quiet_NaN();
    const float inf_value = std::numeric_limits<float>::infinity();
    Class_Up_Stair_Behind_Motor_FSM numerical_fsm(valid_config());
    update_valid(numerical_fsm, 5000U);
    numerical_fsm.Update(true, true, true, true, nan_value, -3.5f, 12.0f,
                         -7.0f, deg(90.0f), deg(350.0f), 5300U);
    assert(numerical_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    assert(near(numerical_fsm.Get_Output_Scale(), 0.0f));
    assert(near(numerical_fsm.Get_Feedback_Pitch_Deg(), 0.0f));
    assert(near(numerical_fsm.Get_Feedback_Roll_Deg(), 0.0f));
    assert(near(numerical_fsm.Get_Pitch_Rate_Dps(), 0.0f));
    assert(near(numerical_fsm.Get_Roll_Rate_Dps(), 0.0f));
    assert(near(numerical_fsm.Limit_Torque(1U, 2.0f), 0.0f));
    update_valid(numerical_fsm, 6000U);
    assert(numerical_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(numerical_fsm.Get_Output_Scale(), 0.0f));
    numerical_fsm.Update(true, true, true, true, 4.5f, inf_value, 12.0f,
                         -7.0f, deg(90.0f), deg(350.0f), 6300U);
    assert(numerical_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    assert(near(numerical_fsm.Get_Output_Scale(), 0.0f));
    assert(near(numerical_fsm.Get_Feedback_Pitch_Deg(), 0.0f));
    assert(near(numerical_fsm.Get_Feedback_Roll_Deg(), 0.0f));
    assert(near(numerical_fsm.Get_Pitch_Rate_Dps(), 0.0f));
    assert(near(numerical_fsm.Get_Roll_Rate_Dps(), 0.0f));
    assert(near(numerical_fsm.Limit_Torque(1U, 2.0f), 0.0f));
    assert(near(imu_fsm.Limit_Torque(1U, 2.0f), 0.0f));
    update_valid(imu_fsm, 4000U);
    assert(imu_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(imu_fsm.Get_Output_Scale(), 0.0f));

    // One feedback channel failing only removes that motor's permission.
    fsm.Update(true, true, false, true, 4.5f, -3.5f, 12.0f, -7.0f,
               deg(90.0f), deg(350.0f), 1301U);
    assert(!fsm.Is_Motor_Controllable(1U));
    assert(fsm.Is_Motor_Controllable(2U));
    assert(near(fsm.Limit_Torque(1U, 1.0f), 0.0f));
    assert(!near(fsm.Limit_Torque(2U, 1.0f), 0.0f));

    // Disable resets recovery and output permission.
    fsm.Update(false, true, true, true, 4.5f, -3.5f, 12.0f, -7.0f,
               deg(90.0f), deg(350.0f), 1400U);
    assert(fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    assert(near(fsm.Get_Output_Scale(), 0.0f));
    assert(near(fsm.Limit_Torque(2U, 1.0f), 0.0f));
    update_valid(fsm, 2000U);
    assert(fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(fsm.Get_Output_Scale(), 0.0f));

    // Normal interval: lower margin blocks negative torque and upper blocks
    // positive torque, while escape torque remains available.
    fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f, -7.0f,
               deg(12.0f), deg(350.0f), 2300U);
    assert(near(fsm.Get_Output_Scale(), 1.0f));
    assert(near(fsm.Limit_Torque(1U, -3.0f), 0.0f));
    assert(near(fsm.Limit_Torque(1U, 3.0f), 3.0f));
    fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f, -7.0f,
               deg(168.0f), deg(350.0f), 2301U);
    assert(near(fsm.Limit_Torque(1U, 3.0f), 0.0f));
    assert(near(fsm.Limit_Torque(1U, -3.0f), -3.0f));

    // Wrapped interval maps values below start above 2*pi and protects both
    // physical ends of the continuous interval.
    assert(fsm.Is_Angle_Valid(2U, deg(350.0f)));
    assert(fsm.Is_Angle_Valid(2U, deg(10.0f)));
    assert(!fsm.Is_Angle_Valid(2U, deg(250.0f)));
    fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f, -7.0f,
               deg(90.0f), deg(302.0f), 2302U);
    assert(near(fsm.Limit_Torque(2U, -4.0f), 0.0f));
    assert(near(fsm.Limit_Torque(2U, 4.0f), 4.0f));
    fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f, -7.0f,
               deg(90.0f), deg(58.0f), 2303U);
    assert(near(fsm.Limit_Torque(2U, 4.0f), 0.0f));
    assert(near(fsm.Limit_Torque(2U, -4.0f), -4.0f));

    // Both interval types reject angles outside the encoder's raw domain.
    const float above_raw_domain = 2.0f * PI + deg(1.0f);
    assert(!fsm.Is_Angle_Valid(1U, deg(-1.0f)));
    assert(!fsm.Is_Angle_Valid(1U, above_raw_domain));
    assert(!fsm.Is_Angle_Valid(2U, deg(-1.0f)));
    assert(!fsm.Is_Angle_Valid(2U, above_raw_domain));
    fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f, -7.0f,
               deg(-1.0f), deg(350.0f), 2304U);
    assert(!fsm.Is_Motor_Controllable(1U));
    assert(near(fsm.Limit_Torque(1U, 2.0f), 0.0f));
    fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f, -7.0f,
               deg(90.0f), above_raw_domain, 2305U);
    assert(!fsm.Is_Motor_Controllable(2U));
    assert(near(fsm.Limit_Torque(2U, 2.0f), 0.0f));

    // Invalid torque requests and public IDs are always rejected.
    assert(near(fsm.Limit_Torque(1U, nan_value), 0.0f));
    assert(near(fsm.Limit_Torque(1U, inf_value), 0.0f));
    assert(near(fsm.Limit_Torque(0U, 1.0f), 0.0f));
    assert(near(fsm.Limit_Torque(3U, 1.0f), 0.0f));

    // Losing one feedback suppresses only that motor. Once both feedbacks
    // return, the shared FSM restarts its 300 ms recovery ramp.
    Class_Up_Stair_Behind_Motor_FSM feedback_fsm(valid_config());
    update_valid(feedback_fsm, 7000U);
    feedback_fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f,
                        -7.0f, deg(90.0f), deg(350.0f), 7300U);
    assert(feedback_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    feedback_fsm.Update(true, true, false, true, 4.5f, -3.5f, 12.0f,
                        -7.0f, deg(90.0f), deg(350.0f), 7301U);
    assert(feedback_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    assert(!feedback_fsm.Is_Motor_Controllable(1U));
    assert(feedback_fsm.Is_Motor_Controllable(2U));
    assert(near(feedback_fsm.Limit_Torque(2U, 2.0f), 2.0f));
    feedback_fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f,
                        -7.0f, deg(90.0f), deg(350.0f), 7302U);
    assert(feedback_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(feedback_fsm.Get_Output_Scale(), 0.0f));
    assert(near(feedback_fsm.Limit_Torque(1U, 2.0f), 0.0f));
    feedback_fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f,
                        -7.0f, deg(90.0f), deg(350.0f), 7452U);
    assert(near(feedback_fsm.Get_Output_Scale(), 0.5f));
    feedback_fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f,
                        -7.0f, deg(90.0f), deg(350.0f), 7602U);
    assert(feedback_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    assert(near(feedback_fsm.Get_Output_Scale(), 1.0f));

    // Staggered return after both feedbacks are lost restarts recovery for
    // each valid transition; no returning motor gets immediate torque.
    Class_Up_Stair_Behind_Motor_FSM staggered_fsm(valid_config());
    update_valid(staggered_fsm, 8000U);
    staggered_fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f,
                         -7.0f, deg(90.0f), deg(350.0f), 8300U);
    staggered_fsm.Update(true, true, false, false, 4.5f, -3.5f, 12.0f,
                         -7.0f, deg(90.0f), deg(350.0f), 8301U);
    assert(!staggered_fsm.Is_Motor_Controllable(1U));
    assert(!staggered_fsm.Is_Motor_Controllable(2U));
    staggered_fsm.Update(true, true, true, false, 4.5f, -3.5f, 12.0f,
                         -7.0f, deg(90.0f), deg(350.0f), 8302U);
    assert(staggered_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(staggered_fsm.Get_Output_Scale(), 0.0f));
    assert(staggered_fsm.Is_Motor_Controllable(1U));
    assert(!staggered_fsm.Is_Motor_Controllable(2U));
    assert(near(staggered_fsm.Limit_Torque(1U, 2.0f), 0.0f));
    assert(near(staggered_fsm.Limit_Torque(2U, 2.0f), 0.0f));
    staggered_fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f,
                         -7.0f, deg(90.0f), deg(350.0f), 8303U);
    assert(staggered_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(staggered_fsm.Get_Output_Scale(), 0.0f));
    assert(near(staggered_fsm.Limit_Torque(1U, 2.0f), 0.0f));
    assert(near(staggered_fsm.Limit_Torque(2U, 2.0f), 0.0f));
    staggered_fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f,
                         -7.0f, deg(90.0f), deg(350.0f), 8453U);
    assert(near(staggered_fsm.Get_Output_Scale(), 0.5f));
    staggered_fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f,
                         -7.0f, deg(90.0f), deg(350.0f), 8603U);
    assert(staggered_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
    assert(near(staggered_fsm.Limit_Torque(1U, 2.0f), 2.0f));

    // Finite raw data can still overflow during calibrated subtraction.
    Class_Up_Stair_Behind_Motor_FSM::Config overflow_config = valid_config();
    overflow_config.pitch_zero_deg = -std::numeric_limits<float>::max();
    Class_Up_Stair_Behind_Motor_FSM overflow_fsm(overflow_config);
    overflow_fsm.Update(true, true, true, true,
                        std::numeric_limits<float>::max(), -3.5f, 12.0f,
                        -7.0f, deg(90.0f), deg(350.0f), 9000U);
    assert(overflow_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    assert(near(overflow_fsm.Get_Feedback_Pitch_Deg(), 0.0f));
    assert(near(overflow_fsm.Get_Output_Scale(), 0.0f));
    assert(near(overflow_fsm.Limit_Torque(1U, 2.0f), 0.0f));

    // Unsigned tick subtraction keeps recovery deterministic across wrap.
    Class_Up_Stair_Behind_Motor_FSM wrap_fsm(valid_config());
    update_valid(wrap_fsm, 0xFFFFFF00U);
    wrap_fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f, -7.0f,
                    deg(90.0f), deg(350.0f), 0x00000000U);
    assert(wrap_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    assert(near(wrap_fsm.Get_Output_Scale(), 256.0f / 300.0f));
    wrap_fsm.Update(true, true, true, true, 4.5f, -3.5f, 12.0f, -7.0f,
                    deg(90.0f), deg(350.0f), 44U);
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
    bad.pitch_zero_deg = nan_value;
    Class_Up_Stair_Behind_Motor_FSM bad_offset(bad);
    update_valid(bad_offset, 0U);
    assert(bad_offset.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    bad = valid_config();
    bad.angle_start_rad[0] = -0.01f;
    Class_Up_Stair_Behind_Motor_FSM bad_negative_limit(bad);
    update_valid(bad_negative_limit, 0U);
    assert(bad_negative_limit.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    bad = valid_config();
    bad.angle_end_rad[0] = 2.0f * PI + 0.01f;
    Class_Up_Stair_Behind_Motor_FSM bad_large_limit(bad);
    update_valid(bad_large_limit, 0U);
    assert(bad_large_limit.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    bad = valid_config();
    bad.angle_start_rad[1] = 2.0f * PI;
    bad.angle_end_rad[1] = 0.0f;
    Class_Up_Stair_Behind_Motor_FSM bad_alias_limit(bad);
    update_valid(bad_alias_limit, 0U);
    assert(bad_alias_limit.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
    return 0;
}
