#include "../fsm/up_stair_behind_motor_fsm.hpp"
#include <assert.h>
#include <math.h>

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

    Class_Up_Stair_Behind_Motor_FSM fsm(valid_config());

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
               deg(12.0f), deg(168.0f), 2300U);
    assert(near(fsm.Get_Output_Scale(), 1.0f));
    assert(near(fsm.Limit_Torque(1U, -3.0f), 0.0f));
    assert(near(fsm.Limit_Torque(1U, 3.0f), 3.0f));
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
    return 0;
}
