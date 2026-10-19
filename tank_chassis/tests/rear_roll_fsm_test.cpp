#include "fsm/up_stair_behind_motor_fsm.hpp"
#include <cassert>
#include <cmath>
#include <initializer_list>

using Rear = Class_Up_Stair_Behind_Motor_FSM;
bool near(float a, float b) { return std::fabs(a - b) < 1e-5f; }
void update(Rear &fsm, uint32_t tick, float roll, bool enabled = true,
            bool imu = true, bool retract = false, bool left_valid = true)
{
    fsm.Update(enabled, imu, left_valid, true, 1.0f, 0.0f,
               1.0f, 0.7f, tick, retract, 0.0f, 0.0f, roll);
}

int main()
{
    auto config = Rear::Config::WithAngleConstants();
    assert(config.roll_target_deg == Rear::ROLL_TARGET_DEG);
    assert(config.roll_kp == Rear::ROLL_KP);
    assert(config.roll_max_torque_nm == Rear::ROLL_MAX_TORQUE_NM);
    // A custom reference/gain verifies the FSM uses its own configuration.
    config.roll_target_deg = 2.0f;
    config.roll_kp = 0.5f;
    config.roll_max_torque_nm = 2.0f;
    Rear fsm(config);
    assert(fsm.Is_Config_Valid());
    assert(fsm.Get_Raw_Roll_Torque(1) == 0);
    for (float deviation : {-10.0f, -1.0f, 0.0f, 1.0f, 10.0f}) {
        update(fsm, 100, config.roll_target_deg + deviation);
        const float expected = deviation == 0 ? 0 :
            std::copysign(std::fabs(deviation) == 1 ? 0.5f : 2.0f, deviation);
        assert(near(fsm.Get_Raw_Roll_Torque(1), expected));
        assert(near(fsm.Get_Raw_Roll_Torque(2), expected));
    }
    assert(fsm.Get_Raw_Roll_Torque(0) == 0 && fsm.Get_Raw_Roll_Torque(3) == 0);
    update(fsm, 101, 12, true, true, true);
    assert(fsm.Uses_Position_Control());
    assert(fsm.Get_Raw_Roll_Torque(1) == 0 && fsm.Get_Raw_Roll_Torque(2) == 0);
    update(fsm, 102, 12, false);
    assert(fsm.Get_Raw_Roll_Torque(1) == 0);
    update(fsm, 103, NAN);
    assert(!fsm.Uses_Attitude_Control());
    assert(fsm.Get_Raw_Roll_Torque(1) == 0);
    update(fsm, 104, 12);
    // Raw getter does not apply the recovery ramp. Protection applies it once.
    assert(near(fsm.Get_Raw_Roll_Torque(1), 2.0f));
    assert(near(fsm.Limit_Torque(1, fsm.Get_Raw_Roll_Torque(1)), 0.0f));
    update(fsm, 1104, 12);
    assert(near(fsm.Get_Raw_Roll_Torque(1), 2.0f));
    assert(near(fsm.Limit_Torque(1, fsm.Get_Raw_Roll_Torque(1)), 1.0f));
    update(fsm, 2104, 12);
    assert(near(fsm.Limit_Torque(1, fsm.Get_Raw_Roll_Torque(1)), 2.0f));
    update(fsm, 2105, 12, true, true, false, false);
    assert(fsm.Get_Raw_Roll_Torque(1) == 0 && fsm.Get_Raw_Roll_Torque(2) == 0);

    config.motor_direction[0] = -1;
    config.motor_direction[1] = 1;
    Rear reversed(config);
    update(reversed, 0, 3);
    assert(near(reversed.Get_Raw_Roll_Torque(1), -0.5f));
    assert(near(reversed.Get_Raw_Roll_Torque(2), -0.5f));
    config.roll_kp = 0;
    Rear zero_gain(config);
    update(zero_gain, 0, 12);
    assert(zero_gain.Get_Raw_Roll_Torque(1) == 0);
    config.roll_kp = 0.5f;
    config.roll_max_torque_nm = 0;
    Rear zero_cap(config);
    update(zero_cap, 0, 12);
    assert(zero_cap.Get_Raw_Roll_Torque(1) == 0);
    for (float invalid : {-1.0f, NAN, INFINITY}) {
        auto bad = config;
        bad.roll_kp = invalid;
        assert(!Rear(bad).Is_Config_Valid());
        bad = config;
        bad.roll_max_torque_nm = invalid;
        assert(!Rear(bad).Is_Config_Valid());
    }
    config.roll_target_deg = NAN;
    assert(!Rear(config).Is_Config_Valid());
}
