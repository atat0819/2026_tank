#include "fsm/up_stair_behind_motor_fsm.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>

using Rear = Class_Up_Stair_Behind_Motor_FSM;
float deg(float x) { return x * 3.14159265358979323846f / 180.0f; }
bool near(float a, float b) { return std::fabs(a - b) < 1e-5f; }

int main()
{
    const auto config = Rear::Config::WithAngleConstants();
    Rear rear(config);
    const float left = config.angle_start_rad[0] + Rear::EXTENSION_MARGIN_RAD;
    const float right = config.angle_end_rad[1] - Rear::EXTENSION_MARGIN_RAD;
    const float kd = 3.5f;
    auto update = [&](uint32_t tick, float pitch, float l, float r,
                      bool imu = true, bool retract = false) {
        rear.Update(true, imu, true, true, pitch, 0, l, r, tick, retract);
    };

    // Normal travel retains the attitude velocity path.
    update(0, 6, deg(60), deg(40));
    assert(near(rear.Limit_Velocity(1, -2, kd), -2));
    assert(near(rear.Limit_Velocity(2, 2, kd), 2));
    // Extended ends: stop at the reserved encoder position for any tilt.
    update(1, 6, left, right);
    assert(near(rear.Limit_Velocity(1, -2, kd), 0));
    assert(near(rear.Limit_Velocity(2, 2, kd), 0));
    assert(near(rear.Get_Target_Pitch_Deg(), 1.0f));
    assert(near(rear.Get_Feedback_Pitch_Deg(), 6.0f));
    assert(near(rear.Get_Target_Pitch_Rad(), deg(1)));

    // Falling inward must not unlatch; fixed targets pull back toward the boundary.
    update(2, 6, left + deg(1), right - deg(1));
    const float lv = rear.Limit_Velocity(1, -2, kd);
    const float rv = rear.Limit_Velocity(2, 2, kd);
    assert(lv < 0 && rv > 0);
    update(3, 21, left + deg(1), right - deg(1));
    assert(near(rear.Limit_Velocity(1, -8, kd), lv));
    assert(near(rear.Limit_Velocity(2, 8, kd), rv));
    // At the Pitch target, keep supporting instead of dropping the position loop.
    update(4, 1, left + deg(1), right - deg(1));
    assert(near(rear.Limit_Velocity(1, 0, kd), lv));
    assert(near(rear.Limit_Velocity(2, 0, kd), rv));
    // A small reverse request and a short impulse cannot release the hold.
    update(5, 0.9f, left + deg(1), right - deg(1));
    assert(near(rear.Limit_Velocity(1, 0.1f, kd), lv));
    update(10, 0, left + deg(1), right - deg(1));
    update(10 + Rear::EXTENSION_RELEASE_TIME_MS - 1, 0, left + deg(1), right - deg(1));
    assert(near(rear.Limit_Velocity(1, 1, kd), lv));
    update(10 + Rear::EXTENSION_RELEASE_TIME_MS, 0, left + deg(1), right - deg(1));
    assert(near(rear.Limit_Velocity(1, 1, kd), 1));
    assert(near(rear.Limit_Velocity(2, -1, kd), -1));

    // A tiny outward error still activates the reserved boundary.
    update(111, 1.01f, left, right);
    assert(near(rear.Limit_Velocity(1, -0.01f, kd), 0));
    assert(near(rear.Limit_Velocity(2, 0.01f, kd), 0));
    // Overshoot is corrected inward; large position error respects the hold speed cap.
    update(112, 6, left - deg(1), right + deg(1));
    assert(rear.Limit_Velocity(1, -2, kd) > 0);
    assert(rear.Limit_Velocity(2, 2, kd) < 0);
    update(113, 6, deg(60), deg(40));
    const float l_expected = fmaxf(-30 / kd, fmaxf(-Rear::EXTENSION_HOLD_MAX_SPEED_RAD_S,
        Rear::EXTENSION_HOLD_KP * (left - deg(60))));
    const float r_expected = fminf(30 / kd, fminf(Rear::EXTENSION_HOLD_MAX_SPEED_RAD_S,
        Rear::EXTENSION_HOLD_KP * (right - deg(40))));
    assert(near(rear.Limit_Velocity(1, -2, kd), l_expected));
    assert(near(rear.Limit_Velocity(2, 2, kd), r_expected));

    // Invalid IMU clears holding, retains the fault ramp, then resumes normal travel.
    update(114, 6, deg(60), deg(40), false);
    assert(!rear.Is_Motor_Controllable(1) && !rear.Is_Motor_Controllable(2));
    update(115, 6, deg(60), deg(40));
    assert(near(rear.Get_Output_Scale(), 0));
    update(2115, 6, deg(60), deg(40));
    assert(near(rear.Limit_Velocity(1, -2, kd), -2));
    // Only the side at the boundary is held.
    update(2116, 6, left, deg(40));
    assert(near(rear.Limit_Velocity(1, -2, kd), 0));
    assert(near(rear.Limit_Velocity(2, 2, kd), 2));
    // Retract selection clears the hold and retains the existing target.
    update(2117, 6, left, right, true, true);
    assert(rear.Uses_Retract_Position_Control());
    assert(near(rear.Get_Position_Target_Angle(1), config.retract_target_rad[0]));
    update(2118, 6, deg(60), deg(40));
    assert(near(rear.Limit_Velocity(1, -2, kd), -2));

    // Release debounce works across unsigned tick rollover and resets on noise.
    const uint32_t wrap_start = 0xFFFFFFF0U;
    update(wrap_start - 1, 6, left, right);
    update(wrap_start, 0, left, right);
    update(wrap_start + Rear::EXTENSION_RELEASE_TIME_MS - 1, 0, left, right);
    assert(near(rear.Limit_Velocity(1, 1, kd), 0));
    const uint32_t restart = wrap_start + Rear::EXTENSION_RELEASE_TIME_MS + 1;
    update(restart - 1, 1, left, right);
    update(restart, 0, left, right);
    update(restart + Rear::EXTENSION_RELEASE_TIME_MS - 1, 0, left, right);
    assert(near(rear.Limit_Velocity(1, 1, kd), 0));
    update(restart + Rear::EXTENSION_RELEASE_TIME_MS, 0, left, right);
    assert(near(rear.Limit_Velocity(1, 1, kd), 1));
    std::puts("PASS: rear extension boundary, fixed hold, independent sides, release, faults and raw Pitch target");
}
