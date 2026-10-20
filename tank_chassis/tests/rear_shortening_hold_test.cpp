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
    const float margin = Rear::EXTENSION_MARGIN_RAD;
    const float left = config.angle_end_rad[0] - margin;
    const float right = config.angle_start_rad[1] + margin;
    const float extended_left = config.angle_start_rad[0] + margin;
    const float extended_right = config.angle_end_rad[1] - margin;
    const float target = config.pitch_target_deg;
    const float reverse = Rear::EXTENSION_RELEASE_PITCH_ERROR_DEG + 1.0f;
    const float kd = 3.6f;
    Rear rear(config);
    auto update = [&](uint32_t tick, float pitch, float l, float r,
                      bool enabled = true, bool imu = true, bool retract = false,
                      bool left_online = true, bool right_online = true) {
        rear.Update(enabled, imu, left_online, right_online, pitch, 0, l, r, tick, retract);
    };

    update(0, target - 5, deg(60), deg(40));
    assert(near(rear.Limit_Velocity(1, 2, kd), 2));
    assert(near(rear.Limit_Velocity(2, -2, kd), -2));
    // The first failure on the old code: shortening continues through the margin.
    update(1, target - 5, left, right);
    assert(near(rear.Limit_Velocity(1, 2, kd), 0));
    assert(near(rear.Limit_Velocity(2, -2, kd), 0));

    // Drifting away cannot release the hold or move its fixed targets.
    update(2, target - 5, left - deg(0.5f), right + deg(0.5f));
    const float speed = fminf(Rear::SHORTENING_HOLD_MAX_SPEED_RAD_S,
                            Rear::SHORTENING_HOLD_KP * deg(0.5f));
    assert(near(rear.Limit_Velocity(1, 2, kd), speed));
    assert(near(rear.Limit_Velocity(2, -2, kd), -speed));
    update(3, target, left - deg(0.5f), right + deg(0.5f));
    assert(near(rear.Limit_Velocity(1, 0, kd), speed));
    assert(near(rear.Limit_Velocity(2, 0, kd), -speed));
    update(4, target + 0.5f * Rear::EXTENSION_RELEASE_PITCH_ERROR_DEG,
           left - deg(0.5f), right + deg(0.5f));
    assert(near(rear.Limit_Velocity(1, -1, kd), speed));

    // Noise interrupts release timing; an extension request must persist.
    uint32_t now = 10;
    update(now, target + reverse, left, right);
    update(now + Rear::EXTENSION_RELEASE_TIME_MS - 1, target + reverse, left, right);
    assert(near(rear.Limit_Velocity(1, -1, kd), 0));
    now += Rear::EXTENSION_RELEASE_TIME_MS;
    update(now++, target, left, right);
    update(now, target + reverse, left, right);
    update(now + Rear::EXTENSION_RELEASE_TIME_MS - 1, target + reverse, left, right);
    assert(near(rear.Limit_Velocity(2, 1, kd), 0));
    now += Rear::EXTENSION_RELEASE_TIME_MS;
    update(now++, target + reverse, left, right);
    assert(near(rear.Limit_Velocity(1, -1, kd), -1));
    assert(near(rear.Limit_Velocity(2, 1, kd), 1));
    // Even after release, an outward combined request remains bounded.
    assert(near(rear.Limit_Velocity(1, 2, kd), 0));
    assert(near(rear.Limit_Velocity(2, -2, kd), 0));

    // Opposite ends on different legs need opposite release conditions.
    rear = Rear(config);
    update(now++, target, extended_left, right);
    update(now, target - reverse, extended_left, right);
    now += Rear::EXTENSION_RELEASE_TIME_MS;
    update(now++, target - reverse, extended_left, right);
    assert(near(rear.Limit_Velocity(1, 1, kd), 1));
    assert(near(rear.Limit_Velocity(2, -1, kd), 0));
    update(now, target + reverse, extended_left, right);
    now += Rear::EXTENSION_RELEASE_TIME_MS;
    update(now++, target + reverse, extended_left, right);
    assert(near(rear.Limit_Velocity(1, -1, kd), 0));
    assert(near(rear.Limit_Velocity(2, 1, kd), 1));

    // Retreat from either endpoint still supports the existing recovery/safety paths.
    rear = Rear(config);
    update(now++, target - 5, left, right);
    update(now++, target - 5, deg(60), deg(40), false);
    update(now++, target - 5, deg(60), deg(40));
    assert(near(rear.Limit_Velocity(1, 2, kd), 2));
    update(now++, target - 5, left, right);
    update(now++, target - 5, left, right, true, true, true);
    assert(rear.Uses_Retract_Position_Control());
    assert(near(rear.Get_Position_Target_Angle(1), config.retract_target_rad[0]));
    update(now++, target - 5, deg(60), deg(40));
    assert(near(rear.Limit_Velocity(1, 2, kd), 2));
    update(now++, target - 5, left, right);
    update(now++, target - 5, left, right, true, false);
    assert(!rear.Is_Motor_Controllable(1) && !rear.Is_Motor_Controllable(2));
    update(now, target - 5, left, right);
    assert(near(rear.Get_Output_Scale(), 0));
    now += 1000;
    update(now++, target - 5, left - deg(0.5f), right + deg(0.5f));
    assert(near(rear.Limit_Velocity(1, 2, kd), 0.5f * speed));
    update(now++, target - 5, left, right, true, true, false, false);
    assert(!rear.Is_Motor_Controllable(1));

    // Release timing across tick rollover, with the opposite encoder boundary pair.
    rear = Rear(config);
    update(0xFFFFFFF0U, target - 5, left, right);
    const uint32_t start = 0xFFFFFFF1U;
    update(start, target + reverse, left, right);
    update(start + Rear::EXTENSION_RELEASE_TIME_MS - 1U, target + reverse, left, right);
    assert(near(rear.Limit_Velocity(1, -1, kd), 0));
    update(start + Rear::EXTENSION_RELEASE_TIME_MS, target + reverse, left, right);
    assert(near(rear.Limit_Velocity(1, -1, kd), -1));
    // A fresh extended-end hold is still available after leaving the shortened end.
    update(start + Rear::EXTENSION_RELEASE_TIME_MS + 1U, target + 5,
           extended_left, extended_right);
    assert(near(rear.Limit_Velocity(1, -2, kd), 0));
    assert(near(rear.Limit_Velocity(2, 2, kd), 0));
    std::puts("PASS: shortened-end hold, mirrored release, mixed ends, safety and recovery");
}
