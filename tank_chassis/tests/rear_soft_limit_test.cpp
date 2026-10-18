#include "fsm/up_stair_behind_motor_fsm.hpp"
#include <cassert>
#include <cmath>
#include <initializer_list>

using Rear = Class_Up_Stair_Behind_Motor_FSM;
float deg(float x) { return x * 3.14159265358979323846f / 180.0f; }
bool near(float a, float b) { return std::fabs(a - b) < 1e-5f; }
void sample(Rear &rear, uint32_t now, float left, float right,
            float lv = 0, float rv = 0, bool imu = true, bool retract = false)
{
    rear.Update(true, imu, true, true, 2, 0, left, right, now, retract, lv, rv);
}

int main()
{
    const auto config = Rear::Config::WithAngleConstants();
    Rear rear(config);
    sample(rear, 0, deg(60), deg(40), 0.2f, -0.2f);
    assert(near(rear.Limit_Velocity(1, 1, 1.3f), 0.2f));
    sample(rear, 1000, deg(60), deg(40), 0.2f, -0.2f);
    assert(near(rear.Limit_Velocity(1, 1, 1.3f), 0.6f));
    sample(rear, 2000, deg(60), deg(40));
    // No slowdown or reverse braking before the endpoint tolerance band.
    for (float distance : {4.0f, 1.0f, 0.6f}) {
        sample(rear, 2001, config.angle_end_rad[0] - deg(distance),
               config.angle_start_rad[1] + deg(distance), 0.8f, -0.8f);
        assert(near(rear.Limit_Torque(1, 20), 20));
        assert(near(rear.Limit_Torque(2, -20), -20));
        assert(near(rear.Limit_Velocity(1, 2, 1.3f), 2));
        assert(near(rear.Limit_Velocity(2, -2, 1.3f), -2));
    }
    // Each motor at either endpoint: block outward torque, pass inward torque.
    // Include the retained inner tolerance band and valid outer tolerance.
    for (bool upper : {false, true}) {
        for (float offset : {-0.25f, 0.0f, 0.25f}) {
            const float direction = upper ? 1.0f : -1.0f;
            const float left = upper ? config.angle_end_rad[0] : config.angle_start_rad[0];
            const float right = upper ? config.angle_end_rad[1] : config.angle_start_rad[1];
            for (float speed : {-0.8f, 0.0f, 0.8f}) {
                sample(rear, 2002, left + deg(offset), right + deg(offset), speed, speed);
                for (uint8_t id : {1U, 2U}) {
                    assert(rear.Is_Motor_Controllable(id));
                    assert(near(rear.Limit_Torque(id, direction * 20), 0));
                    assert(near(rear.Limit_Torque(id, -direction * 20), -direction * 20));
                    for (float kd : {1.3f, 2.5f}) {
                        // Use torque sign, not target-speed sign: actual speed may be nonzero.
                        assert(near(rear.Limit_Velocity(id, speed + direction, kd), speed));
                        assert(near(rear.Limit_Velocity(id, speed - direction, kd), speed - direction));
                        assert(near(rear.Limit_Velocity(id, speed, kd), speed));
                    }
                }
            }
        }
    }
    // Leaving an endpoint restores outward permission immediately, without dwell/backoff.
    sample(rear, 2003, config.angle_end_rad[0] - deg(0.6f),
           config.angle_start_rad[1] + deg(0.6f));
    assert(near(rear.Limit_Torque(1, 20), 20));
    assert(near(rear.Limit_Torque(2, -20), -20));
    // Encoder retraction uses the same directional endpoint protection.
    sample(rear, 2004, config.angle_end_rad[0], config.angle_start_rad[1], 0, 0, true, true);
    assert(rear.Uses_Position_Control());
    assert(near(rear.Limit_Velocity(1, 1, 2.5f), 0));
    assert(near(rear.Limit_Velocity(2, -1, 2.5f), 0));
    assert(near(rear.Limit_Velocity(1, -1, 2.5f), -1));
    assert(near(rear.Limit_Velocity(2, 1, 2.5f), 1));
    sample(rear, 2005, deg(60), deg(40), -20, 20, true, true);
    for (uint8_t id : {1U, 2U}) {
        const float actual = id == 1 ? -20 : 20;
        for (float kd : {1.3f, 2.5f}) {
            const float target = rear.Limit_Velocity(id, -actual, kd);
            assert(std::fabs(target) <= 20);
            assert(std::fabs(kd * (target - actual)) <= 30.00001f);
        }
    }
    assert(rear.Limit_Velocity(1, NAN, 1.3f) == 0);
    assert(rear.Limit_Velocity(1, 1, 0) == 0);
    assert(rear.Limit_Velocity(1, 1, 6) == 0);
    assert(!rear.Is_Angle_Valid(1, config.angle_end_rad[0] + deg(0.6f)));
    sample(rear, 2006, deg(60), deg(40), 0, 0, false);
    assert(rear.Limit_Velocity(1, 1, 1.3f) == 0);
    // Wrapped encoder intervals retain the same inward/outward convention.
    auto wrapped = config;
    wrapped.angle_start_rad[1] = deg(150);
    wrapped.angle_end_rad[1] = deg(-150);
    wrapped.retract_target_rad[1] = deg(170);
    Rear seam(wrapped);
    sample(seam, 0, deg(60), deg(170));
    sample(seam, 2000, deg(60), deg(-150));
    assert(near(seam.Limit_Torque(2, 20), 0));
    assert(near(seam.Limit_Torque(2, -20), -20));
    sample(seam, 2001, deg(60), deg(150));
    assert(near(seam.Limit_Torque(2, -20), 0));
    assert(near(seam.Limit_Torque(2, 20), 20));
}
