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
    // Only a real feedback fault starts the recovery ramp.
    rear.Update(true, false, true, true, 0, 0, deg(60), deg(40), 0U);
    sample(rear, 0, deg(60), deg(40), 0.2f, -0.2f);
    assert(near(rear.Limit_Velocity(1, 1, 1.3f), 0.2f));
    sample(rear, 1000, deg(60), deg(40), 0.2f, -0.2f);
    assert(near(rear.Limit_Velocity(1, 1, 1.3f), 0.6f));
    sample(rear, 2000, deg(60), deg(40));
    // Requests beyond the +/-10 rad/s mapping saturate in both directions.
    const float mapping_speed = 10.0f;
    for (uint8_t id : {1U, 2U}) {
        assert(near(rear.Limit_Velocity(id, 20, 2.5f), mapping_speed));
        assert(near(rear.Limit_Velocity(id, -20, 2.5f), -mapping_speed));
    }
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
    const uint32_t ramp_end = 2004U + Rear::RETRACT_SPEED_RAMP_TIME_MS;
    sample(rear, ramp_end, config.angle_end_rad[0], config.angle_start_rad[1], 0, 0, true, true);
    assert(near(rear.Limit_Velocity(1, 1, 2.5f), 0));
    assert(near(rear.Limit_Velocity(2, -1, 2.5f), 0));
    assert(near(rear.Limit_Velocity(1, -1, 2.5f), -1));
    assert(near(rear.Limit_Velocity(2, 1, 2.5f), 1));
    sample(rear, ramp_end + 1U, deg(60), deg(40), -mapping_speed, mapping_speed, true, true);
    for (uint8_t id : {1U, 2U}) {
        const float actual = id == 1 ? -mapping_speed : mapping_speed;
        for (float kd : {1.3f, 2.5f}) {
            const float target = rear.Limit_Velocity(id, -actual, kd);
            assert(std::fabs(target) <= mapping_speed);
            assert(std::fabs(kd * (target - actual)) <= 30.00001f);
        }
    }
    assert(rear.Limit_Velocity(1, NAN, 1.3f) == 0);
    assert(rear.Limit_Velocity(1, 1, 0) == 0);
    assert(rear.Limit_Velocity(1, 1, 6) == 0);
    assert(!rear.Is_Angle_Valid(1, config.angle_end_rad[0] + deg(0.6f)));
    sample(rear, ramp_end + 2U, deg(60), deg(40), 0, 0, false);
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

    // Nonzero feedforward is sent during retract motion and hold, once per command.
    auto assisted_config = config;
    assisted_config.retract_feedforward_torque_nm[0] = 4.0f;
    assisted_config.retract_feedforward_torque_nm[1] = -6.0f;
    Rear assisted(assisted_config);
    assert(assisted.Get_Retract_Feedforward_Torque(1) == 0);
    sample(assisted, 0, deg(60), deg(40));
    assert(assisted.Get_Retract_Feedforward_Torque(1) == 0);
    sample(assisted, 1, deg(60), deg(40), 0, 0, true, true);
    assert(near(assisted.Get_Retract_Feedforward_Torque(1), 4));
    assert(near(assisted.Get_Retract_Feedforward_Torque(2), -6));
    const uint32_t assisted_ramp_end = 1U + Rear::RETRACT_SPEED_RAMP_TIME_MS;
    sample(assisted, assisted_ramp_end, deg(60), deg(40), 0, 0, true, true);
    assert(near(assisted.Limit_Velocity(1, 1, 2.5f), 1));
    assert(near(assisted.Limit_Velocity(2, -1, 2.5f), -1));
    assert(assisted.Get_Retract_Feedforward_Torque(0) == 0);
    assert(assisted.Get_Retract_Feedforward_Torque(3) == 0);
    sample(assisted, assisted_ramp_end + 1U, config.retract_target_rad[0], config.retract_target_rad[1],
           0, 0, true, true);
    assert(assisted.Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD);
    assert(near(assisted.Get_Retract_Feedforward_Torque(1), 4));
    assert(near(assisted.Get_Retract_Feedforward_Torque(2), -6));
    sample(assisted, assisted_ramp_end + 2U, deg(60), deg(40));
    assert(assisted.Get_Retract_Feedforward_Torque(1) == 0);
    assert(assisted.Get_Retract_Feedforward_Torque(2) == 0);
    sample(assisted, assisted_ramp_end + 2U, deg(60), deg(40), 0, 0, false);
    assert(assisted.Get_Retract_Feedforward_Torque(1) == 0);
    sample(assisted, assisted_ramp_end + 3U, deg(60), deg(40), 0, 0, true, true);
    assert(assisted.Get_Retract_Feedforward_Torque(1) == 0);
    sample(assisted, assisted_ramp_end + 1003U, deg(60), deg(40), 0, 0, true, true);
    assert(near(assisted.Get_Retract_Feedforward_Torque(1), 2));
    assert(near(assisted.Get_Retract_Feedforward_Torque(2), -3));
    const float ramp_velocity = assisted.Limit_Velocity(1, 1, 2.5f);
    const float speed_scale = fminf(1.0f, 1000.0f / Rear::RETRACT_SPEED_RAMP_TIME_MS);
    assert(near(2.5f * ramp_velocity + assisted.Get_Retract_Feedforward_Torque(1),
                0.5f * (2.5f * speed_scale + 4.0f)));

    // Check the actual sum of the MIT speed term and sent feedforward, including
    // cancellation, opposing feedforward signs, mapping saturation and endpoints.
    for (float ff : {-50.0f, -4.0f, 4.0f, 50.0f}) {
        auto grid_config = config;
        grid_config.retract_feedforward_torque_nm[0] = ff;
        grid_config.retract_feedforward_torque_nm[1] = -ff;
        Rear grid(grid_config);
        sample(grid, 0, deg(60), deg(40), 0, 0, true, true);
        for (int endpoint : {-1, 0, 1}) {
            const float left = endpoint < 0 ? config.angle_start_rad[0] :
                endpoint > 0 ? config.angle_end_rad[0] : deg(60);
            const float right = endpoint < 0 ? config.angle_start_rad[1] :
                endpoint > 0 ? config.angle_end_rad[1] : deg(40);
            for (float actual : {-mapping_speed, -0.8f, 0.0f, 0.8f, mapping_speed}) {
                sample(grid, Rear::RETRACT_SPEED_RAMP_TIME_MS, left, right, actual, actual, true, true);
                for (uint8_t id : {1U, 2U}) {
                    for (float kd : {1.3f, 2.5f, 5.0f}) {
                        for (float requested : {-20.0f, -1.0f, 0.0f, 1.0f, 20.0f}) {
                            const float target = grid.Limit_Velocity(id, requested, kd);
                            const float total = kd * (target - actual) + grid.Get_Retract_Feedforward_Torque(id);
                            assert(std::fabs(target) <= mapping_speed);
                            assert(std::fabs(total) <= 30.00002f);
                            if (endpoint < 0) assert(total >= -0.00002f);
                            if (endpoint > 0) assert(total <= 0.00002f);
                        }
                    }
                }
            }
        }
    }
}
