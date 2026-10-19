#include "fsm/up_stair_behind_motor_fsm.hpp"
#include <cassert>
#include <cmath>

int main()
{
    using Rear = Class_Up_Stair_Behind_Motor_FSM;
    for (unsigned phase = 0; phase < 4; ++phase)
    {
        for (unsigned fault = 0; fault < 3; ++fault)
        {
            const auto config = Rear::Config::WithAngleConstants();
            Rear rear(config);
            uint32_t now = 0U;
            rear.Update(true, true, true, true, 4, 1, 1.0f, 1.0f, now);
            if (phase >= 1)
            {
                now = 4000U;
                rear.Update(true, true, true, true, 4, 1, 1.0f, 1.0f, now);
                assert(rear.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
            }
            const bool retract = phase >= 2;
            if (retract)
            {
                ++now;
                rear.Update(true, true, true, true, 4, 1, 1.0f, 1.0f, now, true);
                assert(rear.Uses_Retract_Position_Control());
            }
            if (phase == 3)
            {
                now += 2000U;
                rear.Update(true, true, true, true, 4, 1,
                            config.retract_target_rad[0], config.retract_target_rad[1],
                            now, true);
                assert(rear.Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD);
            }
            // Invalid snapshot, nonfinite angle, and nonfinite rate all stop
            // both rear motors, even while retracting or holding position.
            rear.Update(true, fault != 0, true, true, fault == 1 ? NAN : 4,
                        fault == 2 ? INFINITY : 1, 0.9f, 0.4f, ++now, retract);
            assert(rear.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
            assert(!rear.Uses_Position_Control() && !rear.Uses_Attitude_Control());
            for (uint8_t id = 1; id <= 2; ++id)
            {
                assert(!rear.Is_Motor_Controllable(id));
                assert(rear.Limit_Torque(id, 10.0f) == 0.0f);
                assert(rear.Limit_Torque(id, -10.0f) == 0.0f);
            }
            // Valid IMU restores the selected request through zero-scale recovery.
            rear.Update(true, true, true, true, 4, 1, 0.9f, 0.4f, ++now, retract);
            assert(rear.Get_Output_Scale() == 0.0f);
            if (retract)
            {
                assert(rear.Uses_Retract_Position_Control());
                assert(std::fabs(rear.Get_Position_Target_Angle(1) - config.retract_target_rad[0]) < 1e-5f);
                assert(std::fabs(rear.Get_Position_Target_Angle(2) - config.retract_target_rad[1]) < 1e-5f);
            }
            else
            {
                assert(rear.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
            }
        }
    }
}
