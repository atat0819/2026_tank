#include "fsm/up_stair_behind_motor_fsm.hpp"
#include <cassert>
#include <cmath>

int main()
{
    using Rear = Class_Up_Stair_Behind_Motor_FSM;
    Rear rear(Rear::Config::WithAngleConstants());
    // B2 can request retraction before the 2 s attitude recovery has ended.
    rear.Update(true, true, true, true, 4.0f, 0.0f, 0.8f, 0.8f, 100U);
    assert(rear.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
    rear.Update(true, true, true, true, 4.0f, 0.0f, 0.8f, 0.8f, 101U, true);
    assert(rear.Uses_Retract_Position_Control());
    assert(rear.Get_Output_Scale() < 0.01f);
    rear.Update(true, true, true, true, 4.0f, 0.0f, 0.8f, 0.8f, 201U, true);
    assert(rear.Get_Position_Target_Angle(1U) > 0.8f);
    // Another B reverses immediately, without waiting for the retract target.
    rear.Update(true, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 202U, false);
    assert(rear.Uses_Attitude_Control());
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
    rear.Update(true, true, true, true, 4.0f, 0.0f, 1.0f, 1.0f, 1205U, true);
    assert(std::fabs(rear.Get_Output_Scale() - 0.5f) < 1e-5f);
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
}
