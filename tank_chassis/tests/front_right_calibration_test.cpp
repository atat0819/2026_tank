#include "../fsm/up_stair_fsm.hpp"
#include <cassert>
#include <cmath>

static float rad(float degree) { return degree * 3.14159265359f / 180.0f; }
static bool near(float a, float b) { return std::fabs(a - b) < 1e-5f; }

int main()
{
    Class_Up_Stair_FSM front;
    front.Init();
    const float left = Class_Up_Stair_FSM::HOME_ANGLE_RAD[0];
    assert(near(Class_Up_Stair_FSM::HOME_ANGLE_RAD[1], rad(14)));
    assert(near(Class_Up_Stair_FSM::TARGET_ANGLE_RAD[1], rad(-86)));
    assert(near(Class_Up_Stair_FSM::LIMIT_START_RAD[1], rad(-101)));
    assert(near(Class_Up_Stair_FSM::LIMIT_END_RAD[1], rad(29)));
    front.Update(left, rad(14), true, true, true, true, false, 0);
    assert(front.Get_State() == UP_STAIR_HOME_HOLD);
    front.Update(left, rad(14), true, true, true, true, true, 1);
    assert(near(front.Get_Position_Error(2), rad(-100)));
    front.Update(left, 0, true, true, true, true, true, 2);
    assert(front.Is_Enabled());
    assert(near(front.Get_Position_Feedback(2), 0));
    assert(near(front.Get_Position_Error(2), rad(-86)));
    front.Update(left, rad(-86), true, true, true, true, true, 3);
    assert(near(front.Get_Position_Error(2), 0));
    front.Update(left, rad(-86), true, true, true, true, false, 4);
    assert(near(front.Get_Position_Error(2), rad(100)));
    assert(!front.Is_Angle_Valid(2, rad(-101.1f)));
    assert(!front.Is_Angle_Valid(2, rad(29.1f)));
}
