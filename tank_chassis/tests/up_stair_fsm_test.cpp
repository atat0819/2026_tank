#include "../fsm/up_stair_fsm.hpp"
#include <assert.h>
#include <math.h>

static float deg_to_rad(float degree)
{
    return degree * 3.14159265359f / 180.0f;
}

static bool near(float actual, float expected)
{
    return fabsf(actual - expected) < 1.0e-5f;
}

int main()
{
    Class_Up_Stair_FSM fsm;
    fsm.Init(0U);

    fsm.Update(deg_to_rad(34.0f), deg_to_rad(340.0f), true, true, true, 0U);
    assert(fsm.Get_State() == UP_STAIR_HOME);
    assert(near(fsm.Get_Target_Angle(1U), Class_Up_Stair_FSM::HOME_ANGLE_RAD[0]));
    assert(near(fsm.Get_Target_Angle(2U), Class_Up_Stair_FSM::HOME_ANGLE_RAD[1]));

    assert(fsm.Is_Angle_Valid(1U, deg_to_rad(10.0f)));
    assert(fsm.Is_Angle_Valid(1U, deg_to_rad(178.0f)));
    assert(!fsm.Is_Angle_Valid(1U, deg_to_rad(9.0f)));
    assert(!fsm.Is_Angle_Valid(1U, deg_to_rad(179.0f)));

    assert(fsm.Is_Angle_Valid(2U, deg_to_rad(300.0f)));
    assert(fsm.Is_Angle_Valid(2U, deg_to_rad(350.0f)));
    assert(fsm.Is_Angle_Valid(2U, deg_to_rad(0.0f)));
    assert(fsm.Is_Angle_Valid(2U, deg_to_rad(70.0f)));
    assert(!fsm.Is_Angle_Valid(2U, deg_to_rad(71.0f)));
    assert(!fsm.Is_Angle_Valid(2U, deg_to_rad(299.0f)));

    fsm.Update(deg_to_rad(34.0f), deg_to_rad(340.0f), true, true, true, 1U);
    assert(fsm.Get_State() == UP_STAIR_TARGET);
    assert(near(fsm.Get_Target_Angle(1U), Class_Up_Stair_FSM::TARGET_ANGLE_RAD[0]));
    assert(near(fsm.Get_Target_Angle(2U), deg_to_rad(420.0f)));
    assert(near(fsm.Get_Position_Feedback(2U), deg_to_rad(340.0f)));
    assert(near(fsm.Get_Position_Error(2U), deg_to_rad(80.0f)));

    // A second debounced B press increments the action sequence again and
    // must toggle the command back to the mechanical home position.
    fsm.Update(deg_to_rad(122.0f), deg_to_rad(60.0f), true, true, true, 2U);
    assert(fsm.Get_State() == UP_STAIR_HOME);
    assert(near(fsm.Get_Target_Angle(1U), Class_Up_Stair_FSM::HOME_ANGLE_RAD[0]));
    assert(near(fsm.Get_Target_Angle(2U), Class_Up_Stair_FSM::HOME_ANGLE_RAD[1]));

    // Motor 1 can be offline while motor 2 remains controllable.
    fsm.Update(0.0f, deg_to_rad(340.0f), false, true, true, 2U);
    assert(fsm.Get_State() == UP_STAIR_HOME);
    assert(fsm.Is_Enabled());

    // Both feedback channels offline still disable the shared controller.
    fsm.Update(0.0f, 0.0f, false, false, true, 2U);
    assert(fsm.Get_State() == UP_STAIR_DISABLED);
    assert(!fsm.Is_Enabled());
    return 0;
}
