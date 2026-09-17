#include "../fsm/up_stair_fsm.hpp"
#include <assert.h>
#include <math.h>

static bool near(float actual, float expected)
{
    return fabsf(actual - expected) < 1.0e-5f;
}

int main()
{
    Class_Up_Stair_FSM fsm;
    fsm.Init(0U);

    fsm.Update(1.0f, 1.0f, true, false, 0U);
    assert(fsm.Get_State() == UP_STAIR_DISABLED);
    assert(!fsm.Is_Enabled());

    fsm.Update(2.0f, 2.2f, true, true, 0U);
    assert(fsm.Get_State() == UP_STAIR_HOME);
    assert(fsm.Is_Enabled());
    assert(near(fsm.Get_Target_Angle(1), Class_Up_Stair_FSM::HOME_ANGLE_RAD[0]));
    assert(near(fsm.Get_Target_Angle(2), Class_Up_Stair_FSM::HOME_ANGLE_RAD[1]));

    fsm.Update(2.0f, 2.2f, true, true, 1U);
    assert(fsm.Get_State() == UP_STAIR_TARGET);
    assert(near(fsm.Get_Target_Angle(1), Class_Up_Stair_FSM::TARGET_ANGLE_RAD[0]));
    assert(near(fsm.Get_Target_Angle(2), Class_Up_Stair_FSM::TARGET_ANGLE_RAD[1]));

    const float motor_1_near_max = Class_Up_Stair_FSM::LIMIT_MAX_RAD[0] - 0.01f;
    assert(fsm.Is_Angle_Valid(1U, motor_1_near_max));
    assert(!fsm.Is_Angle_Valid(1U, Class_Up_Stair_FSM::LIMIT_MIN_RAD[0] - 0.01f));
    assert(!fsm.Is_Angle_Valid(2U, Class_Up_Stair_FSM::LIMIT_MAX_RAD[1] + 0.01f));

    fsm.Update(Class_Up_Stair_FSM::LIMIT_MIN_RAD[0] - 0.01f,
               Class_Up_Stair_FSM::HOME_ANGLE_RAD[1],
               true,
               true,
               1U);
    assert(fsm.Get_State() == UP_STAIR_DISABLED);
    assert(!fsm.Is_Enabled());
    return 0;
}
