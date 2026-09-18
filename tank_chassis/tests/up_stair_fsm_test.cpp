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

static void update(Class_Up_Stair_FSM &fsm,
                   float left,
                   float right,
                   bool left_valid,
                   bool right_valid,
                   bool mechanism_enabled,
                   bool stair_command_enabled,
                   uint32_t action_sequence)
{
    fsm.Update(left,
               right,
               left_valid,
               right_valid,
               mechanism_enabled,
               stair_command_enabled,
               action_sequence);
}

int main()
{
    Class_Up_Stair_FSM fsm;
    fsm.Init(0U);

    // Active with no stair command starts in an enabled home hold.
    update(fsm,
           deg_to_rad(34.0f),
           deg_to_rad(340.0f),
           true,
           true,
           true,
           false,
           0U);
    assert(fsm.Get_State() == UP_STAIR_HOME_HOLD);
    assert(fsm.Is_Enabled());
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

    // B starts a move to target and preserves wrapped-angle mapping.
    update(fsm,
           deg_to_rad(34.0f),
           deg_to_rad(340.0f),
           true,
           true,
           true,
           true,
           1U);
    assert(fsm.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    assert(near(fsm.Get_Target_Angle(1U), Class_Up_Stair_FSM::TARGET_ANGLE_RAD[0]));
    assert(near(fsm.Get_Target_Angle(2U), deg_to_rad(420.0f)));
    assert(near(fsm.Get_Position_Feedback(2U), deg_to_rad(340.0f)));
    assert(near(fsm.Get_Position_Error(2U), deg_to_rad(80.0f)));

    // Arrival requires both feedbacks and uses a two-degree tolerance.
    update(fsm,
           deg_to_rad(120.5f),
           deg_to_rad(61.5f),
           true,
           true,
           true,
           true,
           1U);
    assert(fsm.Get_State() == UP_STAIR_TARGET_HOLD);

    // A second B while moving is allowed and reverses the command.
    update(fsm,
           deg_to_rad(80.0f),
           deg_to_rad(320.0f),
           true,
           true,
           true,
           true,
           2U);
    assert(fsm.Get_State() == UP_STAIR_RETURNING_HOME);
    assert(near(fsm.Get_Target_Angle(1U), Class_Up_Stair_FSM::HOME_ANGLE_RAD[0]));
    assert(near(fsm.Get_Target_Angle(2U), Class_Up_Stair_FSM::HOME_ANGLE_RAD[1]));

    update(fsm,
           deg_to_rad(34.0f),
           deg_to_rad(340.0f),
           true,
           true,
           true,
           true,
           2U);
    assert(fsm.Get_State() == UP_STAIR_HOME_HOLD);

    // Leaving command mode returns home and never disables the controller.
    update(fsm,
           deg_to_rad(122.0f),
           deg_to_rad(60.0f),
           true,
           true,
           true,
           false,
           2U);
    assert(fsm.Get_State() == UP_STAIR_RETURNING_HOME);
    assert(fsm.Is_Enabled());
    update(fsm,
           deg_to_rad(34.0f),
           deg_to_rad(340.0f),
           true,
           true,
           true,
           false,
           2U);
    assert(fsm.Get_State() == UP_STAIR_HOME_HOLD);
    assert(fsm.Is_Enabled());

    // Mechanism disable (double-down-like) disables and synchronizes B.
    update(fsm,
           deg_to_rad(34.0f),
           deg_to_rad(340.0f),
           true,
           true,
           false,
           false,
           10U);
    assert(fsm.Get_State() == UP_STAIR_DISABLED);
    assert(!fsm.Is_Enabled());
    assert(near(fsm.Get_Target_Angle(1U), Class_Up_Stair_FSM::HOME_ANGLE_RAD[0]));

    // Re-enable away from home: return first, and ignore the stale B sequence.
    update(fsm,
           deg_to_rad(122.0f),
           deg_to_rad(60.0f),
           true,
           true,
           true,
           true,
           10U);
    assert(fsm.Get_State() == UP_STAIR_RETURNING_HOME);
    assert(near(fsm.Get_Target_Angle(1U), Class_Up_Stair_FSM::HOME_ANGLE_RAD[0]));
    update(fsm,
           deg_to_rad(34.0f),
           deg_to_rad(340.0f),
           true,
           true,
           true,
           true,
           10U);
    assert(fsm.Get_State() == UP_STAIR_HOME_HOLD);

    // One motor offline keeps control enabled but cannot declare arrival.
    update(fsm,
           deg_to_rad(34.0f),
           deg_to_rad(340.0f),
           true,
           true,
           true,
           true,
           11U);
    assert(fsm.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    update(fsm,
           deg_to_rad(122.0f),
           deg_to_rad(60.0f),
           false,
           true,
           true,
           true,
           11U);
    assert(fsm.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    assert(fsm.Is_Enabled());

    // Both feedback channels offline disable the shared controller.
    update(fsm,
           0.0f,
           0.0f,
           false,
           false,
           true,
           true,
           11U);
    assert(fsm.Get_State() == UP_STAIR_DISABLED);
    assert(!fsm.Is_Enabled());
    return 0;
}
