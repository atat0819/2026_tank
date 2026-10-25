#include "../fsm/up_stair_fsm.hpp"
#include <cassert>
#include <cmath>
#include <initializer_list>

static float rad(float degree) { return degree * 3.14159265359f / 180.0f; }
static bool near(float a, float b) { return std::fabs(a-b) < 1e-5f; }

int main()
{
    Class_Up_Stair_FSM front;
    front.Init();
    auto update = [&](float left, float right, bool target,
                      bool left_valid=true, bool right_valid=true,
                      bool enabled=true, bool command_enabled=true) {
        front.Update(rad(left), rad(right), left_valid, right_valid,
                     enabled, command_enabled, target, 0U);
    };
    update(13.32f, -19.04f, false);
    assert(front.Get_State() == UP_STAIR_HOME_HOLD);
    assert(front.Is_Enabled());
    assert(near(front.Get_Current_Angle(2), rad(-19.04f)));
    assert(near(front.Get_Position_Feedback(2), rad(-19.04f)));
    assert(near(front.Get_Target_Angle(1), rad(13.32f)));
    assert(near(front.Get_Target_Angle(2), rad(-19.04f)));
    update(13.32f, -19.04f, true);
    assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    assert(near(front.Get_Target_Angle(1), rad(90.94f)));
    assert(near(front.Get_Target_Angle(2), rad(-98.65f)));
    assert(near(front.Get_Position_Error(2), rad(-79.61f)));
    assert(near(front.Get_Position_Error(1), rad(77.62f)));
    // Both calibrated paths stay inside ordinary intervals without unfolding.
    update(50, -60, true);
    assert(near(front.Get_Position_Feedback(1), rad(50)));
    assert(near(front.Get_Position_Feedback(2), rad(-60)));
    assert(near(front.Get_Position_Error(1), rad(40.94f)));
    assert(near(front.Get_Position_Error(2), rad(-38.65f)));
    const float left_before = front.Get_Position_Error(1);
    const float right_before = front.Get_Position_Error(2);
    update(50.4f, -60.4f, true);
    assert(near(front.Get_Position_Error(1)-left_before, rad(-0.4f)));
    assert(near(front.Get_Position_Error(2)-right_before, rad(0.4f)));
    update(50, -60, true); // Repeated request must not toggle the state.
    assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    update(50, -60, false); // B2 reverses before reaching target.
    assert(front.Get_State() == UP_STAIR_RETURNING_HOME);
    assert(near(front.Get_Target_Angle(1), rad(13.32f)));
    assert(near(front.Get_Target_Angle(2), rad(-19.04f)));
    update(40, -50, true); // B3 reverses while returning home.
    assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    update(89.44f, -97.15f, true);
    assert(front.Get_State() == UP_STAIR_TARGET_HOLD);
    update(90.94f, -98.65f, false);
    assert(front.Get_State() == UP_STAIR_RETURNING_HOME);
    update(13.32f, -60, false);
    assert(front.Get_State() == UP_STAIR_RETURNING_HOME);
    update(13.32f, -19.04f, false);
    assert(front.Get_State() == UP_STAIR_HOME_HOLD);

    update(13.32f, -19.04f, true);
    update(50, -60, true, true, true, true, false);
    assert(front.Get_State() == UP_STAIR_RETURNING_HOME);
    update(13.32f, -19.04f, false, true, true, true, false);
    assert(front.Get_State() == UP_STAIR_HOME_HOLD);
    update(13.32f, -19.04f, false, true, true, false);
    assert(!front.Is_Enabled());

    // Each feedback-loss combination disables the pair; recovery homes first.
    for (unsigned mask=0; mask<3; ++mask)
    {
        update(13.32f, -19.04f, false);
        update(13.32f, -19.04f, true);
        update(50, -60, true, (mask&1)!=0, (mask&2)!=0);
        assert(front.Get_State() == UP_STAIR_DISABLED);
        update(50, -60, true);
        assert(front.Get_State() == UP_STAIR_RETURNING_HOME);
        update(13.32f, -19.04f, true);
        assert(front.Get_State() == UP_STAIR_HOME_HOLD);
        update(13.32f, -19.04f, true); // Stale target request cannot replay after recovery.
        assert(front.Get_State() == UP_STAIR_HOME_HOLD);
        update(13.32f, -19.04f, false);
        assert(front.Get_State() == UP_STAIR_HOME_HOLD);
        update(13.32f, -19.04f, true);
        assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    }
    for (float d : {-1.68f, 105.94f, 13.32f, 90.94f}) assert(front.Is_Angle_Valid(1,rad(d)));
    for (float d : {-1.69f, 105.95f, 180.0f, -180.0f, NAN}) assert(!front.Is_Angle_Valid(1,rad(d)));
    for (float d : {-113.65f, -4.04f, -19.04f, -98.65f}) assert(front.Is_Angle_Valid(2,rad(d)));
    for (float d : {-113.66f, -4.03f, 180.0f, -180.0f, 0.0f, NAN}) assert(!front.Is_Angle_Valid(2,rad(d)));
    for (float outside : {-1.69f, 105.95f})
    {
        update(outside, -19.04f, true);
        assert(!front.Is_Enabled());
        update(13.32f, -19.04f, true);
        assert(front.Get_State() == UP_STAIR_HOME_HOLD);
        assert(near(front.Get_Target_Angle(1), rad(13.32f)));
    }
    for (float outside : {-113.66f, -4.03f})
    {
        update(13.32f, outside, true);
        assert(!front.Is_Enabled());
        update(13.32f, -19.04f, true);
        assert(front.Get_State() == UP_STAIR_HOME_HOLD);
        assert(near(front.Get_Target_Angle(2), rad(-19.04f)));
    }
}
