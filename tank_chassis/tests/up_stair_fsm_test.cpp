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
                     enabled, command_enabled, target);
    };
    update(139, -109, false);
    assert(front.Get_State() == UP_STAIR_HOME_HOLD);
    assert(front.Is_Enabled());
    assert(near(front.Get_Current_Angle(2), rad(-109)));
    assert(near(front.Get_Position_Feedback(2), rad(251)));
    assert(near(front.Get_Target_Angle(2), rad(251)));
    update(139, -109, true);
    assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    assert(near(front.Get_Target_Angle(1), rad(200)));
    assert(near(front.Get_Target_Angle(2), rad(172)));
    assert(near(front.Get_Position_Error(2), rad(-79)));
    assert(near(front.Get_Position_Error(1), rad(61)));
    // The raw target -160 degrees is unfolded to +200 degrees. Crossing
    // +180/-180 must not produce a full-turn position-error jump.
    update(179.8f, -150, true);
    assert(near(front.Get_Position_Feedback(1), rad(179.8f)));
    const float before_seam = front.Get_Position_Error(1);
    update(-179.8f, -150, true);
    assert(near(front.Get_Current_Angle(1), rad(-179.8f)));
    assert(near(front.Get_Position_Feedback(1), rad(180.2f)));
    assert(near(front.Get_Position_Error(1), rad(19.8f)));
    assert(near(front.Get_Position_Error(1)-before_seam, rad(-0.4f)));
    update(-179.8f, -150, false);
    assert(near(front.Get_Position_Error(1), rad(-41.2f)));
    update(179.8f, -150, false);
    assert(near(front.Get_Position_Error(1), rad(-40.8f)));
    update(-179.8f, -150, true);
    assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    // Right front travels in the negative direction: 251 -> 172 degrees.
    // Raw -179.8 -> +179.8 unfolds to 180.2 -> 179.8, only -0.4 degrees.
    update(170, -179.8f, true);
    assert(near(front.Get_Current_Angle(2), rad(-179.8f)));
    assert(near(front.Get_Position_Feedback(2), rad(180.2f)));
    assert(near(front.Get_Position_Error(2), rad(-8.2f)));
    const float right_before_seam = front.Get_Position_Error(2);
    update(170, 179.8f, true);
    assert(near(front.Get_Position_Feedback(2), rad(179.8f)));
    assert(near(front.Get_Position_Error(2), rad(-7.8f)));
    assert(near(front.Get_Position_Error(2)-right_before_seam, rad(0.4f)));
    update(170, 179.8f, false);
    assert(near(front.Get_Target_Angle(2), rad(251)));
    assert(near(front.Get_Position_Error(2), rad(71.2f)));
    update(170, -179.8f, false);
    assert(near(front.Get_Position_Error(2), rad(70.8f)));
    update(170, -150, true);
    update(170, -150, true); // Repeated request must not toggle the state.
    assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    update(170, -150, false); // B2 reverses before reaching target.
    assert(front.Get_State() == UP_STAIR_RETURNING_HOME);
    assert(near(front.Get_Target_Angle(1), rad(139)));
    update(160, -140, true); // B3 reverses while returning home.
    assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    update(-161.5f, 173.5f, true);
    assert(front.Get_State() == UP_STAIR_TARGET_HOLD);
    update(-160, 172, false);
    assert(front.Get_State() == UP_STAIR_RETURNING_HOME);
    update(139, -150, false);
    assert(front.Get_State() == UP_STAIR_RETURNING_HOME);
    update(139, -109, false);
    assert(front.Get_State() == UP_STAIR_HOME_HOLD);

    update(139, -109, true);
    update(170, -150, true, true, true, true, false);
    assert(front.Get_State() == UP_STAIR_RETURNING_HOME);
    update(139, -109, false, true, true, true, false);
    assert(front.Get_State() == UP_STAIR_HOME_HOLD);
    update(139, -109, false, true, true, false);
    assert(!front.Is_Enabled());

    // Each feedback-loss combination disables the pair; recovery homes first.
    for (unsigned mask=0; mask<3; ++mask)
    {
        update(139, -109, false);
        update(139, -109, true);
        update(170, -150, true, (mask&1)!=0, (mask&2)!=0);
        assert(front.Get_State() == UP_STAIR_DISABLED);
        update(170, -150, true);
        assert(front.Get_State() == UP_STAIR_RETURNING_HOME);
        update(139, -109, true);
        assert(front.Get_State() == UP_STAIR_HOME_HOLD);
        update(139, -109, true); // Stale target request cannot replay after recovery.
        assert(front.Get_State() == UP_STAIR_HOME_HOLD);
        update(139, -109, false);
        assert(front.Get_State() == UP_STAIR_HOME_HOLD);
        update(139, -109, true);
        assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    }
    for (float d : {124.0f, -145.0f, 180.0f, -180.0f, 139.0f, -160.0f}) assert(front.Is_Angle_Valid(1,rad(d)));
    for (float d : {123.99f, -144.99f, 0.0f, NAN}) assert(!front.Is_Angle_Valid(1,rad(d)));
    for (float d : {157.0f, -94.0f, 180.0f, -180.0f, -109.0f, 172.0f}) assert(front.Is_Angle_Valid(2,rad(d)));
    for (float d : {156.99f, -93.99f, 0.0f, NAN}) assert(!front.Is_Angle_Valid(2,rad(d)));
    for (float outside : {123.99f, -144.99f})
    {
        update(outside, -109, true);
        assert(!front.Is_Enabled());
        update(139, -109, true);
        assert(front.Get_State() == UP_STAIR_HOME_HOLD);
        assert(near(front.Get_Target_Angle(1), rad(139)));
    }
    for (float outside : {156.99f, -93.99f})
    {
        update(139, outside, true);
        assert(!front.Is_Enabled());
        update(139, -109, true);
        assert(front.Get_State() == UP_STAIR_HOME_HOLD);
        assert(near(front.Get_Target_Angle(2), rad(251)));
    }
}
