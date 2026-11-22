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
    update(110.0f, 110.0f, false);
    assert(front.Get_State() == UP_STAIR_HOME_HOLD);
    assert(front.Is_Enabled());
    assert(near(front.Get_Current_Angle(2), rad(110.0f)));
    assert(near(front.Get_Position_Feedback(2), rad(470.0f)));
    assert(near(front.Get_Target_Angle(1), rad(110.0f)));
    assert(near(front.Get_Target_Angle(2), rad(470.0f)));
    update(110.0f, 110.0f, true);
    assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    assert(near(front.Get_Target_Angle(1), rad(224.0f)));
    assert(near(front.Get_Target_Angle(2), rad(142.35f)));
    assert(near(front.Get_Position_Error(2), rad(-327.65f)));
    assert(near(front.Get_Position_Error(1), rad(114.0f)));
    update(160, -140, true);
    assert(near(front.Get_Position_Feedback(1), rad(160)));
    assert(near(front.Get_Position_Feedback(2), rad(220)));
    assert(near(front.Get_Position_Error(1), rad(64.0f)));
    assert(near(front.Get_Position_Error(2), rad(-77.65f)));
    const float left_before = front.Get_Position_Error(1);
    const float right_before = front.Get_Position_Error(2);
    update(160.4f, -140.4f, true);
    assert(near(front.Get_Position_Error(1)-left_before, rad(-0.4f)));
    assert(near(front.Get_Position_Error(2)-right_before, rad(0.4f)));

    // Each encoder crosses the seam in its commanded direction.
    update(179.9f, -179.9f, true);
    const float left_seam_error = front.Get_Position_Error(1);
    const float right_seam_error = front.Get_Position_Error(2);
    assert(near(front.Get_Position_Feedback(1), rad(179.9f)));
    assert(near(front.Get_Position_Feedback(2), rad(180.1f)));
    assert(left_seam_error > 0.0f && right_seam_error < 0.0f);
    update(180.0f, -180.0f, true);
    assert(near(front.Get_Position_Feedback(1), rad(180.0f)));
    update(-180.0f, 180.0f, true);
    assert(near(front.Get_Position_Feedback(1), rad(180.0f)));
    update(-179.9f, 179.9f, true);
    assert(front.Is_Enabled());
    assert(near(front.Get_Current_Angle(1), rad(-179.9f)));
    assert(near(front.Get_Position_Feedback(1), rad(180.1f)));
    assert(near(front.Get_Position_Feedback(2), rad(179.9f)));
    assert(near(front.Get_Position_Error(1) - left_seam_error, rad(-0.2f)));
    assert(near(front.Get_Position_Error(2) - right_seam_error, rad(0.2f)));
    assert(front.Get_Position_Error(1) > 0.0f);
    assert(front.Get_Position_Error(2) < 0.0f);
    update(-179.9f, 179.9f, false);
    assert(front.Get_Position_Error(1) < 0.0f);
    assert(front.Get_Position_Error(2) > 0.0f); // HOME takes the reverse seam path.
    update(179.9f, -179.9f, false);
    assert(near(front.Get_Position_Error(1), rad(-69.9f)));
    assert(near(front.Get_Position_Error(2), rad(289.9f)));
    update(160, -140, true);
    update(160, -140, true); // Repeated request must not toggle the state.
    assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    update(160, -140, false); // B2 reverses before reaching target.
    assert(front.Get_State() == UP_STAIR_RETURNING_HOME);
    assert(near(front.Get_Target_Angle(1), rad(110.0f)));
    assert(near(front.Get_Target_Angle(2), rad(470.0f)));
    update(150, -130, true); // B3 reverses while returning home.
    assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    update(-138.5f, 142.35f, true);
    assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    update(-137.5f, 143.85f, true);
    assert(front.Get_State() == UP_STAIR_TARGET_HOLD);
    update(-136.0f, 142.35f, true);
    assert(near(front.Get_Position_Error(1), 0.0f));
    update(-136.0f, 142.35f, false);
    assert(front.Get_State() == UP_STAIR_RETURNING_HOME);
    update(110.0f, -140, false);
    assert(front.Get_State() == UP_STAIR_RETURNING_HOME);
    update(110.0f, 110.0f, false);
    assert(front.Get_State() == UP_STAIR_HOME_HOLD);

    update(110.0f, 110.0f, true);
    update(160, -140, true, true, true, true, false);
    assert(front.Get_State() == UP_STAIR_RETURNING_HOME);
    update(110.0f, 110.0f, false, true, true, true, false);
    assert(front.Get_State() == UP_STAIR_HOME_HOLD);
    update(110.0f, 110.0f, false, true, true, false);
    assert(!front.Is_Enabled());

    // Each feedback-loss combination disables the pair; recovery homes first.
    for (unsigned mask=0; mask<3; ++mask)
    {
        update(110.0f, 110.0f, false);
        update(110.0f, 110.0f, true);
        update(160, -140, true, (mask&1)!=0, (mask&2)!=0);
        assert(front.Get_State() == UP_STAIR_DISABLED);
        update(160, -140, true);
        assert(front.Get_State() == UP_STAIR_RETURNING_HOME);
        update(110.0f, 110.0f, true);
        assert(front.Get_State() == UP_STAIR_HOME_HOLD);
        update(110.0f, 110.0f, true); // Stale target request cannot replay after recovery.
        assert(front.Get_State() == UP_STAIR_HOME_HOLD);
        update(110.0f, 110.0f, false);
        assert(front.Get_State() == UP_STAIR_HOME_HOLD);
        update(110.0f, 110.0f, true);
        assert(front.Get_State() == UP_STAIR_MOVING_TO_TARGET);
    }
    assert(near(Class_Up_Stair_FSM::LIMIT_START_RAD[0], rad(95.0f)));
    assert(near(Class_Up_Stair_FSM::LIMIT_END_RAD[0], rad(-121.0f)));
    assert(near(Class_Up_Stair_FSM::LIMIT_START_RAD[1], rad(127.35f)));
    assert(near(Class_Up_Stair_FSM::LIMIT_END_RAD[1], rad(125.0f)));
    for (uint8_t id : {1U, 2U})
    {
        assert(front.Is_Angle_Valid(id, Class_Up_Stair_FSM::LIMIT_START_RAD[id-1]));
        assert(front.Is_Angle_Valid(id, Class_Up_Stair_FSM::LIMIT_END_RAD[id-1]));
    }
    for (float d : {110.0f, -136.0f, 180.0f, -180.0f}) assert(front.Is_Angle_Valid(1,rad(d)));
    for (float d : {94.99f, -120.99f, 0.0f, NAN}) assert(!front.Is_Angle_Valid(1,rad(d)));
    for (float d : {110.0f, 142.35f, -180.0f, 180.0f}) assert(front.Is_Angle_Valid(2,rad(d)));
    for (float d : {127.34f, 125.01f, NAN}) assert(!front.Is_Angle_Valid(2,rad(d)));
    for (float outside : {94.99f, -120.99f})
    {
        update(outside, 110.0f, true);
        assert(!front.Is_Enabled());
        update(110.0f, 110.0f, true);
        assert(front.Get_State() == UP_STAIR_HOME_HOLD);
        assert(near(front.Get_Target_Angle(1), rad(110.0f)));
    }
    for (float outside : {127.34f, 125.01f})
    {
        update(110.0f, outside, true);
        assert(!front.Is_Enabled());
        update(110.0f, 110.0f, true);
        assert(front.Get_State() == UP_STAIR_HOME_HOLD);
        assert(near(front.Get_Target_Angle(2), rad(470.0f)));
    }
}
