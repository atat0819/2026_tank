#include "../feeder_fsm/imu_feedback_fsm.hpp"
#include <cassert>
#include <cmath>
#include <limits>

int main()
{
    constexpr float deg = 3.14159265358979323846f / 180.0f;
    const ImuFeedbackFSM::Sample zero = {0, 0, 0, 0};
    ImuFeedbackFSM fsm;
    assert(fsm.UsingEncoder());
    assert(!fsm.Update(true, zero, 0));
    assert(!fsm.Update(true, zero, 99));
    assert(fsm.Update(true, zero, 100)); // zero is a valid stationary pose
    assert(!fsm.UsingEncoder());
    assert(fsm.Update(false, zero, 101));
    assert(fsm.UsingEncoder());
    assert(!fsm.Update(true, zero, 102));
    auto bad = zero;
    bad.yaw_speed_rad_s = std::numeric_limits<float>::infinity();
    assert(!fsm.Update(true, bad, 150)); // invalid resets recovery timer
    assert(!fsm.Update(true, zero, 151));
    assert(!fsm.Update(true, zero, 250));
    assert(fsm.Update(true, zero, 251));
    bad = zero;
    bad.pitch_rad = std::numeric_limits<float>::quiet_NaN();
    assert(fsm.Update(true, bad, 252));
    assert(fsm.UsingEncoder());

    ImuFeedbackFSM wrap;
    auto a = zero;
    a.yaw_rad = 179 * deg;
    wrap.Update(true, a, 0);
    a.yaw_rad = -179 * deg;
    assert(wrap.Update(true, a, 100)); // seam crossing is only two degrees
    a.yaw_rad += 44 * deg;
    assert(!wrap.Update(true, a, 101));
    a.yaw_rad += 46 * deg;
    assert(wrap.Update(true, a, 102));
    assert(wrap.UsingEncoder());
    assert(!wrap.Update(true, a, 103)); // rebase recovery at the new pose
    assert(wrap.Update(true, a, 203));
    a.pitch_rad = 46 * deg;
    assert(wrap.Update(true, a, 204)); // Pitch has the same jump threshold

    ImuFeedbackFSM boundary;
    boundary.Update(true, zero, 0);
    boundary.Update(true, zero, 100);
    auto at_limit = zero;
    at_limit.yaw_rad = ImuFeedbackFSM::MAX_JUMP_RAD;
    at_limit.pitch_rad = ImuFeedbackFSM::MAX_JUMP_RAD;
    assert(!boundary.Update(true, at_limit, 101)); // exactly 45 degrees is allowed
    at_limit.pitch_speed_rad_s = std::numeric_limits<float>::quiet_NaN();
    assert(boundary.Update(true, at_limit, 102)); // either gyro can invalidate IMU

    ImuFeedbackFSM tick_wrap;
    tick_wrap.Update(true, zero, 0xfffffff0U);
    assert(!tick_wrap.Update(true, zero, 0x53U));
    assert(tick_wrap.Update(true, zero, 0x54U));
}
