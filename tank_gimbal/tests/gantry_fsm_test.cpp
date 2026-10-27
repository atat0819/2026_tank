#include "gantry_fsm.hpp"
#include <cassert>
#include <cmath>
#include <limits>

using State = GantryState;
using Fault = GantryFault;

struct Rig
{
    Class_Gantry_FSM f;
    bool enabled = true;
    bool feedback_ok = true;
    bool lift_feedback_ok = true;
    bool right_raw = false;
    bool right_pressed = false;
    bool limit[4] = {false, false, false, true};
    double angle_deg[3] = {5000, -8000, 600};
    float current_a[2] = {};
    float velocity_rad_s[2] = {};
    uint32_t now_ms = 0;
    Rig(bool valid = true, bool control = true)
        : enabled(control), feedback_ok(valid), lift_feedback_ok(valid) {
        tick();
    }
    void tick(uint32_t dt = 5) {
        now_ms += dt;
        f.Update(enabled, feedback_ok, lift_feedback_ok, right_raw, right_pressed,
                 limit[0], limit[1], limit[2], limit[3],
                 angle_deg[0], angle_deg[1], angle_deg[2],
                 current_a[0], current_a[1], velocity_rad_s[0], velocity_rad_s[1], now_ms);
        if (!f.IsActive()) {
            assert(f.GetProgress() == 0 && f.GetScale() == 1);
            for (unsigned axis = 0; axis < 3; ++axis) {
                assert(!f.IsMoving(axis));
                assert(!f.IsKeeping(axis));
                assert(f.GetTargetSpeed(axis) == 0 && f.GetHoldAngle(axis) == 0);
            }
        }
    }
    void press() { right_raw = right_pressed = true; tick(); }
    void release() { right_raw = right_pressed = false; tick(); }
    void lift(float delta) { angle_deg[0] += delta; angle_deg[1] -= delta; tick(); }
    void horizontal(float delta) { angle_deg[2] += delta; tick(); }
    void hit(unsigned i) { limit[i] = true; tick(); limit[i] = false; }
    void lower() {
        press(); limit[3] = false;
        lift(1000); hit(0); horizontal(-500); hit(1);
        lift(1000); hit(2); horizontal(500); limit[3] = true; tick();
        assert(f.GetState() == State::Lower);
    }
};

int main()
{
    {
        Rig r;
        assert(r.f.GetState() == State::Idle && !r.f.IsActive());
        for (unsigned axis = 0; axis < 3; ++axis) assert(!r.f.IsKeeping(axis));
        r.press();
        assert(r.f.GetState() == State::LiftFirst && r.f.IsActive());
        assert(!r.f.IsKeeping(0) && !r.f.IsKeeping(1) && r.f.IsKeeping(2));
        assert(r.f.GetTargetSpeed(0) == 10 && r.f.GetTargetSpeed(1) == -10);
        r.lift(499); assert(r.f.GetScale() == 1);
        r.lift(1); assert(r.f.GetScale() == .5f);
        r.lift(349); assert(r.f.GetScale() == .5f);
        r.lift(1); assert(r.f.GetScale() == .1f);
        r.lift(-1); assert(r.f.GetScale() == .1f); // 回弹不重新加速
        r.lift(151);
        r.hit(0);
        assert(r.f.GetState() == State::Extend && r.f.GetScale() == 1);
        assert(!r.f.IsMoving(0) && !r.f.IsMoving(1) && r.f.IsMoving(2));
        assert(r.f.IsKeeping(0) && r.f.IsKeeping(1) && !r.f.IsKeeping(2));
        const double hold = r.f.GetHoldAngle(0);
        r.angle_deg[0] -= 2; r.tick(); assert(r.f.GetHoldAngle(0) == hold);
        r.angle_deg[0] += 2;
        r.limit[3] = false;
        r.horizontal(-250); assert(r.f.GetScale() == .5f);
        r.horizontal(-175); assert(r.f.GetScale() == .1f);
        r.horizontal(-75); r.hit(1);
        assert(r.f.GetState() == State::LiftSecond && r.f.GetScale() == 1);
        assert(!r.f.IsKeeping(0) && !r.f.IsKeeping(1) && r.f.IsKeeping(2));
        r.lift(500); assert(r.f.GetScale() == 1 && r.f.GetTargetSpeed(0) == 10);
        r.lift(400); assert(r.f.GetScale() == 1);
        r.lift(100); r.hit(2);
        assert(r.f.GetState() == State::Retract && r.f.GetTargetSpeed(2) == 2);
        assert(r.f.IsKeeping(0) && r.f.IsKeeping(1) && !r.f.IsKeeping(2));
        r.horizontal(250); assert(r.f.GetScale() == .5f);
        r.horizontal(175); assert(r.f.GetScale() == .1f);
        r.horizontal(75); r.limit[3] = true; r.tick();
        assert(r.f.GetState() == State::Lower && r.f.GetTargetSpeed(0) == -10);
        assert(!r.f.IsKeeping(0) && !r.f.IsKeeping(1) && r.f.IsKeeping(2));
        // Lower keeps each configured speed through the 50%, 85% and 100% positions.
        const float descent_steps[] = {-999, -1, -699, -1, -300};
        for (float delta : descent_steps) {
            r.lift(delta);
            assert(r.f.GetScale() == 1);
            assert(r.f.GetTargetSpeed(0) == -GantryTuning::LIFT_ID5_SPEED_RAD_S);
            assert(r.f.GetTargetSpeed(1) == GantryTuning::LIFT_ID6_SPEED_RAD_S);
        }
        r.current_a[0] = 1;
        r.tick(); r.tick(99); assert(r.f.GetState() == State::Lower);
        r.tick(1);
        assert(r.f.GetState() == State::Idle && r.f.IsActive());
        assert(!r.f.IsKeeping(0) && !r.f.IsKeeping(1) && r.f.IsKeeping(2));
        for (unsigned axis = 0; axis < 3; ++axis)
            assert(!r.f.IsMoving(axis) && r.f.GetTargetSpeed(axis) == 0);
        for (int i = 0; i < 10; ++i) r.tick();
        assert(r.f.GetState() == State::Idle); // 全程按住只运行一次
        r.release(); r.press(); assert(r.f.GetState() == State::LiftFirst);
    }
    {
        Rig r; r.press(); r.release(); r.press(); r.release();
        r.limit[3] = false;
        r.lift(1000); r.hit(0); r.horizontal(-500); r.hit(1);
        r.lift(1000); r.hit(2); r.horizontal(500); r.limit[3] = true; r.tick();
        r.lift(-2000); r.current_a[1] = -1; r.tick(); r.tick(100);
        assert(r.f.GetState() == State::Idle);
        r.tick(); assert(r.f.GetState() == State::Idle); // 忙时点击不排队
    }
    {
        Rig r; r.enabled = false; r.tick();
        r.right_raw = true; r.right_pressed = false;
        r.enabled = true; r.tick();
        r.press(); assert(r.f.GetState() == State::Idle); // 按住右键切入模式
        r.release(); r.press(); assert(r.f.GetState() == State::LiftFirst);
    }
    {
        Rig r; r.angle_deg[0] += 20; r.press();
        assert(r.f.GetFault() == Fault::NotHome && !r.f.IsActive());
        r.angle_deg[0] -= 20; r.release(); r.press();
        assert(r.f.GetState() == State::Fault); // 故障锁存，退出模式才能复位
        r.enabled = false; r.tick(); r.enabled = true; r.release(); r.press();
        assert(r.f.GetState() == State::LiftFirst);
        r.feedback_ok = false; r.tick();
        assert(r.f.GetFault() == Fault::Feedback && !r.f.IsActive());
        r.feedback_ok = true; r.tick(); assert(!r.f.IsActive());
    }
    {
        Rig r; r.press(); r.tick(10000);
        assert(r.f.GetFault() == Fault::Timeout && !r.f.IsActive());
    }
    {
        Rig r; r.now_ms = UINT32_MAX - 100; r.press(); r.tick(200);
        assert(r.f.GetState() == State::LiftFirst);
        r.tick(10000); assert(r.f.GetFault() == Fault::Timeout);
    }
    {
        Rig r; r.press(); r.lift(1200);
        assert(r.f.GetFault() == Fault::Travel && !r.f.IsActive());
    }
    {
        Rig r; r.press(); r.angle_deg[0] += 200; r.tick();
        assert(r.f.GetFault() == Fault::LiftMismatch && !r.f.IsActive());
    }
    {
        Rig r; r.press(); r.angle_deg[0] += 500; r.angle_deg[1] -= 490; r.tick();
        assert(r.f.GetScale() == .5f && r.f.GetTargetSpeed(0) == -r.f.GetTargetSpeed(1));
    }
    {
        Rig r; r.limit[0] = true; r.press();
        assert(r.f.GetFault() == Fault::SwitchSequence && !r.f.IsActive());
    }
    {
        // Record boot home while control is closed; later mode entry must not rebase it.
        Rig r(true, false);
        r.angle_deg[0] += 20; r.angle_deg[1] -= 20;
        r.enabled = true; r.release(); r.press();
        assert(r.f.GetFault() == Fault::NotHome);
    }
    {
        Rig r(false); r.angle_deg[0] = 10000; r.angle_deg[1] = -20000;
        r.feedback_ok = r.lift_feedback_ok = true; r.release(); r.press();
        assert(r.f.GetState() == State::LiftFirst); // Capture only fresh startup feedback.
    }
    {
        // DM can remain disabled with the control gate closed; lift home must be recorded now.
        Rig r(false, false); r.lift_feedback_ok = true; r.tick();
        r.angle_deg[0] += 20; r.angle_deg[1] -= 20;
        r.feedback_ok = r.enabled = true; r.release(); r.press();
        assert(r.f.GetFault() == Fault::NotHome);
    }
    {
        Rig r; r.press(); r.lift(100);
        r.feedback_ok = false; r.tick(); assert(r.f.GetFault() == Fault::Feedback);
        r.enabled = false; r.tick(); r.feedback_ok = r.enabled = true;
        r.release(); r.press(); assert(r.f.GetFault() == Fault::NotHome);
    }
    {
        Rig r; r.lower(); r.lift(-2000);
        r.tick(150); assert(r.f.GetState() == State::Lower); // No high current, no home event.
        r.current_a[0] = 2; r.velocity_rad_s[1] = .2f;
        r.tick(); r.tick(150); assert(r.f.GetState() == State::Lower);
        r.velocity_rad_s[1] = 0;
        r.tick(); r.tick(50);
        r.current_a[0] = 0; r.tick(); // A transient must reset the full confirmation time.
        r.current_a[1] = -1;
        r.tick(); r.tick(99); assert(r.f.GetState() == State::Lower);
        r.tick(1); assert(r.f.GetState() == State::Idle);
    }
    {
        Rig r; r.lower(); r.lift(-1000);
        r.current_a[0] = 1; r.tick(); r.tick(100);
        assert(r.f.GetFault() == Fault::Stall && !r.f.IsActive());
        r.current_a[0] = 0; r.tick();
        assert(r.f.GetState() == State::Fault); // A mid-travel jam is not home.
    }
    {
        Rig r; r.lower(); r.lift(-2000);
        r.current_a[0] = 1; r.tick(); r.tick(50);
        r.feedback_ok = false; r.tick();
        r.feedback_ok = true; r.tick(100);
        assert(r.f.GetFault() == Fault::Feedback && !r.f.IsActive());
    }
    {
        Rig r; r.lower(); r.lift(-2000);
        r.current_a[0] = std::numeric_limits<float>::quiet_NaN(); r.tick();
        assert(r.f.GetFault() == Fault::Feedback && !r.f.IsActive());
    }
    {
        Rig r; r.lower(); r.lift(-2000);
        r.now_ms = UINT32_MAX - 50;
        // Re-enter Lower near rollover without letting the action timeout expire.
        r.f = Class_Gantry_FSM{};
        r.angle_deg[0] = 5000; r.angle_deg[1] = -8000;
        r.current_a[0] = 0; r.release(); r.lower(); r.lift(-2000);
        r.current_a[0] = 1; r.tick(); r.tick(99);
        assert(r.f.GetState() == State::Lower);
        r.tick(1); assert(r.f.GetState() == State::Idle);
    }
    {
        Rig r; r.lower(); r.tick(10000);
        assert(r.f.GetFault() == Fault::Timeout);
    }
}
