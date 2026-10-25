#include "gantry_fsm.hpp"
#include <cassert>
#include <cmath>

using State = GantryState;
using Fault = GantryFault;

struct Rig
{
    Class_Gantry_FSM f;
    bool enabled = true;
    bool feedback_ok = true;
    bool right_raw = false;
    bool right_pressed = false;
    bool limit[5] = {false, false, false, true, true};
    double angle_deg[3] = {5000, -8000, 600};
    uint32_t now_ms = 0;
    Rig() {
        tick();
    }
    void tick(uint32_t dt = 5) {
        now_ms += dt;
        f.Update(enabled, feedback_ok, right_raw, right_pressed,
                 limit[0], limit[1], limit[2], limit[3], limit[4],
                 angle_deg[0], angle_deg[1], angle_deg[2], now_ms);
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
        r.limit[4] = false;
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
        r.lift(-1000); assert(r.f.GetScale() == .5f);
        r.lift(-700); assert(r.f.GetScale() == .1f);
        r.lift(-300); r.limit[4] = true; r.tick();
        assert(r.f.GetState() == State::Idle && r.f.IsActive());
        for (unsigned axis = 0; axis < 3; ++axis) assert(r.f.IsKeeping(axis));
        for (int i = 0; i < 10; ++i) r.tick();
        assert(r.f.GetState() == State::Idle); // 全程按住只运行一次
        r.release(); r.press(); assert(r.f.GetState() == State::LiftFirst);
    }
    {
        Rig r; r.press(); r.release(); r.press(); r.release();
        r.limit[3] = r.limit[4] = false;
        r.lift(1000); r.hit(0); r.horizontal(-500); r.hit(1);
        r.lift(1000); r.hit(2); r.horizontal(500); r.hit(3); r.hit(4);
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
        Rig r; r.limit[4] = false; r.press();
        assert(r.f.GetFault() == Fault::NotHome && !r.f.IsActive());
        r.limit[4] = true; r.release(); r.press();
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
}
