"""Run the real J4340 output branch with its real angle FSM and PID."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "RtosTask/up_stair.cpp").read_text(encoding="utf-8")
start = source.index("        if (front_left_feedback_valid && up_stair_fsm.Is_Enabled())")
end = source.index("        const bool rear_both_controllable =", start)
branch = source[start:end]
declarations = "\n".join(re.findall(
    r"ALG::PID::PID front_4340_(?:left|right)_pid = \{.*?\};",
    source, flags=re.S))
host = r'''
#include <cassert>
#include <cmath>
#include "fsm/up_stair_fsm.hpp"
#include "user/core/Alg/PID/pid.hpp"
Class_Up_Stair_FSM up_stair_fsm;
float rad(float v) { return v * 3.14159265358979323846f / 180; }
bool near(float a, float b) { return std::fabs(a-b) < 1e-5f; }
float SafeMotorAngle(float v) { return std::isfinite(v) ? v : 0; }
float front_left_target_velocity;
struct MotorSink {
    float speed[2] = {}, kd[2] = {}, torque[2] = {}, actual_speed[2] = {};
    unsigned sent[2] = {};
    void ctrl_Mit(unsigned id, float position, float velocity, float kp,
                  float damping, float effort) {
        assert(std::isfinite(position) && std::isfinite(velocity));
        assert(std::fabs(velocity) <= 10 && kp == 0 && effort == 0);
        assert(up_stair_fsm.Is_Enabled() ? near(damping, 1.3f) : damping == 0);
        if (!up_stair_fsm.Is_Enabled()) assert(velocity == 0);
        speed[id-1] = velocity;
        kd[id-1] = damping;
        torque[id-1] = damping == 0 ? 0 : damping * (velocity - actual_speed[id-1]);
        ++sent[id-1];
    }
} front_4340;
'''
wrapper = r'''
void run(float left, float right, bool target = false,
         float front_left_velocity = 0, float front_right_velocity = 0,
         bool left_online = true, bool right_online = true, bool enabled = true) {
    const float front_left_angle = rad(left), front_right_angle = rad(right);
    const bool front_left_feedback_valid = left_online && up_stair_fsm.Is_Angle_Valid(1, front_left_angle);
    const bool front_right_feedback_valid = right_online && up_stair_fsm.Is_Angle_Valid(2, front_right_angle);
    front_4340.actual_speed[0] = front_left_velocity;
    front_4340.actual_speed[1] = front_right_velocity;
    up_stair_fsm.Update(front_left_angle, front_right_angle,
        front_left_feedback_valid, front_right_feedback_valid, enabled, true, target);
'''
main = r'''
}
int main() {
    up_stair_fsm.Init();
    run(139, -109);
    assert(near(front_4340.speed[0], 0) && near(front_4340.speed[1], 0));
    // Both encoder angle outer loops already output rad/s; do not convert again.
    run(138, -108);
    assert(near(front_4340.speed[0], 15 * rad(1)));
    assert(near(front_4340.speed[1], -8 * rad(1)));
    run(139, -109, true);
    assert(near(front_4340.speed[0], 10) && near(front_4340.speed[1], -10));
    run(179.8f, -150, true);
    const float left_before_seam = front_4340.speed[0];
    run(-179.8f, -150, true);
    assert(near(front_4340.speed[0] - left_before_seam, -15 * rad(0.4f)));
    run(170, -179.8f, true);
    const float right_before_seam = front_4340.speed[1];
    run(170, 179.8f, true);
    assert(near(front_4340.speed[1] - right_before_seam, 8 * rad(0.4f)));
    run(-160, 172, true, 0.3f, -0.3f);
    assert(up_stair_fsm.Get_State() == UP_STAIR_TARGET_HOLD);
    assert(near(front_4340.speed[0], 0) && near(front_4340.speed[1], 0));
    assert(near(front_4340.torque[0], -1.3f * 0.3f));
    assert(near(front_4340.torque[1], 1.3f * 0.3f));
    run(-160, 172, false);
    assert(front_4340.speed[0] < 0 && front_4340.speed[1] > 0);
    // One missing feedback, bad angle or disabled permission stops both motors.
    run(170, -150, false, 0.3f, -0.3f, false);
    assert(front_4340.kd[0] == 0 && front_4340.kd[1] == 0);
    run(170, -150, true); // Recovery first returns HOME, without replaying B.
    assert(up_stair_fsm.Get_State() == UP_STAIR_RETURNING_HOME);
    assert(front_4340.speed[0] < 0 && front_4340.speed[1] > 0);
    run(170, -150, true, 0, 0, true, false);
    assert(front_4340.kd[0] == 0 && front_4340.kd[1] == 0);
    run(170, -150);
    run(123, -150);
    assert(front_4340.kd[0] == 0 && front_4340.kd[1] == 0);
    run(170, -150);
    run(170, -93);
    assert(front_4340.kd[0] == 0 && front_4340.kd[1] == 0);
    run(170, -150);
    run(NAN, -150);
    assert(front_4340.kd[0] == 0 && front_4340.kd[1] == 0);
    run(170, -150);
    run(170, -150, false, 0.3f, -0.3f, true, true, false);
    assert(front_4340.kd[0] == 0 && front_4340.kd[1] == 0);
    assert(front_4340.sent[0] == front_4340.sent[1]);
}
'''
with tempfile.TemporaryDirectory(prefix="front-mit-speed-") as directory:
    work = Path(directory)
    test = work / "test.cpp"
    test.write_text(host + declarations + wrapper + branch + main, encoding="utf-8")
    exe = work / "test.exe"
    subprocess.run([
        "g++", "-std=c++17", "-Wall", "-Wextra", "-I", ROOT.as_posix(), test.as_posix(),
        (ROOT / "fsm/up_stair_fsm.cpp").as_posix(),
        (ROOT / "user/core/Alg/PID/pid.cpp").as_posix(),
        (ROOT / "user/core/Alg/FSM/alg_fsm.cpp").as_posix(), "-o", exe.as_posix(),
    ], check=True)
    subprocess.run([str(exe)], check=True)
print("PASS: front MIT Kd=1.3, angle outer loops, wrap seams, HOME/TARGET, shaft damping and pair fault zero gains")
