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
kd_declaration = re.search(r"(?m)^float FRONT_MIT_KD\s*=\s*[^;]+;", source)
assert kd_declaration is not None, "front MIT damping must have one shared tuning variable"
declarations = kd_declaration.group() + "\n" + "\n".join(re.findall(
    r"ALG::PID::PID front_4340_(?:left|right)_pid = \{.*?\};",
    source, flags=re.S))
for side in ("left", "right"):
    parameters = re.search(
        rf"ALG::PID::PID front_4340_{side}_pid = \{{(.*?)\}};", source, re.S
    ).group(1).split(",")
    declarations += f"\nconstexpr float {side}_kp = {parameters[0].strip()};"
    declarations += f"\nconstexpr float {side}_speed_limit = {parameters[3].strip()};"
host = r'''
#include <cassert>
#include <cmath>
#include "fsm/up_stair_fsm.hpp"
#include "user/core/Alg/PID/pid.hpp"
extern float FRONT_MIT_KD;
Class_Up_Stair_FSM up_stair_fsm;
float rad(float v) { return v * 3.14159265358979323846f / 180; }
bool near(float a, float b) { return std::fabs(a-b) < 1e-5f; }
float SafeMotorAngle(float v) { return std::isfinite(v) ? v : 0; }
float front_left_target_velocity;
uint32_t now_tick = 0U;
struct MotorSink {
    float speed[2] = {}, kd[2] = {}, torque[2] = {}, actual_speed[2] = {};
    unsigned sent[2] = {};
    void ctrl_Mit(unsigned id, float position, float velocity, float kp,
                  float damping, float effort) {
        assert(std::isfinite(position) && std::isfinite(velocity));
        assert(std::fabs(velocity) <= 10 && kp == 0 && effort == 0);
        assert(up_stair_fsm.Is_Enabled() ? near(damping, FRONT_MIT_KD) : damping == 0);
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
         bool left_online = true, bool right_online = true, bool enabled = true,
         bool command_enabled = true) {
    const float front_left_angle = left, front_right_angle = right;
    const bool front_left_feedback_valid = left_online && up_stair_fsm.Is_Angle_Valid(1, front_left_angle);
    const bool front_right_feedback_valid = right_online && up_stair_fsm.Is_Angle_Valid(2, front_right_angle);
    front_4340.actual_speed[0] = front_left_velocity;
    front_4340.actual_speed[1] = front_right_velocity;
    up_stair_fsm.Update(front_left_angle, front_right_angle,
        front_left_feedback_valid, front_right_feedback_valid, enabled, command_enabled, target, now_tick);
'''
main = r'''
}
int main() {
    using Front = Class_Up_Stair_FSM;
    const float hl = Front::HOME_ANGLE_RAD[0], hr = Front::HOME_ANGLE_RAD[1];
    const float tl = Front::TARGET_ANGLE_RAD[0], tr = Front::TARGET_ANGLE_RAD[1];
    const float ml = (hl + tl) / 2, mr = (hr + tr) / 2;
    const uint32_t ramp_time = Front::SPEED_RAMP_TIME_MS;
    assert(ramp_time == 1000U);
    const float error = rad(0.05f);
    up_stair_fsm.Init();
    run(hl, hr);
    assert(near(front_4340.speed[0], 0) && near(front_4340.speed[1], 0));
    // Both encoder angle outer loops already output rad/s; do not convert again.
    now_tick = ramp_time;
    run(hl - error, hr + error);
    assert(near(front_4340.speed[0], left_kp * error));
    assert(near(front_4340.speed[1], -right_kp * error));
    // New motion starts at zero speed, while exposing the final target angles.
    ++now_tick;
    const uint32_t start = now_tick;
    run(hl, hr, true);
    assert(near(front_4340.speed[0], 0) && near(front_4340.speed[1], 0));
    assert(near(up_stair_fsm.Get_Target_Angle(1), tl));
    assert(near(up_stair_fsm.Get_Target_Angle(2), tr));
    now_tick = start + ramp_time / 4U;
    run(hl, hr, true);
    assert(near(front_4340.speed[0], left_speed_limit * 0.25f));
    assert(near(front_4340.speed[1], -right_speed_limit * 0.25f));
    run(hl, hr, true); // Repeated tick must not advance the ramp.
    assert(near(front_4340.speed[0], left_speed_limit * 0.25f));
    // Encoder feedback moving ahead does not bypass the time-based ramp.
    now_tick = start + ramp_time / 2U;
    run(ml, mr, true);
    assert(near(front_4340.speed[0], left_speed_limit * 0.5f));
    assert(near(front_4340.speed[1], -right_speed_limit * 0.5f));
    run(tl, tr, true, 0.3f, -0.3f); // Reaching HOLD does not restart the ramp.
    assert(up_stair_fsm.Get_State() == UP_STAIR_TARGET_HOLD);
    assert(near(front_4340.speed[0], 0) && near(front_4340.speed[1], 0));
    now_tick = start + 3U * ramp_time / 4U;
    run(tl - error, tr + error, true);
    assert(near(front_4340.speed[0], left_kp * error * 0.75f));
    assert(near(front_4340.speed[1], -right_kp * error * 0.75f));
    now_tick = start + ramp_time;
    run(tl - error, tr + error, true);
    assert(near(front_4340.speed[0], left_kp * error));
    assert(near(front_4340.speed[1], -right_kp * error));
    const float left_before = front_4340.speed[0];
    const float right_before = front_4340.speed[1];
    run(tl - error / 2, tr + error / 2, true);
    assert(near(front_4340.speed[0] - left_before, -left_kp * error / 2));
    assert(near(front_4340.speed[1] - right_before, right_kp * error / 2));
    run(tl, tr, true, 0.3f, -0.3f);
    assert(near(front_4340.speed[0], 0) && near(front_4340.speed[1], 0));
    assert(near(front_4340.torque[0], -FRONT_MIT_KD * 0.3f));
    assert(near(front_4340.torque[1], FRONT_MIT_KD * 0.3f));
    FRONT_MIT_KD = 2.1f;
    run(tl, tr, true, 0.3f, -0.3f);
    assert(near(front_4340.kd[0], 2.1f) && near(front_4340.kd[1], 2.1f));
    assert(near(front_4340.torque[0], -FRONT_MIT_KD * 0.3f));
    assert(near(front_4340.torque[1], FRONT_MIT_KD * 0.3f));
    run(tl, tr, false); // Returning HOME also starts at zero.
    assert(near(front_4340.speed[0], 0) && near(front_4340.speed[1], 0));
    assert(near(up_stair_fsm.Get_Target_Angle(1), hl));
    assert(near(up_stair_fsm.Get_Target_Angle(2), hr));
    now_tick += ramp_time / 4U;
    run(tl, tr, false);
    assert(near(front_4340.speed[0], -left_speed_limit * 0.25f));
    assert(near(front_4340.speed[1], right_speed_limit * 0.25f));
    run(ml, mr, true); // Reverse before HOME: restart the shared ramp once.
    assert(near(front_4340.speed[0], 0) && near(front_4340.speed[1], 0));
    now_tick += ramp_time / 2U;
    run(ml, mr, true);
    assert(near(front_4340.speed[0], left_speed_limit * 0.5f));
    assert(near(front_4340.speed[1], -right_speed_limit * 0.5f));
    run(ml, mr, true, 0, 0, true, true, true, false); // Forced HOME.
    assert(near(front_4340.speed[0], 0) && near(front_4340.speed[1], 0));
    now_tick += ramp_time / 4U;
    run(ml, mr, true, 0, 0, true, true, true, false);
    assert(near(front_4340.speed[0], -left_speed_limit * 0.25f));
    assert(near(front_4340.speed[1], right_speed_limit * 0.25f));
    now_tick += 5U * ramp_time; // Long time step is capped at full output.
    run(ml, mr, true, 0, 0, true, true, true, false);
    assert(near(front_4340.speed[0], -left_speed_limit));
    assert(near(front_4340.speed[1], right_speed_limit));
    // One missing feedback, bad angle or disabled permission stops both motors.
    run(ml, mr, false, 0.3f, -0.3f, false);
    assert(front_4340.kd[0] == 0 && front_4340.kd[1] == 0);
    now_tick += 20000U;
    run(ml, mr, true); // Recovery homes first, without replaying B or old ramp.
    assert(up_stair_fsm.Get_State() == UP_STAIR_RETURNING_HOME);
    assert(near(front_4340.speed[0], 0) && near(front_4340.speed[1], 0));
    now_tick += ramp_time / 2U;
    run(ml, mr, true);
    assert(near(front_4340.speed[0], -left_speed_limit * 0.5f));
    assert(near(front_4340.speed[1], right_speed_limit * 0.5f));
    now_tick += ramp_time / 2U;
    run(ml, mr, true);
    assert(front_4340.speed[0] < 0 && front_4340.speed[1] > 0);
    run(hl, hr, true);
    run(hl, hr, true);
    assert(up_stair_fsm.Get_State() == UP_STAIR_HOME_HOLD);
    run(ml, mr, true, 0, 0, true, false);
    assert(front_4340.kd[0] == 0 && front_4340.kd[1] == 0);
    run(ml, mr);
    run(Front::LIMIT_START_RAD[0] - error, mr);
    assert(front_4340.kd[0] == 0 && front_4340.kd[1] == 0);
    run(ml, mr);
    run(ml, Front::LIMIT_END_RAD[1] + error);
    assert(front_4340.kd[0] == 0 && front_4340.kd[1] == 0);
    run(ml, mr);
    run(NAN, mr);
    assert(front_4340.kd[0] == 0 && front_4340.kd[1] == 0);
    run(ml, mr);
    run(ml, mr, false, 0.3f, -0.3f, true, true, false);
    assert(front_4340.kd[0] == 0 && front_4340.kd[1] == 0);
    // First enable and unsigned millisecond rollover retain the same ramp.
    up_stair_fsm.Init();
    now_tick = UINT32_MAX - ramp_time / 4U;
    run(ml, mr);
    assert(near(front_4340.speed[0], 0) && near(front_4340.speed[1], 0));
    now_tick += ramp_time / 2U;
    run(ml, mr);
    assert(near(front_4340.speed[0], -left_speed_limit * 0.5f));
    assert(near(front_4340.speed[1], right_speed_limit * 0.5f));
    now_tick += ramp_time / 2U;
    run(ml, mr);
    assert(near(front_4340.speed[0], -left_speed_limit));
    assert(near(front_4340.speed[1], right_speed_limit));
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
        (ROOT / "user/core/Alg/FSM/alg_fsm.cpp").as_posix(),
        (ROOT / "user/core/Alg/UtilityFunction/SlopPlanning.cpp").as_posix(),
        "-o", exe.as_posix(),
    ], check=True)
    subprocess.run([str(exe)], check=True)
print("PASS: front shared 1 s speed ramp, unchanged final angles, HOME/TARGET/reversal/forced HOME, HOLD, recovery, rollover and pair fault zero gains")
