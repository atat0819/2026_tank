"""Execute the real rear task output branch with real FSM/PIDs and a CAN sink."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "RtosTask/up_stair.cpp").read_text(encoding="utf-8")
start = source.index("        const bool rear_both_controllable =")
end = source.index("\n        }\n\n        osDelay", start)
branch = source[start:end]
kd_declaration = re.search(r"static constexpr float REAR_MIT_KD\s*=\s*[^;]+;", source).group(0)
pid_declarations = "\n".join(re.findall(
    r"ALG::PID::PID rear_6248_(?:pitch|left_retract|right_retract)_pid = \{.*?\};",
    source, flags=re.S))
host = r'''
#include <cassert>
#include <cmath>
#include <initializer_list>
#include "fsm/up_stair_behind_motor_fsm.hpp"
#include "fsm/rear_torque_safety.hpp"
#include "user/core/Alg/PID/pid.hpp"
using Rear = Class_Up_Stair_Behind_Motor_FSM;
Rear up_stair_behind_motor_fsm(Rear::Config::WithAngleConstants());
float deg(float v) { return v * 3.14159265358979323846f / 180; }
float SafeMotorAngle(float v) { return std::isfinite(v) ? v : 0; }
struct MotorSink {
    float torque[2] = {};
    float actual_velocity[2] = {}, target_velocity[2] = {}, kd[2] = {};
    unsigned sent[2] = {};
    void ctrl_Mit(unsigned id, float, float velocity, float kp, float damping, float effort) {
        assert(kp == 0 && effort == 0);
        assert(std::isfinite(velocity) && std::fabs(velocity) <= 20);
        const bool active = up_stair_behind_motor_fsm.Is_Motor_Controllable(1) &&
            up_stair_behind_motor_fsm.Is_Motor_Controllable(2) &&
            (up_stair_behind_motor_fsm.Uses_Attitude_Control() ||
             up_stair_behind_motor_fsm.Uses_Position_Control());
        const float expected_kd = REAR_MIT_KD;
        assert(active ? std::fabs(damping - expected_kd) < 1e-6f : damping == 0);
        if (!active) assert(velocity == 0);
        target_velocity[id-1] = velocity;
        kd[id-1] = damping;
        torque[id-1] = damping == 0 ? 0 : damping * (velocity - actual_velocity[id-1]);
        assert(std::isfinite(torque[id-1]) && std::fabs(torque[id-1]) <= 30.00001f);
        ++sent[id-1];
    }
} rear_6248;
float left_pitch_velocity, right_pitch_velocity;
'''
wrapper = r'''
void run(uint32_t tick, float rear_left_angle, float rear_right_angle,
         float pitch, float rear_left_velocity = 0, float rear_right_velocity = 0,
         bool enabled = true, bool imu = true, bool retract = false,
         bool left_feedback = true, bool right_feedback = true, float pitch_rate = 0) {
    // Test Pitch is the deviation from the calibrated working reference.
    const float raw_pitch = pitch + Rear::PITCH_TARGET_DEG + Rear::PITCH_ZERO_DEG;
    up_stair_behind_motor_fsm.Update(enabled, imu, left_feedback, right_feedback, raw_pitch, pitch_rate,
        rear_left_angle, rear_right_angle, tick, retract,
        rear_left_velocity, rear_right_velocity);
    rear_6248.actual_velocity[0] = rear_left_velocity;
    rear_6248.actual_velocity[1] = rear_right_velocity;
'''
main = r'''
}
int main() {
    // The production Pitch PID consumes radians and returns rad/s.
    assert(std::fabs(rear_6248_pitch_pid.UpDate(0.1f, 0) - 1.5f) < 1e-6f);
    assert(std::fabs(rear_6248_pitch_pid.UpDate(1, 0) - 5.4f) < 1e-6f);
    run(0, deg(60), deg(40), -5);
    run(2000, deg(60), deg(40), -5);
    const float nominal = deg(5) * 15;
    assert(std::fabs(rear_6248.target_velocity[0] - nominal) < 1e-5f);
    assert(std::fabs(rear_6248.target_velocity[1] + nominal) < 1e-5f);
    // No preemptive braking or automatic position recovery, even after a long dwell.
    for (uint32_t tick : {2001U, 4001U, 10001U}) {
        run(tick, Rear::ANGLE_END_RAD[0] - deg(1), Rear::ANGLE_START_RAD[1] + deg(1), -5, 0.8f, -0.8f);
        assert(std::fabs(rear_6248.target_velocity[0] - nominal) < 1e-5f);
        assert(std::fabs(rear_6248.target_velocity[1] + nominal) < 1e-5f);
    }
    // Upper left / lower right: outward effort is zero, inward effort passes immediately.
    run(10002, Rear::ANGLE_END_RAD[0], Rear::ANGLE_START_RAD[1], -5, 0.4f, -0.4f);
    assert(std::fabs(rear_6248.torque[0]) < 1e-5f && std::fabs(rear_6248.torque[1]) < 1e-5f);
    assert(std::fabs(rear_6248.target_velocity[0] - 0.4f) < 1e-5f);
    assert(std::fabs(rear_6248.target_velocity[1] + 0.4f) < 1e-5f);
    run(10003, Rear::ANGLE_END_RAD[0], Rear::ANGLE_START_RAD[1], 5);
    assert(rear_6248.torque[0] < 0 && rear_6248.torque[1] > 0);
    // Lower left / upper right use the opposite torque signs.
    run(10004, Rear::ANGLE_START_RAD[0], Rear::ANGLE_END_RAD[1], 5);
    assert(std::fabs(rear_6248.torque[0]) < 1e-5f && std::fabs(rear_6248.torque[1]) < 1e-5f);
    run(10005, Rear::ANGLE_START_RAD[0], Rear::ANGLE_END_RAD[1], -5);
    assert(rear_6248.torque[0] > 0 && rear_6248.torque[1] < 0);
    // No latched recovery: immediately resume nominal output after leaving the band.
    run(10006, Rear::ANGLE_END_RAD[0] - deg(0.6f), Rear::ANGLE_START_RAD[1] + deg(0.6f), -5);
    assert(std::fabs(rear_6248.target_velocity[0] - nominal) < 1e-5f);
    // With zero position error, do not invent an inward backoff command.
    run(10007, Rear::ANGLE_END_RAD[0], Rear::ANGLE_START_RAD[1], 0);
    assert(std::fabs(rear_6248.torque[0]) < 1e-5f && std::fabs(rear_6248.torque[1]) < 1e-5f);
    // Safety failures still zero both native speed-loop gains.
    run(10008, deg(60), deg(40), 5, 0, 0, false);
    assert(rear_6248.kd[0] == 0 && rear_6248.kd[1] == 0);
    run(10009, deg(60), deg(40), 5, 0, 0, true, false);
    assert(rear_6248.kd[0] == 0 && rear_6248.kd[1] == 0);
    run(10010, deg(60), deg(40), 5, NAN);
    assert(rear_6248.kd[0] == 0 && rear_6248.kd[1] == 0);
    run(10011, deg(60), deg(40), 5, 0, 0, true, true, false, false, true);
    assert(rear_6248.kd[0] == 0 && rear_6248.kd[1] == 0);
    run(10012, deg(60), deg(40), 5, 0, 0, true, true, false, true, false);
    assert(rear_6248.kd[0] == 0 && rear_6248.kd[1] == 0);
    run(10013, Rear::ANGLE_END_RAD[0] + deg(0.6f), deg(40), 5);
    assert(rear_6248.kd[0] == 0 && rear_6248.kd[1] == 0);
    // Retraction keeps its existing targets, ramp, Kd and encoder direction convention.
    run(11000, deg(60), deg(40), 0, 0, 0, false);
    run(11001, deg(60), deg(40), 0, 0, 0, true, true, true);
    run(13001, deg(60), deg(40), 0, 0, 0, true, true, true);
    assert(rear_6248.torque[0] > 0 && rear_6248.torque[1] < 0);
    assert(std::fabs(up_stair_behind_motor_fsm.Get_Position_Target_Angle(1) - deg(86.7f)) < 1e-5f);
    assert(std::fabs(up_stair_behind_motor_fsm.Get_Position_Target_Angle(2) - deg(12.66f)) < 1e-5f);
    // Use the actual commanded Kd=3.0 for the sampled torque cap.
    run(13002, deg(60), deg(40), 0, -20, 20, true, true, true);
    assert(std::fabs(rear_6248.torque[0] - 30) < 1e-5f);
    assert(std::fabs(rear_6248.torque[1] + 30) < 1e-5f);
    assert(rear_6248.sent[0] == rear_6248.sent[1]);
}
'''
with tempfile.TemporaryDirectory(prefix="rear-soft-limit-task-") as directory:
    work = Path(directory)
    test = work / "test.cpp"
    test.write_text(kd_declaration + "\n" + host + pid_declarations + wrapper + branch + main, encoding="utf-8")
    exe = work / "test.exe"
    subprocess.run([
        "g++", "-std=c++17", "-I", ROOT.as_posix(), test.as_posix(),
        (ROOT / "fsm/up_stair_behind_motor_fsm.cpp").as_posix(),
        (ROOT / "user/core/Alg/PID/pid.cpp").as_posix(),
        (ROOT / "user/core/Alg/FSM/alg_fsm.cpp").as_posix(),
        (ROOT / "user/core/Alg/UtilityFunction/SlopPlanning.cpp").as_posix(),
        "-o", exe.as_posix(),
    ], check=True)
    subprocess.run([str(exe)], check=True)
print("PASS: rear task endpoint torque signs, no slowdown/backoff, retract Kd consistency, units and fault zero gains")
