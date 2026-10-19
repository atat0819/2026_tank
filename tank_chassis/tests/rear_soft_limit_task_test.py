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
kd_declaration = "\n".join(re.search(
    rf"^float {name}\s*=\s*[^;]+;", source, flags=re.M).group(0)
    for name in ("REAR_IMU_MIT_KD", "REAR_RETRACT_MIT_KD", "REAR_IMU_PITCH_DAMPING"))
pid_declarations = "\n".join(re.findall(
    r"ALG::PID::PID rear_6248_(?:pitch|left_retract|right_retract)_pid = \{.*?\};",
    source, flags=re.S))
host = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <initializer_list>
#include "fsm/up_stair_behind_motor_fsm.hpp"
#include "fsm/rear_torque_safety.hpp"
#include "user/core/Alg/PID/pid.hpp"
using Rear = Class_Up_Stair_Behind_Motor_FSM;
const float REAR_IMU_ROLL_TARGET_DEG = Rear::ROLL_TARGET_DEG;
float REAR_IMU_ROLL_KP = Rear::ROLL_KP;
const float REAR_IMU_ROLL_MAX_TORQUE = Rear::ROLL_MAX_TORQUE_NM;
Rear::Config test_config() {
    auto config = Rear::Config::WithAngleConstants();
    config.roll_kp = REAR_IMU_ROLL_KP;
    config.retract_feedforward_torque_nm[0] = TEST_RETRACT_FF;
    config.retract_feedforward_torque_nm[1] = -TEST_RETRACT_FF;
    return config;
}
Rear up_stair_behind_motor_fsm(test_config());
float deg(float v) { return v * 3.14159265358979323846f / 180; }
float SafeMotorAngle(float v) { return std::isfinite(v) ? v : 0; }
float filtered_roll_deg = REAR_IMU_ROLL_TARGET_DEG;
struct MotorSink {
    float torque[2] = {};
    float actual_velocity[2] = {}, target_velocity[2] = {}, kd[2] = {};
    unsigned sent[2] = {};
    void ctrl_Mit(unsigned id, float, float velocity, float kp, float damping, float effort) {
        assert(kp == 0);
        float expected_effort = up_stair_behind_motor_fsm.Get_Retract_Feedforward_Torque(id);
        if (up_stair_behind_motor_fsm.Uses_Attitude_Control()) {
            const float roll = std::clamp(REAR_IMU_ROLL_KP *
                (filtered_roll_deg - REAR_IMU_ROLL_TARGET_DEG),
                -REAR_IMU_ROLL_MAX_TORQUE, REAR_IMU_ROLL_MAX_TORQUE);
            const float raw = up_stair_behind_motor_fsm.Get_Motor_Direction(id) *
                (-REAR_IMU_PITCH_DAMPING * deg(up_stair_behind_motor_fsm.Get_Pitch_Rate_Dps()) +
                 (id == 1 ? roll : -roll));
            expected_effort = StairTorqueSafety::ClampJ6248Torque(
                up_stair_behind_motor_fsm.Limit_Torque(id, raw));
        }
        assert(std::fabs(effort - expected_effort) < 1e-6f);
        assert(std::isfinite(velocity) && std::fabs(velocity) <= Rear::MIT_SPEED_LIMIT_RAD_S);
        const bool active = up_stair_behind_motor_fsm.Is_Motor_Controllable(1) &&
            up_stair_behind_motor_fsm.Is_Motor_Controllable(2) &&
            (up_stair_behind_motor_fsm.Uses_Attitude_Control() ||
             up_stair_behind_motor_fsm.Uses_Position_Control());
        const float expected_kd = up_stair_behind_motor_fsm.Uses_Position_Control()
            ? REAR_RETRACT_MIT_KD : REAR_IMU_MIT_KD;
        assert(active ? std::fabs(damping - expected_kd) < 1e-6f : damping == 0);
        if (!active) assert(velocity == 0);
        target_velocity[id-1] = velocity;
        kd[id-1] = damping;
        torque[id-1] = (damping == 0 ? 0 : damping * (velocity - actual_velocity[id-1])) + effort;
        assert(std::fabs(rear_estimated_torque_nm[id-1] - torque[id-1]) < 1e-5f);
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
        rear_left_velocity, rear_right_velocity, filtered_roll_deg);
    rear_6248.actual_velocity[0] = rear_left_velocity;
    rear_6248.actual_velocity[1] = rear_right_velocity;
'''
main = r'''
}
int main() {
    // Derive the expected speed from the current production PID calibration.
    const float nominal = rear_6248_pitch_pid.UpDate(deg(5), 0);
    rear_6248_pitch_pid.reset();
    run(0, deg(60), deg(40), -5);
    run(2000, deg(60), deg(40), -5);
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
    // At zero angle error, IMU motion alone creates mirrored braking torque.
    for (float rate : {-30.0f, 30.0f}) {
        run(10007, deg(60), deg(40), 0, 0, 0, true, true, false, true, true, rate);
        assert(std::fabs(rear_6248.torque[0] + REAR_IMU_PITCH_DAMPING * deg(rate)) < 1e-5f);
        assert(std::fabs(rear_6248.torque[1] - REAR_IMU_PITCH_DAMPING * deg(rate)) < 1e-5f);
        assert(std::fabs(rear_6248.target_velocity[0]) < 1e-5f);
        assert(std::fabs(rear_6248.target_velocity[1]) < 1e-5f);
    }
    run(10007, deg(60), deg(40), -5, 0.2f, -0.2f,
        true, true, false, true, true, 30);
    const float expected = REAR_IMU_MIT_KD * (nominal - 0.2f) -
        REAR_IMU_PITCH_DAMPING * deg(30);
    assert(std::fabs(rear_6248.torque[0] - expected) < 1e-5f);
    assert(std::fabs(rear_6248.torque[1] + expected) < 1e-5f);
    // Verify the sum at all endpoints, including a damping term that reverses
    // the speed-loop torque sign. Feedforward must enter protection exactly once.
    for (int endpoint : {-1, 0, 1}) {
        const float left = endpoint < 0 ? Rear::ANGLE_START_RAD[0] :
            endpoint > 0 ? Rear::ANGLE_END_RAD[0] : deg(60);
        const float right = endpoint < 0 ? Rear::ANGLE_START_RAD[1] :
            endpoint > 0 ? Rear::ANGLE_END_RAD[1] : deg(40);
        for (float rate : {-3000.0f, -30.0f, 0.0f, 30.0f, 3000.0f}) {
            for (float actual : {-10.0f, 0.0f, 10.0f}) {
                for (float pitch : {-5.0f, 0.0f, 5.0f}) {
                    run(10007, left, right, pitch, actual, actual,
                        true, true, false, true, true, rate);
                    for (unsigned i : {0U, 1U}) {
                        if (endpoint < 0) assert(rear_6248.torque[i] >= -1e-5f);
                        if (endpoint > 0) assert(rear_6248.torque[i] <= 1e-5f);
                    }
                }
            }
        }
    }
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
    // A real IMU fault starts recovery. Both the speed and damping torque scale
    // once, and a disabled command carries no damping feedforward.
    run(10014, deg(60), deg(40), 0, 0, 0, true, false, false, true, true, 30);
    run(10015, deg(60), deg(40), 0, 0, 0, true, true, false, true, true, 30);
    assert(std::fabs(rear_6248.torque[0]) < 1e-5f);
    run(11015, deg(60), deg(40), 0, 0, 0, true, true, false, true, true, 30);
    assert(std::fabs(rear_6248.torque[0] + 0.5f * REAR_IMU_PITCH_DAMPING * deg(30)) < 1e-5f);
    assert(std::fabs(rear_6248.torque[1] - 0.5f * REAR_IMU_PITCH_DAMPING * deg(30)) < 1e-5f);
    run(12015, deg(60), deg(40), 0, 0, 0, true, true, false, true, true, 30);
    assert(std::fabs(rear_6248.torque[0] + REAR_IMU_PITCH_DAMPING * deg(30)) < 1e-5f);
    // Retraction keeps its existing targets, ramp, Kd and encoder direction convention.
    run(14000, deg(60), deg(40), 0, 0, 0, false);
    run(14001, deg(60), deg(40), 0, 0, 0, true, true, true, true, true, 30);
    run(16001, deg(60), deg(40), 0, 0, 0, true, true, true, true, true, 30);
    assert(rear_6248.torque[0] > 0 && rear_6248.torque[1] < 0);
    assert(std::fabs(up_stair_behind_motor_fsm.Get_Position_Target_Angle(1) - Rear::RETRACT_TARGET_RAD[0]) < 1e-5f);
    assert(std::fabs(up_stair_behind_motor_fsm.Get_Position_Target_Angle(2) - Rear::RETRACT_TARGET_RAD[1]) < 1e-5f);
    // The sampled torque cap must use the actual retract Kd at valid shaft speeds.
    run(16002, deg(60), deg(40), 0, -10, 10, true, true, true, true, true, 30);
    assert(std::fabs(rear_6248.torque[0] - 30) < 1e-5f);
    assert(std::fabs(rear_6248.torque[1] + 30) < 1e-5f);
    // Arrival retains feedforward; leaving retract control clears it.
    run(16003, Rear::RETRACT_TARGET_RAD[0], Rear::RETRACT_TARGET_RAD[1],
        0, 0, 0, true, true, true, true, true, 30);
    assert(up_stair_behind_motor_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD);
    assert(std::fabs(rear_6248.torque[0] - TEST_RETRACT_FF) < 1e-5f);
    assert(std::fabs(rear_6248.torque[1] + TEST_RETRACT_FF) < 1e-5f);
    run(16004, deg(60), deg(40), 0);
    run(16005, deg(60), deg(40), 0, 0, 0, false);

    // User confirmed the original Roll polarity: left-high needs positive shaft torque on both sides.
    // Actual magnitudes are independent of MIT Kd; the correction is torque feedforward.
    const float saved_kd = REAR_IMU_MIT_KD;
    for (float kd : {1.5f, 3.5f, 4.5f}) {
        REAR_IMU_MIT_KD = kd;
        for (float deviation : {-20.0f, -5.0f, -0.05f, 0.0f, 0.05f, 5.0f, 20.0f}) {
            filtered_roll_deg = REAR_IMU_ROLL_TARGET_DEG + deviation;
            run(20000, deg(60), deg(40), 0);
            const float expected = deviation == 0 ? 0 : std::copysign(
                std::min(REAR_IMU_ROLL_KP * std::fabs(deviation), REAR_IMU_ROLL_MAX_TORQUE), deviation);
            for (unsigned i : {0U, 1U}) {
                assert(std::fabs(rear_6248.torque[i] - expected) < 1e-5f);
                assert(std::fabs(rear_6248.target_velocity[i]) < 1e-5f);
            }
        }
    }
    REAR_IMU_MIT_KD = saved_kd;
    const float saved_roll_kp = REAR_IMU_ROLL_KP;
    REAR_IMU_ROLL_KP = 0;
    up_stair_behind_motor_fsm = Rear(test_config());
    run(20001, deg(60), deg(40), 0);
    assert(std::fabs(rear_6248.torque[0]) < 1e-5f && std::fabs(rear_6248.torque[1]) < 1e-5f);
    REAR_IMU_ROLL_KP = saved_roll_kp;
    up_stair_behind_motor_fsm = Rear(test_config());

    // Combined Pitch damping and saturated Roll still obey every endpoint guard.
    for (float roll : {-30.0f, 30.0f}) {
        filtered_roll_deg = REAR_IMU_ROLL_TARGET_DEG + roll;
        for (float rate : {-30.0f, 0.0f, 30.0f}) {
            for (float speed : {-10.0f, 0.0f, 10.0f}) {
                run(20002, Rear::ANGLE_START_RAD[0], Rear::ANGLE_START_RAD[1], 0,
                    speed, speed, true, true, false, true, true, rate);
                assert(rear_6248.torque[0] >= -1e-5f && rear_6248.torque[1] >= -1e-5f);
                run(20002, Rear::ANGLE_END_RAD[0], Rear::ANGLE_END_RAD[1], 0,
                    speed, speed, true, true, false, true, true, rate);
                assert(rear_6248.torque[0] <= 1e-5f && rear_6248.torque[1] <= 1e-5f);
            }
        }
    }
    // Roll participates in the existing fault ramp exactly once.
    filtered_roll_deg = REAR_IMU_ROLL_TARGET_DEG + 20;
    const float roll_effort = std::min(REAR_IMU_ROLL_KP * 20, REAR_IMU_ROLL_MAX_TORQUE);
    run(21000, deg(60), deg(40), 0, 0, 0, true, false);
    assert(rear_6248.torque[0] == 0 && rear_6248.torque[1] == 0);
    run(21001, deg(60), deg(40), 0);
    assert(std::fabs(rear_6248.torque[0]) < 1e-5f);
    run(22001, deg(60), deg(40), 0);
    assert(std::fabs(rear_6248.torque[0] - 0.5f * roll_effort) < 1e-5f);
    assert(std::fabs(rear_6248.torque[1] - 0.5f * roll_effort) < 1e-5f);
    run(23001, deg(60), deg(40), 0);
    assert(std::fabs(rear_6248.torque[0] - roll_effort) < 1e-5f);
    // Retraction must ignore Roll, even with a large change in its measurement.
    run(24000, deg(60), deg(40), 0, 0, 0, true, true, true);
    run(25000, deg(60), deg(40), 0, 0, 0, true, true, true);
    const float retract_left = rear_6248.torque[0], retract_right = rear_6248.torque[1];
    filtered_roll_deg = REAR_IMU_ROLL_TARGET_DEG - 20;
    run(25000, deg(60), deg(40), 0, 0, 0, true, true, true);
    assert(std::fabs(rear_6248.torque[0] - retract_left) < 1e-5f);
    assert(std::fabs(rear_6248.torque[1] - retract_right) < 1e-5f);
    assert(rear_6248.sent[0] == rear_6248.sent[1]);
}
'''
with tempfile.TemporaryDirectory(prefix="rear-soft-limit-task-") as directory:
    work = Path(directory)
    test = work / "test.cpp"
    exe = work / "test.exe"
    distinct_kd = ("float REAR_IMU_MIT_KD = 1.5f;\nfloat REAR_RETRACT_MIT_KD = 4.5f;\n"
                   "float REAR_IMU_PITCH_DAMPING = 0.7f;")
    zero_damping = re.sub(r"float REAR_IMU_PITCH_DAMPING\s*=\s*[^;]+;",
                          "float REAR_IMU_PITCH_DAMPING = 0.0f;", kd_declaration)
    for declarations, feedforward in ((kd_declaration, "0"), (distinct_kd, "4"), (zero_damping, "0")):
        test.write_text(declarations + "\n" + host + pid_declarations + wrapper + branch + main, encoding="utf-8")
        subprocess.run([
            "g++", "-std=c++17", f"-DTEST_RETRACT_FF={feedforward}", "-I", ROOT.as_posix(), test.as_posix(),
            (ROOT / "fsm/up_stair_behind_motor_fsm.cpp").as_posix(),
            (ROOT / "user/core/Alg/PID/pid.cpp").as_posix(),
            (ROOT / "user/core/Alg/FSM/alg_fsm.cpp").as_posix(),
            (ROOT / "user/core/Alg/UtilityFunction/SlopPlanning.cpp").as_posix(),
            "-o", exe.as_posix(),
        ], check=True)
        subprocess.run([str(exe)], check=True)
print("PASS: rear task IMU damping, recovery, independent Kd, retract feedforward, torque cap, endpoints and fault zero outputs")
