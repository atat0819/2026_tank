"""Exercise the task's IMU preprocessing with the real LPF and rear FSM."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "RtosTask/up_stair.cpp").read_text(encoding="utf-8")
start = source.index("        const bool imu_valid = imu_snapshot.valid")
end = source.index("        // 前左反馈从离线恢复", start)
preprocess = source[start:end]
declarations = "\n".join(re.findall(
    r"(?:constexpr float REAR_(?:PITCH|PITCH_RATE)_FILTER_(?:RATIO|TIME_CONSTANT_MS)\s*=\s*[^;]+;"
    r"|LPFFilter rear_6248_pitch(?:_rate)?_filter\([^;]*\);)", source))

HOST = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include "user/core/Alg/Filter/Filter.hpp"
#include "fsm/up_stair_behind_motor_fsm.hpp"
struct Snapshot { bool valid; uint32_t tick; float pitch_deg, pitch_rate_dps; };
Class_Up_Stair_Behind_Motor_FSM up_stair_behind_motor_fsm(
    Class_Up_Stair_Behind_Motor_FSM::Config::WithAngleConstants());
bool rear_imu_filter_initialized = false;
uint32_t rear_imu_filter_tick = 0U;
'''
WRAPPER = r'''
void update(uint32_t now_tick, const Snapshot &imu_snapshot) {
    struct { bool rear_attitude_enabled = true;
             bool rear_retract_command_enabled = false; } policy;
    const bool rear_left_online = true, rear_right_online = true;
    const float rear_left_angle = 0.0f, rear_right_angle = 0.0f;
    const uint32_t rear_action_sequence = 0U;
'''
MAIN = r'''
}
float pitch() { return up_stair_behind_motor_fsm.Get_Feedback_Pitch_Deg(); }
float rate() { return up_stair_behind_motor_fsm.Get_Pitch_Rate_Dps(); }
bool near(float a, float b) { return std::fabs(a-b) < 1e-4f; }
void reseed(uint32_t tick, float angle, float speed) {
    update(tick, {false, tick, 0, 0});
    update(tick, {true, tick, angle, speed});
    assert(near(pitch(), angle) && near(rate(), speed));
}
int main() {
    // First valid data must not ramp from an invented zero attitude.
    update(100, {true, 100, 5, 2});
    assert(near(pitch(), 5) && near(rate(), 2));
    // Each new frame applies the fixed angle/rate weights independently.
    update(110, {true, 110, 15, 12});
    assert(near(pitch(), 5.9f));
    assert(near(rate(), 4.5f));
    const float p = pitch(), r = rate();
    update(111, {true, 110, 15, 12});
    assert(near(pitch(), p) && near(rate(), r));
    update(120, {true, 120, 15, 12});
    assert(near(pitch(), p + (15-p)*0.09f));
    assert(near(rate(), r + (12-r)*0.25f));

    // Exactly 20 ms old must retain the existing basic-angle fallback.
    update(140, {true, 120, 15, 12});
    assert(up_stair_behind_motor_fsm.Get_State() ==
           UP_STAIR_BEHIND_MOTOR_BASIC_ANGLE_CONTROL);
    update(141, {true, 141, 9, -5});
    assert(near(pitch(), 9) && near(rate(), -5));
    // Neither NaN channel may contaminate subsequent valid filter state.
    update(142, {true, 142, NAN, 1});
    assert(!up_stair_behind_motor_fsm.Uses_Attitude_Control());
    update(143, {true, 143, -3, 4});
    assert(near(pitch(), -3) && near(rate(), 4));
    update(144, {true, 144, 1, NAN});
    assert(!up_stair_behind_motor_fsm.Uses_Attitude_Control());
    update(145, {true, 145, 2, -6});
    assert(near(pitch(), 2) && near(rate(), -6));

    // Frame spacing does not change weights; tick rollover still accepts new frames.
    reseed(200, 0, 0);
    update(205, {true, 205, 10, 10});
    assert(near(pitch(), 0.9f) && near(rate(), 2.5f));
    reseed(0xfffffff0U, 0, 0);
    update(0xfffffffaU, {true, 0xfffffffaU, 10, 10});
    const float before_wrap_p = pitch(), before_wrap_r = rate();
    update(4, {true, 4, 10, 10});
    assert(near(pitch(), before_wrap_p + (10-before_wrap_p)*0.09f));
    assert(near(rate(), before_wrap_r + (10-before_wrap_r)*0.25f));

    // A sustained tilt must still pass through, while fast vibration shrinks.
    reseed(1000, 0, 0);
    for (uint32_t t=1010; t<=3010; t+=10) update(t, {true,t,10,4});
    assert(near(pitch(), 10) && near(rate(), 4));
    reseed(4000, 0, 0);
    float input_energy=0, pitch_energy=0, rate_energy=0;
    for (uint32_t i=1; i<=400; ++i) {
        const uint32_t t=4000+i*10;
        const float input=std::sin(6.28318530718f*10*i*0.01f);
        update(t, {true,t,input,input});
        if (i>100) {
            input_energy+=input*input;
            pitch_energy+=pitch()*pitch(); rate_energy+=rate()*rate();
        }
    }
    assert(pitch_energy < 0.09f*input_energy);
    assert(rate_energy < 0.423f*input_energy);
}
'''
with tempfile.TemporaryDirectory(prefix="rear-imu-filter-") as directory:
    work = Path(directory)
    test = work / "test.cpp"
    test.write_text(HOST + declarations + WRAPPER + preprocess + MAIN, encoding="utf-8")
    executable = work / "test.exe"
    subprocess.run([
        "g++", "-std=c++17", "-I", ROOT.as_posix(), test.as_posix(),
        (ROOT / "user/core/Alg/Filter/Filter.cpp").as_posix(),
        (ROOT / "fsm/up_stair_behind_motor_fsm.cpp").as_posix(),
        (ROOT / "user/core/Alg/FSM/alg_fsm.cpp").as_posix(),
        (ROOT / "user/core/Alg/UtilityFunction/SlopPlanning.cpp").as_posix(),
        "-o", executable.as_posix(),
    ], check=True)
    subprocess.run([str(executable)], check=True)
print("PASS: fixed rear IMU weights, new-frame updates, initialization, expiry, NaN recovery and tick rollover")
