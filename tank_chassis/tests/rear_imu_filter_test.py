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
timeout_header = (ROOT / "RtosTask/remote_task.hpp").read_text(encoding="utf-8")
timeout_declaration = re.search(
    r"constexpr uint32_t IMU_CONTROL_TIMEOUT_MS\s*=\s*\d+U;", timeout_header).group(0)
declarations = timeout_declaration + "\n" + "\n".join(re.findall(
    r"(?:constexpr float REAR_(?:PITCH|PITCH_RATE)_FILTER_(?:RATIO|TIME_CONSTANT_MS)\s*=\s*[^;]+;"
    r"|LPFFilter rear_6248_(?:pitch(?:_rate)?|roll)_filter\([^;]*\);)", source))

HOST = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include "user/core/Alg/Filter/Filter.hpp"
#include "fsm/up_stair_behind_motor_fsm.hpp"
struct Snapshot { bool valid; uint32_t tick; float pitch_deg, pitch_rate_dps; float roll_deg = 1.5f; };
float observed_roll;
Class_Up_Stair_Behind_Motor_FSM up_stair_behind_motor_fsm(
    [] {
        auto config = Class_Up_Stair_Behind_Motor_FSM::Config::WithAngleConstants();
        config.pitch_zero_deg = 0; // Isolate filter behavior from sensor calibration.
        return config;
    }());
bool rear_imu_filter_initialized = false;
uint32_t rear_imu_filter_tick = 0U;
float vofa_channels[6] = {};
unsigned vofa_frames = 0;
void vofa_send(float raw_pitch, float filtered_pitch, float raw_rate,
               float filtered_rate, float sample_time, float reserved) {
    vofa_channels[0] = raw_pitch;
    vofa_channels[1] = filtered_pitch;
    vofa_channels[2] = raw_rate;
    vofa_channels[3] = filtered_rate;
    vofa_channels[4] = sample_time;
    vofa_channels[5] = reserved;
    ++vofa_frames;
}
'''
WRAPPER = r'''
void update(uint32_t now_tick, const Snapshot &imu_snapshot, bool rear_control_enabled = true) {
    struct { bool rear_retract_requested = false; } command;
    const bool rear_left_online = true, rear_right_online = true;
    const float rear_left_angle = 1.0f, rear_right_angle = 1.0f;
    const float rear_left_velocity = 0.0f, rear_right_velocity = 0.0f;
'''
MAIN = r'''
    observed_roll = filtered_roll_deg;
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
    assert(vofa_frames == 1);
    assert(near(vofa_channels[0], 5) && near(vofa_channels[1], 5));
    assert(near(vofa_channels[2], 2) && near(vofa_channels[3], 2));
    assert(near(vofa_channels[4], 0.100f) && vofa_channels[5] == 0);
    // Each new frame applies the fixed angle/rate weights independently.
    update(110, {true, 110, 15, 12});
    assert(near(pitch(), 7.0f));
    assert(near(rate(), 4.5f));
    assert(vofa_frames == 2);
    assert(near(vofa_channels[0], 15) && near(vofa_channels[1], 7.0f));
    assert(near(vofa_channels[2], 12) && near(vofa_channels[3], 4.5f));
    assert(near(vofa_channels[4], 0.110f));
    const float p = pitch(), r = rate();
    update(111, {true, 110, 15, 12});
    assert(near(pitch(), p) && near(rate(), r));
    assert(vofa_frames == 2);
    update(120, {true, 120, 15, 12});
    assert(near(pitch(), p + (15-p)*0.20f));
    assert(near(rate(), r + (12-r)*0.25f));

    // Repeated samples remain valid at 20/99 ms; expire at exactly 100 ms.
    update(140, {true, 120, 15, 12});
    assert(up_stair_behind_motor_fsm.Uses_Attitude_Control());
    update(219, {true, 120, 15, 12});
    assert(up_stair_behind_motor_fsm.Uses_Attitude_Control());
    const unsigned before_expiry = vofa_frames;
    update(220, {true, 120, 15, 12});
    assert(vofa_frames == before_expiry);
    assert(up_stair_behind_motor_fsm.Get_State() ==
           UP_STAIR_BEHIND_MOTOR_DISABLED);
    assert(!up_stair_behind_motor_fsm.Uses_Position_Control());
    assert(up_stair_behind_motor_fsm.Limit_Torque(1U, 20.0f) == 0.0f);
    assert(up_stair_behind_motor_fsm.Limit_Torque(2U, -20.0f) == 0.0f);
    update(221, {true, 221, 9, -5});
    assert(near(pitch(), 9) && near(rate(), -5));
    // Neither NaN channel may contaminate subsequent valid filter state.
    const unsigned before_nan = vofa_frames;
    update(222, {true, 222, NAN, 1});
    assert(vofa_frames == before_nan);
    assert(!up_stair_behind_motor_fsm.Uses_Attitude_Control());
    update(223, {true, 223, -3, 4});
    assert(near(pitch(), -3) && near(rate(), 4));
    update(224, {true, 224, 1, NAN});
    assert(!up_stair_behind_motor_fsm.Uses_Attitude_Control());
    update(225, {true, 225, 2, -6});
    assert(near(pitch(), 2) && near(rate(), -6));

    // NORMAL disables the rear FSM and clears history even with a valid IMU.
    const unsigned before_normal = vofa_frames;
    update(230, {true, 230, 30, 10}, false);
    assert(vofa_frames == before_normal);
    assert(!rear_imu_filter_initialized);
    assert(up_stair_behind_motor_fsm.Limit_Torque(1U, 20.0f) == 0.0f);
    update(231, {true, 231, -10, -4});
    assert(near(pitch(), -10) && near(rate(), -4));

    // Frame spacing does not change weights; tick rollover still accepts new frames.
    reseed(300, 0, 0);
    update(305, {true, 305, 10, 10});
    assert(near(pitch(), 2.0f) && near(rate(), 2.5f));
    reseed(0xfffffff0U, 0, 0);
    update(0xfffffffaU, {true, 0xfffffffaU, 10, 10});
    const float before_wrap_p = pitch(), before_wrap_r = rate();
    update(4, {true, 4, 10, 10});
    assert(near(pitch(), before_wrap_p + (10-before_wrap_p)*0.20f));
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
    // At this test's 100 Hz frame rate, alpha=0.20 retains about 11.6% of 10 Hz energy.
    assert(pitch_energy < 0.13f*input_energy);
    assert(rate_energy < 0.423f*input_energy);

    // Roll has the same once-per-frame filter and recovery semantics as Pitch.
    update(9000, {false, 9000, 0, 0});
    update(9001, {true, 9001, 0, 0, 1.5f});
    assert(near(observed_roll, 1.5f));
    update(9006, {true, 9006, 0, 0, 11.5f});
    assert(near(observed_roll, 2.4f));
    update(9007, {true, 9006, 0, 0, 11.5f});
    assert(near(observed_roll, 2.4f));
    update(9011, {true, 9011, 0, 0, NAN});
    assert(!up_stair_behind_motor_fsm.Uses_Attitude_Control());
    assert(!rear_imu_filter_initialized);
    update(9016, {true, 9016, 0, 0, -3.5f});
    assert(near(observed_roll, -3.5f));
    update(9116, {true, 9016, 0, 0, -3.5f});
    assert(!rear_imu_filter_initialized);
    update(9117, {true, 9117, 0, 0, 6.5f});
    assert(near(observed_roll, 6.5f));
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
print("PASS: raw/filtered IMU VOFA channels, new-frame sends, fixed weights, initialization, expiry, NaN recovery and tick rollover")
