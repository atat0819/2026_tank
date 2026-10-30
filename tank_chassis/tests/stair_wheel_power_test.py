"""Run actual task fragments with real command, PID and power-limit code."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
start = source.index("taskENTER_CRITICAL();", source.index("一次 B 同时发布两个明确目标"))
publish = source[start:source.index("taskEXIT_CRITICAL();", start)]
start = source.index("motor_output[0] = motor_pid[0].UpDate")
control = source[start:source.index("// 保存功率控制前", start)].strip()
coefficients = re.search(r"float poly_coeffs\[6\] = \{.*?\};", source, re.S).group(0)
limit_start = source.index("float I[4], V[4], I_other[4];")
limit_end = source.index("// ========== 功率控制结束", limit_start)
limit = source[limit_start:limit_end].rstrip()
assert limit.endswith("}")
limit = limit[:-1]  # Outer CHASSIS_POWER_LIMIT_ENABLED branch is supplied below.
stop_start = source.index("for (int i = 0; i < 4; i++)", source.index("// STOP 模式"))
stop = source[stop_start:source.index("chassis_vx_filter.Reset", stop_start)]
host = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include "fsm/stair_mode_policy.hpp"
#include "user/core/Alg/PID/pid.hpp"
#include "user/core/Alg/PowerControl/PowerControl.hpp"
#define taskENTER_CRITICAL() ((void)0)
struct Keys { bool imu_mode_request=false, normal_mode_request=false,
                   stair_toggle=false, valid=true; };
StairCommand stair_command;
uint32_t stair_command_generation = 0;
ALG::PID::PID motor_pid[4] = {{660,.2f,0,16384,5000,500},
    {660,.2f,0,16384,5000,500}, {660,.2f,0,16384,5000,500},
    {660,.2f,0,16384,5000,500}};
float motor_output[4], motor_output_pre[4], motor_target_speed[4];
float current_speed_rads[4];
struct Motor {
    float velocity[4] = {100,100,100,100};
    float sent[4] = {};
    float getVelocityRads(int id) { return velocity[id-1]; }
    void setCAN(int16_t value, int id) { sent[id-1] = value; }
} chassis_motor;
ALG::PowerControl::PowerControl<4> chassis_power_ctrl;
'''
host += coefficients + r'''
void step(Keys keyboard_cmd={}, bool stopped=false, bool limited=true) {
    const auto stair_sample_generation = stair_command_generation;
    const bool source_changed = false;
    struct { bool zero_all_torque; } stair_policy = {stopped};
'''
host += publish + "\nif (stopped) {\n" + stop + "\nreturn;\n}\n" + control
host += "\nif (limited) {\nconst float PowerMax = 50.0f;\n" + limit + "\n}\n}\n"
host += r'''
void check(bool front_enabled) {
    for (int i=0; i<4; ++i) {
        assert(std::isfinite(motor_output[i]));
        if (i<2 && !front_enabled) assert(motor_output[i] == 0.0f);
        else assert(motor_output[i] * motor_target_speed[i] > 0.0f);
    }
    float power = 0;
    for (int i=0; i<4; ++i) {
        const float a = motor_output[i] * (20.0f / 16384.0f);
        const float w = chassis_motor.velocity[i];
        power += poly_coeffs[0] + poly_coeffs[1]*a + poly_coeffs[2]*fabsf(w)
            + poly_coeffs[3]*a*w + poly_coeffs[4]*a*a + poly_coeffs[5]*w*w;
    }
    assert(power <= 50.001f);
}
int main() {
    for (int i=0; i<4; ++i) motor_target_speed[i] = 100;
    step(); check(true);
    const float normal_rear = motor_output[2];
    step({true,false,false,true}); check(true); // V: initial leveling stays four-wheel drive
    step({false,false,true,true}); check(false); // B: front target, leveling
    assert(motor_output[2] > normal_rear);
    for (int i=0; i<20; ++i) { step(); check(false); } // Rear drive persists until next B
    step({false,false,true,true}); check(true); // B: retract
    for (int i=0; i<20; ++i) { step(); check(true); } // retract hold
    step({false,false,true,true}); check(false); // B: reverse to leveling
    step({false,true,false,true}); check(true); // C: normal
    // Exercise actual forward motor signs and reverse, with moving feedback.
    for (int direction=-1; direction<=1; direction+=2) {
        const float signs[4] = {-1,1,1,-1};
        for (int i=0; i<4; ++i) {
            motor_target_speed[i] = 100 * signs[i] * direction;
            chassis_motor.velocity[i] = 100 * signs[i] * direction;
            current_speed_rads[i] = 20 * signs[i] * direction;
        }
        step({false,true,false,true}); check(true);
        step({true,false,false,true}); check(true);
        step({false,false,true,true}); check(false);
        step({false,false,true,true}); check(true);
    }
    for (int i=0; i<4; ++i) {
        motor_target_speed[i] = chassis_motor.velocity[i] = 100;
        current_speed_rads[i] = 0;
    }
    step({true,false,false,true}, false, false);
    assert(motor_output[0] > 0 && motor_output[1] > 0);
    step({false,false,true,true}, false, false);
    assert(motor_output[0] == 0 && motor_output[1] == 0); // Also gates bypass
    // A freely rotating front wheel must not produce braking current.
    current_speed_rads[0] = current_speed_rads[1] = 200;
    for (int i=0; i<20; ++i) { step(); check(false); }
    current_speed_rads[0] = current_speed_rads[1] = 100;
    step({false,true,false,true}, false, false);
    assert(motor_output[0] == 0 && motor_output[1] == 0); // No stored integral
    current_speed_rads[0] = current_speed_rads[1] = 0;
    step({true,false,false,true}); check(true); // Re-enter V resets stage to four-wheel drive
    step({false,false,true,true}); check(false);
    step({}, true);
    assert(stair_command.mode == StairMode::NORMAL);
    for (int i=0; i<4; ++i) {
        assert(motor_output[i] == 0 && chassis_motor.sent[i] == 0);
        assert(motor_target_speed[i] == 0);
    }
}
'''
with tempfile.TemporaryDirectory(prefix="stair-wheel-power-") as directory:
    cpp = Path(directory) / "test.cpp"
    exe = Path(directory) / "test.exe"
    cpp.write_text(host, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-I", ROOT.as_posix(), "-I", (ROOT / "user").as_posix(),
                    cpp.as_posix(), (ROOT / "user/core/Alg/PID/pid.cpp").as_posix(),
                    "-o", exe.as_posix()], check=True)
    subprocess.run([str(exe)], check=True)
print("PASS: V four-wheel drive, first-B rear drive, second-B retract four-wheel drive, budget, PID reset and STOP")
