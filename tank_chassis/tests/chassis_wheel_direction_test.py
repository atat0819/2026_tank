"""Exercise the task's wheel targets and PID with the real IK and a speed plant."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
targets = re.search(r"motor_target_speed\[0\] = .*?motor_target_speed\[3\] = [^;]+;",
                    source, re.S).group(0)
pid = re.search(r"motor_output\[0\] = motor_pid.*?motor_output\[3\] = [^;]+;",
                source, re.S).group(0)
mouse = re.search(r"wz_cmd = -static_cast<float>\(input.mouse_x\).*?"
                  r"if \(wz_cmd < [^\n]+", source, re.S).group(0)
geometry = "\n".join(re.search(r"const float " + name + r"[^;]+;", source).group(0)
                     for name in [r"wheel_azimuth\[4\]", r"wheel_coords\[4\]\[2\]"])
ik_declaration = re.search(r"Alg::CalculationBase::Omni_IK ik\([^;]+;", source).group(0)
host = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include "user/core/Alg/ChassisCalculation/OmniCalculation.hpp"
#include "user/core/Alg/PID/pid.hpp"
float mouse_yaw(short count) {
    const float M_PI = 3.14159265358979323846f;
    struct { int16_t mouse_x; } input = {count};
    float wz_cmd = 0;
'''
host += mouse + r'''
    return wz_cmd;
}
int main() {
    assert(mouse_yaw(0) == 0);
    assert(std::fabs(mouse_yaw(1) + 2.0f * 3.14159265358979323846f / 180.0f) < 1e-6f);
    assert(std::fabs(mouse_yaw(-1) - 2.0f * 3.14159265358979323846f / 180.0f) < 1e-6f);
    assert(mouse_yaw(200) == -4.0f && mouse_yaw(-200) == 4.0f);
    const float M_PI = 3.14159265358979323846f;
'''
host += geometry + "\n" + ik_declaration + r'''
    // Preserve measured translation; only yaw on IDs 2/4 changes.
    const float forward_direction[4] = {-1, 1, 1, -1};
    const float lateral_direction[4] = {-1, -1, 1, 1};
    const float commands[][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},
                                  {0,0,1},{0,0,-1},{0,0,4},{0,0,-4},
                                  {.4f,-.7f,.2f},{0,0,0}};
    for (const auto &command : commands) {
        ALG::PID::PID motor_pid[4] = {{1,0,0,16384,0,0},{1,0,0,16384,0,0},
                                     {1,0,0,16384,0,0},{1,0,0,16384,0,0}};
        float motor_target_speed[4], motor_output[4], current_speed_rads[4] = {};
        ik.OmniInvKinematics(command[0], command[1], command[2], 0, 1, 1);
        for (unsigned step = 0; step < 100; ++step) {
'''
host += targets + "\n" + pid + r'''
            for (unsigned i = 0; i < 4; ++i)
                current_speed_rads[i] += .2f * motor_output[i];
        }
        for (unsigned i = 0; i < 4; ++i) {
            // Independent vehicle mapping: 1 LF, 2 RF, 3 RR, 4 LR.
            // For positive yaw, all four motor-coordinate targets are positive.
            const float expected = (forward_direction[i] * command[0] +
                                    lateral_direction[i] * command[1] +
                                    (.215f + .243f) * command[2]) /
                                   (std::sqrt(2.0f) * .076f);
            assert(std::fabs(current_speed_rads[i] - expected) < 1e-4f);
        }
    }
}
'''
with tempfile.TemporaryDirectory() as temp:
    cpp = Path(temp) / "wheel_direction.cpp"
    exe = Path(temp) / "wheel_direction.exe"
    cpp.write_text(host, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-I", ROOT.as_posix(),
                    "-I", (ROOT / "user").as_posix(), cpp.as_posix(),
                    (ROOT / "user/core/Alg/PID/pid.cpp").as_posix(),
                    "-o", exe.as_posix()], check=True)
    subprocess.run([str(exe)], check=True)
print("PASS: mouse yaw gain/limits and zero; wheel directions and translation/spin tracking")
