"""Verify all nine switch combinations use one mode and have no gyro state."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
header = (ROOT / "fsm/chassis_fsm.hpp").read_text(encoding="utf-8-sig")
source = (ROOT / "fsm/chassis_fsm.cpp").read_text(encoding="utf-8-sig")
assert "rotation_mode" not in header + source, "use one current mode for switch selection and yaw output"
assert "CHASSIS_GYRO_SPIN" not in header + source, "remove the gyro state and its delay"
assert header.count("void StateUpdate(") == 1, "use one switch update function"
assert header.count("float Get_wz_cmd(") == 1, "use one yaw output function"
harness = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include "fsm/chassis_fsm.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

uint32_t tick = 100;
uint32_t HAL_GetTick() { return tick; }
constexpr float MAX_WZ = 2.5f;
static_assert(CHASSIS_STOP == 0 && CHASSIS_FOLLOW_GIMBAL == 1 && CHASSIS_NOT_FOLLOW == 2);
static_assert(CHASSIS_MODE_COUNT == 5 && CHASSIS_MODE_COUNT <= STATUS_MAX);

Enum_Chassis_Mode expected_mode(uint8_t s1, uint8_t s2, bool online, bool keyboard_valid) {
    if (!online || s1 < 1 || s1 > 3 || s2 < 1 || s2 > 3 || (s1 == 2 && s2 == 2))
        return CHASSIS_STOP;
    if (s1 == 3 && s2 == 3)
        return keyboard_valid ? CHASSIS_MOUSE_ROTATE : CHASSIS_STOP;
    return CHASSIS_JOYSTICK_ROTATE;
}

void near(float actual, float expected) {
    assert(std::fabs(actual - expected) < 0.00001f);
}

int main() {
    Chassis_FSM fsm(8, 0, 0, 100, 25, 2.5f);
    fsm.Init();
    assert(fsm.Get_Mode() == CHASSIS_STOP);
    for (bool online : {false, true}) {
        for (uint8_t s1 = 0; s1 <= 4; ++s1) {
            for (uint8_t s2 = 0; s2 <= 4; ++s2) {
                for (bool keyboard_valid : {false, true}) {
                    const auto mode = expected_mode(s1, s2, online, keyboard_valid);
                    fsm.StateUpdate(s1, s2, online, keyboard_valid);
                    assert(fsm.Get_Mode() == mode && fsm.Get_Now_Status_Serial() == mode);
                    for (float stick : {-1.f, -.5f, -.0015f, 0.f, .12345678f, .5f, 1.f}) {
                        for (int16_t mouse : {int16_t(-32768), int16_t(-1), int16_t(0),
                                              int16_t(1), int16_t(32767)}) {
                            const float expected = mode == CHASSIS_STOP ? 0 :
                                mode == CHASSIS_MOUSE_ROTATE ?
                                    (std::abs(mouse) <= 6 ? 0.0f :
                                     -static_cast<float>(mouse) * (1.0f * M_PI / 180.0f)) :
                                -stick * MAX_WZ;
                            assert(fsm.Get_wz_cmd(.2f, stick, mouse, MAX_WZ) == expected);
                        }
                    }
                }
            }
        }
    }
    // The dead zone is inclusive and larger inputs retain their conversion.
    fsm.StateUpdate(3, 3, true, true);
    for (int count = -32768; count <= 32767; ++count) {
        const float expected = std::abs(count) <= 6 ? 0.0f :
            -static_cast<float>(count) * (1.0f * M_PI / 180.0f);
        const float actual = fsm.Get_wz_cmd(0, 1, static_cast<int16_t>(count), MAX_WZ);
        assert(actual == expected);
        assert(std::clamp(actual, -MAX_WZ, MAX_WZ) == std::clamp(expected, -MAX_WZ, MAX_WZ));
    }
    // Keyboard unavailable/expired: no joystick fallback in double-middle mode.
    fsm.StateUpdate(3, 3, true, false);
    assert(fsm.Get_Mode() == CHASSIS_STOP);
    assert(fsm.Get_wz_cmd(0, 1, 32767, MAX_WZ) == 0);
    // STOP wins even if stale manual inputs remain nonzero.
    fsm.StateUpdate(2, 2, true, true);
    assert(fsm.Get_Mode() == CHASSIS_STOP);
    assert(fsm.Get_wz_cmd(0, 1, 32767, MAX_WZ) == 0);
    fsm.StateUpdate(3, 3, false, true);
    assert(fsm.Get_wz_cmd(0, 1, 32767, MAX_WZ) == 0);
    fsm.StateUpdate(1, 1, true, false);
    fsm.TIM_Calculate_PeriodElapsedCallback();
    fsm.StateUpdate(1, 2, true, false);
    assert(fsm.Status[CHASSIS_JOYSTICK_ROTATE].Count_Time == 1);
    near(fsm.Get_wz_cmd(0, .5f, 32767, MAX_WZ), -1.25f);
    fsm.StateUpdate(3, 3, true, true);
    assert(fsm.Status[CHASSIS_JOYSTICK_ROTATE].Count_Time == 0);
    near(fsm.Get_wz_cmd(.2f, 1, 10, MAX_WZ), -10.f * M_PI / 180.f);
    near(fsm.Get_wz_cmd(.2f, 1, 10, MAX_WZ, true), 0);
    // Retain the existing follow PID without selecting it in the switch map.
    near(fsm.Get_follow_wz_cmd(.2f), 1.6f);
    near(fsm.Get_follow_wz_cmd(.02f), 0);
    near(fsm.Get_follow_wz_cmd(1.f), 4.f);
    fsm.Init();
    assert(fsm.Get_Mode() == CHASSIS_STOP);
}
'''
with tempfile.TemporaryDirectory(prefix="chassis-rotation-fsm-") as directory:
    work = Path(directory)
    (work / "main.h").write_text(
        '#pragma once\n#include <cstdint>\nuint32_t HAL_GetTick();\n', encoding="utf-8")
    cpp, exe = work / "test.cpp", work / "test.exe"
    cpp.write_text(harness, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-I", work.as_posix(), "-I", ROOT.as_posix(),
                    "-I", (ROOT / "user/core").as_posix(), cpp.as_posix(),
                    (ROOT / "fsm/chassis_fsm.cpp").as_posix(),
                    (ROOT / "user/core/Alg/FSM/alg_fsm.cpp").as_posix(),
                    (ROOT / "user/core/Alg/PID/pid.cpp").as_posix(),
                    "-o", exe.as_posix()], check=True)
    subprocess.run([str(exe)], check=True)
print("PASS: nine switch combinations, single mode, all mouse values, keyboard/STOP gates and retained follow PID")
