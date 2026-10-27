"""Check the actual starter tuning, without replacing PID gains with fixtures."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
task = (root / "RtosTask/gantry_task.cpp").read_text(encoding="utf-8")
header = (root / "feeder_fsm/gantry_fsm.hpp").read_text(encoding="utf-8")
assert re.search(r"static constexpr bool CALIBRATED\s*=\s*true;", header), "bench profile must be enabled"
pids = "\n".join(re.search(
    r"ALG::PID::PID " + name + r"\[\d+\]\s*=\s*\{.*?\n\};", task, re.S
).group() for name in ("lift_move_pid", "lift_hold_pid", "hold_angle_pid"))
code = r'''
#include "gantry_fsm.hpp"
#include "pid.hpp"
#include <cassert>
#include <cmath>
PID_DEFINITIONS
int main() {
    using namespace GantryTuning;
    assert(Class_Gantry_FSM::ConfigValid() == bool(EXPECT_CALIBRATED));
    assert(DM_MOVE_KD > 0 && DM_MOVE_KD <= 5);
    assert(DM_HOLD_KD > 0 && DM_HOLD_KD <= 5);
    assert(DM_TORQUE_LIMIT_NM > 0 && DM_TORQUE_LIMIT_NM <= .2f);
    // Preserve physical travel/speed/tolerance when moving from rotor to 19:1 output units.
    assert(std::abs((LIFT_ID5_FIRST_DEG-LIFT_ID5_HOME_DEG)*19-360)<1e-6);
    assert(std::abs((LIFT_ID6_SECOND_DEG-LIFT_ID6_HOME_DEG)*19-720)<1e-6);
    assert(std::abs(LIFT_ID5_SPEED_RAD_S*19-3)<1e-6);
    assert(std::abs(LIFT_ID6_SPEED_RAD_S*19-3)<1e-6);
    assert(std::abs(LOWER_STALL_SPEED_RAD_S*19-.4f)<1e-6);
    assert(std::abs(LIFT_HOME_TOLERANCE_DEG*19-10)<1e-6);
    // The new speed loops must produce the same currents, including integral limiting.
    for (unsigned i=0; i<2; ++i) {
        ALG::PID::PID rotor(100, .2f, 0, 409.6f, 1000, 0);
        for (auto *pid : {&lift_move_pid[i], &lift_hold_pid[i]}) {
            rotor.reset(); pid->reset();
            for (unsigned j=0; j<1800; ++j) {
                const float target=j<600 ? 3 : j<1200 ? -3 : .2f;
                const float feedback=j%2 ? .1f : -.1f;
                assert(std::abs(pid->UpDate(target/19, feedback/19)-
                                rotor.UpDate(target, feedback))<.01f);
            }
            pid->reset();
        }
        ALG::PID::PID rotor_angle(2, .001f, 0, 3, 100, 0);
        rotor_angle.reset(); hold_angle_pid[i].reset();
        for (unsigned j=0; j<1800; ++j) {
            const float error=j<600 ? .8f : j<1200 ? -.8f : .2f;
            assert(std::abs(hold_angle_pid[i].UpDate(error/19,0)*19-
                            rotor_angle.UpDate(error,0))<.001f);
        }
        hold_angle_pid[i].reset();
    }
    if (EXPECT_CALIBRATED) {
        Class_Gantry_FSM f;
        auto step = [&](bool right, bool first, bool extend, bool second, bool retract,
                        double lift, double dm, unsigned tick) {
            f.Update(true, true, true, right, right, first, extend, second, retract,
                     lift, lift, dm, 0, 0, 0, 0, tick);
        };
        step(false,false,false,false,true,0,0,0);
        step(true,false,false,false,true,0,0,5);
        assert(f.GetState()==GantryState::LiftFirst);
        assert(f.GetTargetSpeed(0)>0 && f.GetTargetSpeed(1)>0);
        step(true,true,false,false,false,360.0/19,0,10);
        assert(f.GetState()==GantryState::Extend);
        assert(f.GetTargetSpeed(2)>0);
        step(true,false,true,false,false,360.0/19,30,15);
        assert(f.GetState()==GantryState::LiftSecond);
        step(true,false,false,true,false,720.0/19,30,20);
        assert(f.GetState()==GantryState::Retract);
        assert(f.GetTargetSpeed(2)<0);
        step(true,false,false,false,true,720.0/19,0,25);
        assert(f.GetState()==GantryState::Lower);
        assert(f.GetTargetSpeed(0)<0 && f.GetTargetSpeed(1)<0);
        // With no mechanical stop, zero current must not falsely confirm home.
        step(true,false,false,false,true,0,0,30);
        assert(f.GetState()==GantryState::Lower);
        step(true,false,false,false,true,-100.0/19,0,35);
        assert(f.GetState()==GantryState::Fault && f.GetFault()==GantryFault::Travel);
    }
    for (unsigned i=0; i<2; ++i) {
        const float speed = i == 0 ? LIFT_ID5_SPEED_RAD_S : LIFT_ID6_SPEED_RAD_S;
        for (auto *pid : {&lift_move_pid[i], &lift_hold_pid[i]}) {
            assert(pid->UpDate(speed, 0) > 0);
            pid->reset();
            assert(pid->UpDate(-speed, 0) < 0);
            pid->reset();
            for (unsigned j=0; j<1000; ++j)
                assert(std::abs(pid->UpDate(1000, 0)) <= .5f * 16384 / 20);
            pid->reset();
            assert(pid->UpDate(0, 0) == 0);
        }
    }
    for (auto &pid : hold_angle_pid) {
        assert(pid.UpDate(.1f, 0) > 0);
        pid.reset();
        assert(pid.UpDate(-.1f, 0) < 0);
        pid.reset();
        assert(pid.UpDate(0, 0) == 0);
    }
}
'''.replace("PID_DEFINITIONS", pids).replace(
    '#include <cmath>', '#include <cmath>\n#include <initializer_list>')
with tempfile.TemporaryDirectory(prefix="gantry_tuning_") as directory:
    work = Path(directory)
    (work / "test.cpp").write_text(code, encoding="utf-8")
    (work / "gantry_fsm.cpp").write_text(
        (root / "feeder_fsm/gantry_fsm.cpp").read_text(encoding="utf-8"), encoding="utf-8")
    for calibrated in (False, True):
        fixture = re.sub(r"(static constexpr bool CALIBRATED\s*=\s*)[^;]+;",
                         lambda m: m[1] + str(calibrated).lower() + ";", header)
        (work / "gantry_fsm.hpp").write_text(fixture, encoding="utf-8")
        exe = work / "test.exe"
        subprocess.run([
            "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
            "-DEXPECT_CALIBRATED=" + str(int(calibrated)),
            "-I", work.as_posix(), "-I", (root / "user/core/Alg/PID").as_posix(),
            (work / "test.cpp").as_posix(), (work / "gantry_fsm.cpp").as_posix(),
            (root / "user/core/Alg/PID/pid.cpp").as_posix(), "-o", exe.as_posix()
        ], check=True)
        subprocess.run([str(exe)], check=True)
        print(f"PASS: actual starter tuning, calibration gate={calibrated}")
