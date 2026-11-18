"""Exercise production lift synchronization with both motor installation signs."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
header = (root / "feeder_fsm/gantry_fsm.hpp").read_text(encoding="utf-8")
source = (root / "feeder_fsm/gantry_fsm.cpp").read_text(encoding="utf-8")
test = r'''
#include "gantry_fsm.hpp"
#include <cassert>
#include <cmath>
#include <limits>
using namespace GantryTuning;
struct Rig {
    Class_Gantry_FSM f;
    double a[3] = {5000, -8000, 600};
    double origin[2] = {5000, -8000};
    double sign[2] = {LIFT_ID5_FIRST_DEG > LIFT_ID5_HOME_DEG ? 1.0 : -1.0,
                      LIFT_ID6_FIRST_DEG > LIFT_ID6_HOME_DEG ? 1.0 : -1.0};
    bool limit[5] = {};
    bool key = false, enabled = true, feedback = true;
    unsigned time = 0;
    Rig() { tick(); key = true; tick(); tick(2000); }
    void tick(unsigned dt = 5) {
        time += dt;
        f.Update(enabled, feedback, feedback, key, key,
                 limit[0], limit[1], limit[2], limit[3], limit[4],
                 a[0], a[1], a[2], 0, 0, 0, 0, time);
    }
    void heights(double h5, double h6) {
        a[0] = origin[0] + sign[0] * h5;
        a[1] = origin[1] + sign[1] * h6;
        tick();
    }
    void hit(unsigned i) { limit[i] = true; tick(); limit[i] = false; }
    double speed(unsigned i) { return sign[i] * f.GetTargetSpeed(i); }
    void lower() {
        heights(1000, 1000); hit(0); a[2] -= 500; tick(); hit(1);
        heights(2000, 2000); hit(2); a[2] += 500; tick(); hit(3);
        assert(f.GetState() == GantryState::Lower);
    }
};
int main() {
    const double base = std::fmin(LIFT_ID5_SPEED_RAD_S, LIFT_ID6_SPEED_RAD_S);
    if (LIFT_SYNC_KP == 0 || LIFT_SYNC_MAX_SPEED_RATIO == 0) {
        Rig r; r.heights(104, 100);
        assert(std::abs(r.speed(0) - base) < 1e-6);
        assert(std::abs(r.speed(1) - base) < 1e-6);
        assert(std::abs(r.f.GetLiftSyncErrorDeg() - 4) < 1e-6);
        return 0;
    }
    {
        Rig r; r.heights(100, 100);
        assert(std::abs(r.speed(0) - base) < 1e-6);
        assert(std::abs(r.speed(1) - base) < 1e-6);
        r.heights(104, 100);
        assert(std::abs(r.f.GetLiftSyncErrorDeg() - 4) < 1e-6);
        assert(r.speed(0) < r.speed(1)); // Higher side must slow, including opposite signs.
        assert(std::abs((base - r.speed(0)) - 4 * 3.141592653589793 / 180) < 1e-6);
        assert(r.speed(0) > 0 && r.speed(1) <= base);
        r.heights(100, 104);
        assert(r.speed(1) < r.speed(0));
        assert(r.speed(1) > 0 && r.speed(0) <= base);
    }
    {
        Rig r; r.lower(); r.heights(1904, 1900);
        assert(r.speed(0) < r.speed(1)); // Higher side descends faster.
        assert(r.speed(0) >= -base && r.speed(1) < 0);
        r.heights(1900, 1904);
        assert(r.speed(1) < r.speed(0));
        assert(r.speed(1) >= -base && r.speed(0) < 0);
    }
    {
        Rig r; r.heights(900, 850); // First-lift slow zone; correction must not reverse either side.
        assert(r.f.GetScale() == .1f);
        assert(r.speed(0) > 0 && r.speed(0) < r.speed(1));
        assert(r.speed(1) <= base * .1 + 1e-6);
    }
    {
        Rig r; r.heights(1000, 990); r.hit(0);
        assert(r.f.IsKeeping(0) && r.f.IsKeeping(1));
        assert(std::abs(r.f.GetLiftSyncErrorDeg() - 10) < 1e-6);
        r.a[2] -= 500; r.tick(); r.hit(1);
        assert(r.f.GetState() == GantryState::LiftSecond);
        assert(r.speed(0) < r.speed(1)); // A new stage must not erase the existing offset.
    }
    {
        Rig r; r.heights(300, 100);
        assert(r.f.GetFault() == GantryFault::LiftMismatch);
        assert(r.f.GetTargetSpeed(0) == 0 && r.f.GetTargetSpeed(1) == 0);
        assert(r.f.GetLiftSyncErrorDeg() == 0);
    }
    {
        Rig r; r.heights(104, 100); r.feedback = false; r.tick();
        assert(r.f.GetFault() == GantryFault::Feedback);
        assert(r.f.GetTargetSpeed(0) == 0 && r.f.GetTargetSpeed(1) == 0);
        assert(r.f.GetLiftSyncErrorDeg() == 0);
    }
    {
        Rig r; r.lower(); r.heights(0, 0); r.hit(4);
        assert(r.f.GetState() == GantryState::Idle);
        assert(r.f.GetTargetSpeed(0) == 0 && r.f.GetTargetSpeed(1) == 0);
    }
    {
        Rig r; r.heights(4, 0); r.enabled = false; r.tick();
        assert(r.f.GetLiftSyncErrorDeg() == 0);
        r.enabled = true; r.key = false; r.tick();
        r.key = true; r.tick(); r.tick(2000);
        assert(r.f.GetState() == GantryState::LiftFirst);
        assert(std::abs(r.f.GetLiftSyncErrorDeg() - 4) < 1e-6);
        assert(r.speed(0) < r.speed(1)); // Mode changes must not rebase alignment.
    }
}
'''
with tempfile.TemporaryDirectory(prefix="gantry_sync_") as directory:
    work = Path(directory)
    (work / "gantry_fsm.cpp").write_text(source, encoding="utf-8")
    (work / "test.cpp").write_text(test, encoding="utf-8")
    for sign5, sign6 in ((1, -1), (-1, 1), (1, 1), (-1, -1)):
        for speed5, gain, ratio in ((1, "1", ".2f"), (2, "1", ".2f"),
                                    (1, "0", ".2f"), (1, "1", "0")):
            values = {
                "LIFT_ID5_HOME_DEG": "100", "LIFT_ID6_HOME_DEG": "-200",
                "LIFT_ID5_FIRST_DEG": str(100 + sign5 * 1000),
                "LIFT_ID6_FIRST_DEG": str(-200 + sign6 * 1000),
                "LIFT_ID5_SECOND_DEG": str(100 + sign5 * 2000),
                "LIFT_ID6_SECOND_DEG": str(-200 + sign6 * 2000),
                "DM_RETRACT_DEG": "40", "DM_EXTEND_DEG": "-460",
                "LIFT_ID5_SPEED_RAD_S": str(speed5), "LIFT_ID6_SPEED_RAD_S": "1",
                "DM_SPEED_RAD_S": "2",
                "LIFT_SYNC_KP": gain, "LIFT_SYNC_MAX_SPEED_RATIO": ratio,
            }
            fixture = header
            for key, value in values.items():
                fixture, count = re.subn(
                    r"(static constexpr \w+ " + key + r"\s*=\s*)[^;]+;",
                    lambda match: match[1] + value + ";", fixture)
                assert count == 1, key
            (work / "gantry_fsm.hpp").write_text(fixture, encoding="utf-8")
            exe = work / "test.exe"
            obj = work / "gantry_fsm.o"
            # Keep the production control arithmetic single precision; the test oracle may use double.
            subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                            "-Wdouble-promotion", "-I", str(work), "-c",
                            str(work / "gantry_fsm.cpp"), "-o", str(obj)], check=True)
            subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                            "-I", str(work), str(work / "test.cpp"),
                            str(obj), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
            print(f"PASS: lift sync signs={sign5},{sign6}, speed limits={speed5},1, "
                  f"gain={gain}, correction ratio={ratio}", flush=True)
