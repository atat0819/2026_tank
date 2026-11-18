"""Build the actual FSM against constexpr calibration fixtures."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
header = (root / "feeder_fsm/gantry_fsm.hpp").read_text(encoding="utf-8")
source = (root / "feeder_fsm/gantry_fsm.cpp").read_text(encoding="utf-8")
valid = {
    "CALIBRATED": "true",
    "LIFT_ID5_HOME_DEG": "100", "LIFT_ID6_HOME_DEG": "-200",
    "LIFT_ID5_FIRST_DEG": "1100", "LIFT_ID6_FIRST_DEG": "-1200",
    "LIFT_ID5_SECOND_DEG": "2100", "LIFT_ID6_SECOND_DEG": "-2200",
    "DM_RETRACT_DEG": "40", "DM_EXTEND_DEG": "-460",
    "LIFT_ID5_SPEED_RAD_S": "10", "LIFT_ID6_SPEED_RAD_S": "10", "DM_SPEED_RAD_S": "2",
    "LIFT_FIRST_TIMEOUT_MS": "10000", "EXTEND_TIMEOUT_MS": "10000",
    "LIFT_SECOND_TIMEOUT_MS": "10000", "RETRACT_TIMEOUT_MS": "10000", "LOWER_TIMEOUT_MS": "10000",
    "HOME_STOP_SPEED_RAD_S": ".1f",
    "LIFT_HOME_TOLERANCE_DEG": "10",
}
invalid_test = '''
#include "gantry_fsm.hpp"
#include <cassert>
int main() {
    assert(!Class_Gantry_FSM::ConfigValid());
    Class_Gantry_FSM f;
    f.Update(true, true, true, false, false, false, false, false, true, false, 0, 0, 0, 0, 0, 0, 0, 0);
    f.Update(true, true, true, true, true, false, false, false, true, false, 0, 0, 0, 0, 0, 0, 0, 5);
    assert(!f.IsActive());
    for (unsigned axis=0; axis<3; ++axis) assert(!f.IsKeeping(axis));
    assert(f.GetState()==GantryState::Disabled);
}
'''
cases = [
    ("sequence", valid),
    ("uncalibrated_profile", {"CALIBRATED": "false"}),
    ("uncalibrated", {**valid, "CALIBRATED": "false"}),
    ("zero_travel", {**valid, "LIFT_ID5_FIRST_DEG": "100"}),
    ("wrong_direction", {**valid, "LIFT_ID6_SECOND_DEG": "100"}),
    ("nan_speed", {**valid, "LIFT_ID5_SPEED_RAD_S": "std::numeric_limits<float>::quiet_NaN()"}),
    ("zero_home_stop_speed", {**valid, "HOME_STOP_SPEED_RAD_S": "0"}),
    ("zero_home_tolerance", {**valid, "LIFT_HOME_TOLERANCE_DEG": "0"}),
    ("negative_sync_gain", {**valid, "LIFT_SYNC_KP": "-1"}),
    ("nan_sync_gain", {**valid, "LIFT_SYNC_KP": "std::numeric_limits<float>::quiet_NaN()"}),
    ("negative_sync_limit", {**valid, "LIFT_SYNC_MAX_SPEED_RATIO": "-.1f"}),
    ("excessive_sync_limit", {**valid, "LIFT_SYNC_MAX_SPEED_RATIO": "1.01f"}),
    ("nan_sync_limit", {**valid, "LIFT_SYNC_MAX_SPEED_RATIO": "std::numeric_limits<float>::quiet_NaN()"}),
]
with tempfile.TemporaryDirectory(prefix="gantry_fsm_") as directory:
    work = Path(directory)
    (work / "gantry_fsm.cpp").write_text(source, encoding="utf-8")
    (work / "gantry_fsm.hpp").write_text(header, encoding="utf-8")
    zero_test = invalid_test.replace(
        "assert(!Class_Gantry_FSM::ConfigValid());",
        """using namespace GantryTuning;
    assert(LIFT_ID5_HOME_DEG == 0 && LIFT_ID6_HOME_DEG == 0);
    assert(LIFT_ID5_FIRST_DEG == 0 && LIFT_ID6_FIRST_DEG == 0);
    assert(LIFT_ID5_SECOND_DEG == 0 && LIFT_ID6_SECOND_DEG == 0);
    assert(DM_RETRACT_DEG == 0 && DM_EXTEND_DEG == 0);
    assert(LIFT_ID5_SPEED_RAD_S == 0 && LIFT_ID6_SPEED_RAD_S == 0 && DM_SPEED_RAD_S == 0);
    assert(!Class_Gantry_FSM::ConfigValid());""")
    (work / "test.cpp").write_text(zero_test, encoding="utf-8")
    exe = work / "test.exe"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", str(work),
                    str(work / "test.cpp"), str(work / "gantry_fsm.cpp"), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
    print("PASS: zeroed actual configuration remains disabled")
    home_fixture = header
    for key in ("LIFT_ID5_HOME_DEG", "LIFT_ID6_HOME_DEG", "LIFT_ID5_FIRST_DEG",
                "LIFT_ID6_FIRST_DEG", "LIFT_ID5_SECOND_DEG", "LIFT_ID6_SECOND_DEG",
                "DM_RETRACT_DEG", "DM_EXTEND_DEG", "LIFT_ID5_SPEED_RAD_S",
                "LIFT_ID6_SPEED_RAD_S", "DM_SPEED_RAD_S", "LIFT_HOME_TOLERANCE_DEG"):
        home_fixture, count = re.subn(
            r"(static constexpr \w+ " + key + r"\s*=\s*)[^;]+;",
            lambda match: match[1] + valid[key] + ";", home_fixture)
        assert count == 1, key
    (work / "gantry_fsm.hpp").write_text(home_fixture, encoding="utf-8")
    (work / "test.cpp").write_text(r'''
#include "gantry_fsm.hpp"
#include <cassert>
int main() {
    auto check = [](double id5, double id6, bool allowed) {
        Class_Gantry_FSM f;
        f.Update(true, true, true, false, false, false, false, false, false, false,
                 0, 0, 0, 0, 0, 0, 0, 0);
        f.Update(true, true, true, true, true, false, false, false, false, false,
                 id5, id6, 0, 0, 0, 0, 0, 5);
        f.Update(true, true, true, true, true, false, false, false, false, false,
                 id5, id6, 0, 0, 0, 0, 0, 2005);
        assert((f.GetState() == GantryState::LiftFirst) == allowed);
        if (!allowed) assert(f.GetFault() == GantryFault::NotHome);
    };
    check(10, -10, true);
    check(-10, 10, true);
    check(10.01, 0, false);
    check(0, -10.01, false);
}
''', encoding="utf-8")
    exe = work / "test.exe"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", str(work),
                    str(work / "test.cpp"), str(work / "gantry_fsm.cpp"), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
    print("PASS: gantry startup home tolerance with valid travel fixture")
    for name, values in cases:
        fixture = header
        for key, value in values.items():
            fixture, count = re.subn(
                r"(static constexpr \w+ " + key + r"\s*=\s*)[^;]+;",
                lambda match: match[1] + value + ";", fixture)
            assert count == 1, key
        (work / "gantry_fsm.hpp").write_text("#include <limits>\n" + fixture, encoding="utf-8")
        test = ((root / "tests/gantry_fsm_test.cpp").read_text(encoding="utf-8")
                if name == "sequence" else invalid_test)
        (work / "test.cpp").write_text(test, encoding="utf-8")
        exe = work / "test.exe"
        subprocess.run(
            ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", str(work),
             str(work / "test.cpp"), str(work / "gantry_fsm.cpp"), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        print("PASS: gantry FSM " + name)
