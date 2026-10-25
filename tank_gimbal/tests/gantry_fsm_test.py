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
}
invalid_test = '''
#include "gantry_fsm.hpp"
#include <cassert>
int main() {
    assert(!Class_Gantry_FSM::ConfigValid());
    Class_Gantry_FSM f;
    f.Update(true, true, false, false, false, false, false, true, true, 0, 0, 0, 0);
    f.Update(true, true, true, true, false, false, false, true, true, 0, 0, 0, 5);
    assert(!f.IsActive());
    for (unsigned axis=0; axis<3; ++axis) assert(!f.IsKeeping(axis));
    assert(f.GetState()==GantryState::Disabled);
}
'''
cases = [
    ("sequence", valid),
    ("defaults", {}),
    ("uncalibrated", {**valid, "CALIBRATED": "false"}),
    ("zero_travel", {**valid, "LIFT_ID5_FIRST_DEG": "100"}),
    ("wrong_direction", {**valid, "LIFT_ID6_SECOND_DEG": "100"}),
    ("nan_speed", {**valid, "LIFT_ID5_SPEED_RAD_S": "std::numeric_limits<float>::quiet_NaN()"}),
]
with tempfile.TemporaryDirectory(prefix="gantry_fsm_") as directory:
    work = Path(directory)
    (work / "gantry_fsm.cpp").write_text(source, encoding="utf-8")
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
