"""Verify that each gantry movement stage uses its own speed setting."""
from pathlib import Path
import re
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
header = (ROOT / "fsm/gantry_fsm.hpp").read_text(encoding="utf-8")
values = {
    "CALIBRATED": "true",
    "LIFT_ID5_HOME_DEG": "0.0f",
    "LIFT_ID6_HOME_DEG": "0.0f",
    "LIFT_ID5_FIRST_DEG": "100.0f",
    "LIFT_ID6_FIRST_DEG": "-100.0f",
    "LIFT_ID5_SECOND_DEG": "200.0f",
    "LIFT_ID6_SECOND_DEG": "-200.0f",
    "DM_RETRACT_DEG": "0.0f",
    "DM_EXTEND_DEG": "-80.0f",
    "LIFT_FIRST_SPEED_RAD_S": "1.0f",
    "DM_EXTEND_SPEED_RAD_S": "2.0f",
    "LIFT_SECOND_SPEED_RAD_S": "3.0f",
    "DM_RETRACT_SPEED_RAD_S": "4.0f",
    "LOWER_SPEED_RAD_S": "5.0f",
}

fixture = header
for name, value in values.items():
    fixture, count = re.subn(
        rf"(static constexpr \w+ {name}\s*=\s*)[^;]+;",
        lambda match: match[1] + value + ";",
        fixture,
    )
    assert count == 1, f"Missing independent gantry setting: {name}"

harness = r'''
#include "gantry_fsm.hpp"
#include <cassert>
#include <cmath>

Class_Gantry_FSM gantry;
bool limit[5] = {};
bool start_key = false;
float angle[3] = {};
uint32_t tick = 0;

void update(uint32_t elapsed = 2U) {
    tick += elapsed;
    gantry.Update(true, true, true, start_key, start_key,
                  limit[0], limit[1], limit[2], limit[3], limit[4],
                  angle[0], angle[1], angle[2], 0, 0, 0, 0, tick);
}

void hit_limit(unsigned index) {
    limit[index] = true;
    update();
    limit[index] = false;
}

void expect(GantryState state, float id5, float id6, float id9) {
    assert(gantry.GetState() == state);
    const float targets[] = {id5, id6, id9};
    for (unsigned axis = 0; axis < 3; ++axis)
        assert(std::abs(gantry.GetTargetSpeed(axis) - targets[axis]) < 1e-6f);
}

int main() {
    assert(Class_Gantry_FSM::ConfigValid());
    update();
    start_key = true;
    update();
    update(GantryTuning::START_HOLD_MS);
    expect(GantryState::LiftFirst, 1, -1, 0);
    angle[0] = 79; angle[1] = -79;
    update();
    expect(GantryState::LiftFirst, 1, -1, 0);
    angle[0] = 80;
    update();
    assert(gantry.GetScale() == 0.3f); // The leading side reaches the threshold first.
    angle[1] = -80;
    update();
    expect(GantryState::LiftFirst, 0.3f, -0.3f, 0);
    angle[0] = 79; angle[1] = -79;
    update();
    expect(GantryState::LiftFirst, 0.3f, -0.3f, 0);

    angle[0] = 100; angle[1] = -100;
    hit_limit(0);
    expect(GantryState::Extend, 0, 0, -2);
    angle[2] = -72;
    update();
    expect(GantryState::Extend, 0, 0, -2);

    angle[2] = -80;
    hit_limit(1);
    expect(GantryState::LiftSecond, 3, -3, 0);
    angle[0] = 190; angle[1] = -190;
    update();
    expect(GantryState::LiftSecond, 3, -3, 0);

    angle[0] = 200; angle[1] = -200;
    hit_limit(2);
    expect(GantryState::Retract, 0, 0, 4);
    angle[2] = -8;
    update();
    expect(GantryState::Retract, 0, 0, 4);

    angle[2] = 0;
    hit_limit(3);
    expect(GantryState::Lower, -5, 5, 0);
    angle[0] = 20; angle[1] = -20;
    update();
    expect(GantryState::Lower, -5, 5, 0);

    angle[0] = angle[1] = 0;
    hit_limit(4);
    expect(GantryState::Idle, 0, 0, 0);
}
'''

with tempfile.TemporaryDirectory(prefix="gantry_stage_speed_") as directory:
    work = Path(directory)
    (work / "gantry_fsm.hpp").write_text(fixture, encoding="utf-8")
    (work / "gantry_fsm.cpp").write_text((ROOT / "fsm/gantry_fsm.cpp").read_text(encoding="utf-8"),
                                        encoding="utf-8")
    (work / "test.cpp").write_text(harness, encoding="utf-8")
    exe = work / "test.exe"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "test.cpp",
                    "gantry_fsm.cpp", "-o", exe.name], cwd=work, check=True)
    subprocess.run([str(exe)], check=True)

print("PASS: independent stage speeds, opposing lift directions, and first-lift-only slowdown at 80%")
