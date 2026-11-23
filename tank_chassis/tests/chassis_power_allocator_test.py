"""Compile the allocator against the firmware's actual fitted power coefficients."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
coefficients = re.search(r"float poly_coeffs\[6\] = \{.*?\};", source, re.S).group(0)
assert "chassis_power_allocator.Allocate(" in source
assert "stair_retract_power_split || !CHASSIS_POWER_LIMIT_ENABLED" in source
assert "chassis_power_ctrl.DecayingCurrent(" not in source
assert "TURN_MAX_ACCEL_ERROR_M_S" not in source
with tempfile.TemporaryDirectory(prefix="chassis-allocation-") as directory:
    work = Path(directory)
    (work / "task_power_coefficients.hpp").write_text(coefficients, encoding="utf-8")
    executable = work / "test.exe"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-I", str(ROOT),
                    "-I", str(work), str(ROOT / "tests/chassis_power_allocator_test.cpp"),
                    "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
