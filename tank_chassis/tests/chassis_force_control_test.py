"""Compile the real force controller, power limiter and filtered speed plant."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
assert (ROOT / "user/core/Alg/ChassisCalculation/MecanumForceControl.hpp").exists(), \
    "missing body-speed-to-force controller"
source = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
coefficients = re.search(r"float poly_coeffs\[6\] = \{.*?\};", source, re.S).group(0)
filter_params = [re.search(r"SecondOrderLPFFilter chassis_" + axis + r"_filter\(([^;]+)\);",
                           source).group(1) for axis in ("vx", "vy", "wz")]
with tempfile.TemporaryDirectory(prefix="chassis-force-") as directory:
    work = Path(directory)
    fixture = coefficients + "\n" + "\n".join(
        "constexpr float test_" + axis + "_filter[3] = {" + params + "};"
        for axis, params in zip(("x", "y", "z"), filter_params))
    (work / "task_power_coefficients.hpp").write_text(fixture, encoding="utf-8")
    executable = work / "test.exe"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-I", ROOT.as_posix(),
                    "-I", (ROOT / "user").as_posix(), "-I", work.as_posix(),
                    (ROOT / "tests/chassis_force_control_test.cpp").as_posix(),
                    (ROOT / "user/core/Alg/Filter/Filter.cpp").as_posix(),
                    "-o", executable.as_posix()], check=True)
    subprocess.run([str(executable)], check=True)
