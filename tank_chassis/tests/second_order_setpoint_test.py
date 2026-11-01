"""Compile and exercise the actual velocity-domain second-order planner."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="second-order-setpoint-") as directory:
    executable = Path(directory) / "test.exe"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-I", ROOT.as_posix(),
                    (ROOT / "tests/second_order_setpoint_test.cpp").as_posix(),
                    "-o", executable.as_posix()], check=True)
    subprocess.run([str(executable)], check=True)
