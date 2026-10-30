"""Compile and execute the actual omni/swerve headers with independent checks."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="omni-swerve-") as directory:
    executable = Path(directory) / "test.exe"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", ROOT.as_posix(),
                    "-I", (ROOT / "user").as_posix(),
                    (ROOT / "tests/omni_swerve_calculation_test.cpp").as_posix(),
                    "-o", executable.as_posix()], check=True)
    subprocess.run([str(executable)], check=True)
