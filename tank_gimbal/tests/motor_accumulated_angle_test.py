"""验证实际累计角度 getter 的单位换算及多圈保留。"""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "user/core/BSP/Motor/MotorBase.hpp").read_text(encoding="utf-8")
getters = "\n".join(
    re.search(r"float " + name + r"\(uint8_t id\)\s*\{[^}]+\}", source).group()
    for name in ("getAddAngleDeg", "getAddAngleRad")
)
code = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
struct Motor {
    struct { double add_angle; } unit_data_[2];
''' + getters + r'''
};
int main() {
    Motor motor{};
    constexpr double pi = 3.14159265358979323846;
    for (double degrees : {0.0, 90.0, -180.0, 360.0, -720.0, 1080.0}) {
        for (uint8_t id = 1; id <= 2; ++id) {
            motor.unit_data_[id - 1].add_angle = degrees;
            assert(motor.getAddAngleDeg(id) == degrees);
            assert(std::fabs(motor.getAddAngleRad(id) - degrees * pi / 180.0) < 1e-6);
            assert(motor.unit_data_[id - 1].add_angle == degrees);
        }
    }
}
'''
with tempfile.TemporaryDirectory(prefix="motor_angle_test_") as directory:
    cpp = Path(directory) / "test.cpp"
    exe = Path(directory) / "test.exe"
    cpp.write_text("#include <initializer_list>\n" + code, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", str(cpp), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print("PASS: accumulated degrees/radians, both IDs, negative and multiple turns")
