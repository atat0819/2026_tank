"""Compile actual encoder PID calls and the remote/keymouse outer-loop selection."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class EncoderPidTest(unittest.TestCase):
    def test_encoder_pid_calls_and_resets(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
        stems = ["pitch_remote_angle", "pitch_keymouse_angle"]
        self.assertTrue("_imu_pid" not in task, "IMU PID remains active")
        for stem in stems:
            self.assertIn(f"ALG::PID::PID {stem}_encoder_pid(", task)
            self.assertEqual(task.count(f"{stem}_encoder_pid.reset();"), 3)
        calls = re.findall(r"\b(pitch_\w+_encoder_pid)\.UpDate\(([^)]*)\)", task)
        self.assertEqual({name.removesuffix("_encoder_pid") for name, _ in calls}, set(stems))
        harness = '#include <cassert>\n#include "user/core/Alg/PID/pid.hpp"\n'
        harness += "\n".join(f"ALG::PID::PID {name}(7,0,0,100,10,10);" for name, _ in calls)
        harness += "\nint main() { float pitch_error=1;\n"
        harness += "\n".join(f"{name}.reset(); assert({name}.UpDate({args})==7);" for name, args in calls)
        harness += "\n}\n"
        self.compile_and_run(harness)

    def test_remote_and_keymouse_use_their_encoder_gains(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
        begin = task.index("pitch_target_speed = is_keymouse")
        expression = task[begin:task.index(";", begin) + 1]
        harness = r'''
#include <cassert>
#include "user/core/Alg/PID/pid.hpp"
ALG::PID::PID pitch_remote_angle_encoder_pid(7,0,0,100,10,10);
ALG::PID::PID pitch_keymouse_angle_encoder_pid(13,0,0,100,10,10);
float select(bool is_keymouse) {
    float pitch_error=1,pitch_target_speed=0;
    EXPRESSION
    return pitch_target_speed;
}
int main() {
    assert(select(false)==7);
    assert(pitch_keymouse_angle_encoder_pid.getOutput()==0);
    assert(select(true)==13);
    pitch_remote_angle_encoder_pid.setK(9,0,0);
    assert(select(false)==9);
    assert(select(true)==13);
}
'''.replace("EXPRESSION", expression)
        self.compile_and_run(harness)

    def compile_and_run(self, harness):
        with tempfile.TemporaryDirectory(prefix="yaw_pid_") as temp:
            cpp = Path(temp) / "test.cpp"
            exe = Path(temp) / "test.exe"
            cpp.write_text(harness, encoding="utf-8")
            subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                            "-I", ROOT.as_posix(),
                            "-I", (ROOT / "user/core/Alg/PID").as_posix(), cpp.as_posix(),
                            (ROOT / "user/core/Alg/PID/pid.cpp").as_posix(), "-o", exe.as_posix()], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
