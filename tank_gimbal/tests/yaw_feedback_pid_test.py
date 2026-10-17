"""Verify the actual Yaw outer-loop expression selects independent PID objects."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class YawFeedbackPidTest(unittest.TestCase):
    def test_all_nonvision_pid_pairs(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        stems = [f"{axis}_{loop}" for axis in ("yaw", "pitch")
                 for loop in ("angle", "angle_to_speed", "keymouse_angle",
                              "keymouse_angle_to_speed", "remote_speed", "keymouse_speed")]
        stems.remove("yaw_keymouse_speed")
        stems.remove("pitch_keymouse_speed")
        self.assertNotRegex(task, r"\b(?:yaw|pitch)_keymouse_speed_(?:imu|encoder)_pid\b")
        for stem in stems:
            for source in ("imu", "encoder"):
                self.assertTrue(f"ALG::PID::PID {stem}_{source}_pid(" in task,
                                f"missing independent {stem}_{source}_pid")
                self.assertEqual(task.count(f"{stem}_{source}_pid.reset();"), 3)
            self.assertNotRegex(task, rf"\b{stem}_pid\b")
        calls = re.findall(r"\(using_encoder\s*\?\s*(\w+_encoder_pid)\s*:\s*(\w+_imu_pid)\)\.UpDate\(([^)]*)\)", task)
        self.assertEqual({enc.removesuffix("_encoder_pid") for enc, _, _ in calls}, set(stems))
        declarations = []
        checks = []
        for enc, imu, args in calls:
            declarations.extend((f"ALG::PID::PID {imu}(2,0,0,100,10,10);",
                                 f"ALG::PID::PID {enc}(7,0,0,100,10,10);"))
            expression = f"(using_encoder ? {enc} : {imu}).UpDate({args})"
            checks.append(f"{imu}.reset(); {enc}.reset(); using_encoder=false; "
                          f"assert({expression}==2); assert({enc}.getOutput()==0); "
                          f"using_encoder=true; assert({expression}==7); assert({imu}.getOutput()==2);")
        harness = '#include <cassert>\n#include "user/core/Alg/PID/pid.hpp"\n'
        harness += "\n".join(declarations)
        harness += "\nint main() { bool using_encoder=false; float yaw_error=1,pitch_error=1; "
        harness += "float yaw_target_speed=1,pitch_target_speed=1,yaw_current_speed=0,pitch_current_speed=0;\n"
        harness += "\n".join(checks) + "\n}\n"
        self.compile_and_run(harness)

    def test_pid_selection_and_reset(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        self.assertTrue("ALG::PID::PID yaw_angle_encoder_pid(" in task,
                        "encoder mode needs its own Yaw angle PID")
        self.assertTrue("ALG::PID::PID yaw_angle_imu_pid(" in task)
        self.assertIsNone(re.search(r"\byaw_angle_pid\b|\byaw_angle_pid1\b", task))
        for marker in ("static void ResetYawControlState()",
                       "if (yaw_mode_changed != 0U || yaw_limit_reset != 0U)",
                       "if (yaw_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_STOP"):
            body = task.split(marker, 1)[1].split("\n}", 1)[0]
            for source in ("imu", "encoder"):
                self.assertIn(f"yaw_angle_{source}_pid.reset();", body)
        begin = task.index("yaw_target_speed = is_keymouse")
        expression = task[begin:task.index(";", begin) + 1]
        harness = r'''
#include <cassert>
#include "user/core/Alg/PID/pid.hpp"
ALG::PID::PID yaw_angle_imu_pid(2,0,0,100,10,10);
ALG::PID::PID yaw_angle_encoder_pid(7,0,0,100,10,10);
ALG::PID::PID yaw_keymouse_angle_imu_pid(11,0,0,100,10,10);
ALG::PID::PID yaw_keymouse_angle_encoder_pid(13,0,0,100,10,10);
float select(bool is_keymouse, bool using_encoder) {
    float yaw_error=1, yaw_target_speed=0;
    EXPRESSION
    return yaw_target_speed;
}
int main() {
    yaw_angle_imu_pid.reset();
    yaw_angle_encoder_pid.reset();
    yaw_keymouse_angle_imu_pid.reset();
    yaw_keymouse_angle_encoder_pid.reset();
    assert(select(false,false)==2);
    assert(yaw_angle_encoder_pid.getOutput()==0);
    assert(select(false,true)==7);
    assert(yaw_angle_imu_pid.getOutput()==2);
    assert(select(true,false)==11);
    assert(select(true,true)==13);
    yaw_angle_encoder_pid.setK(9,0,0);
    assert(select(false,false)==2);
    assert(select(false,true)==9);
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
