"""Verify encoder feedback, gravity calibration and recovery remain connected."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FeedbackIntegrationTest(unittest.TestCase):
    def test_gravity_encoder_horizontal_zero(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        self.assertTrue("PITCH_ENCODER_HORIZONTAL_RAD = 46.7f * DEG_TO_RAD" in task,
                        "measured horizontal encoder zero is missing")
        start = task.index(" // 重力补偿按编码器换算水平角：")
        end = task.index(" float pitch_mit_kd", start)
        block = task[start:end]
        header = (ROOT / "user/core/Alg/Feedforward/Feedforward.hpp").read_text(encoding="utf-8-sig")
        start = header.index("    class Gravity\n")
        end = header.index("\n    };", start) + len("\n    };")
        harness = '#include <cassert>\n#include <cmath>\nnamespace Alg::Feedforward {\n'
        harness += header[start:end] + '\n}\n'
        harness += 'constexpr float DEG_TO_RAD=3.14159265358979323846f/180.0f;\n'
        harness += 'constexpr float PITCH_ENCODER_HORIZONTAL_RAD=46.7f*DEG_TO_RAD;\n'
        harness += 'Alg::Feedforward::Gravity pitch_gravity_ff(2.0f,0.0f);\n'
        harness += 'float compute(float pitch_current_angle){\n'
        harness += block + 'return pitch_gravity_output;\n}\n'
        harness += r'''
int main() {
    assert(compute(PITCH_ENCODER_HORIZONTAL_RAD)==2.0f);
    for (float angle : {-10.0f, 20.0f}) {
        const float actual=compute(PITCH_ENCODER_HORIZONTAL_RAD+angle*DEG_TO_RAD);
        assert(std::fabs(actual-2.0f*std::cos(angle*DEG_TO_RAD))<0.00001f);
    }
}
'''
        harness = '#include <initializer_list>\n' + harness
        with tempfile.TemporaryDirectory(prefix="pitch_zero_") as temp:
            cpp = Path(temp) / "test.cpp"
            exe = Path(temp) / "test.exe"
            cpp.write_text(harness, encoding="utf-8")
            subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                            cpp.as_posix(), "-o", exe.as_posix()], check=True)
            subprocess.run([str(exe)], check=True)

    def test_encoder_startup_recovery_and_limits(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        self.assertLess(task.index("pitch_target_angle = pitch_current_angle;"),
                        task.index("gimbal_motor.On(1,"))
        loop = task.split("    for(;;)", 1)[1]
        self.assertLess(loop.index("UpdatePitchMotorFeedback();"),
                        loop.index("if(!remoteController.isConnected()"))
        for marker in ("if (pitch_motor_online && pitch_motor_was_offline)", "if (remote_was_offline)"):
            body = task.split(marker, 1)[1].split("\n  }", 1)[0]
            self.assertIn("pitch_gimbal_fsm.ReAnchor(pitch_current_angle)", body)
        self.assertIn("pitch_gimbal_fsm.Update_Angle_Limit(pitch_current_angle, pitch_current_angle)", task)
        self.assertIn("pitch_current_angle - PITCH_ENCODER_HORIZONTAL_RAD", task)
        self.assertIn("pitch_gravity_ff.GravityFeedforward(pitch_gravity_angle)", task)
        self.assertIn("pitch_gravity_output = pitch_gravity_ff.getFeedforward()", task)


if __name__ == "__main__":
    unittest.main()
