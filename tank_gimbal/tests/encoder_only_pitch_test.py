"""Encoder-only task wiring: no IMU receive/control/telemetry dependency."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class EncoderOnlyTest(unittest.TestCase):
    def test_no_active_imu_or_vision_takeover(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
        remote = (ROOT / "RtosTask/remote_control_task.cpp").read_text(encoding="utf-8")
        for marker in ("imu.", "imu_feedback_fsm", "using_encoder", "feedback_source_changed",
                       "_imu_pid", "GetPitchAngle()", "GIMBAL_MODE_VISION"):
            self.assertTrue(marker not in task, f"active dependency remains: {marker}")
        for marker in ("StartImuReceive", "imu.", "get_uart7()", "SendToVision("):
            self.assertTrue(marker not in remote, f"IMU or vision TX remains: {marker}")
        self.assertIn("pitch_input.vision_ready = false;", task)
        self.assertIn("pitch_input.vision_fresh = false;", task)
        self.assertIn("pitch_recovery_fsm.Should_Enable(pitch_motor_online", task)
        self.assertIn("pitch_gimbal_fsm.Limit_Torque(pitch_control_output)", task)

    def test_actual_feedback_uses_only_pitch_encoder(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
        begin = task.rindex("static void UpdatePitchMotorFeedback()")
        selector = task[begin:task.index("\n}", begin) + 2]
        code = r'''
#include <cassert>
#include <cstdint>
struct {
    float angle=0.4f,speed=-0.6f;
    float getAngleDeg(int slot) { assert(slot==1); return 30.0f; }
    float getAngleRad(int slot) { assert(slot==1); return angle; }
    float getVelocityRpm(int slot) { assert(slot==1); return 6.0f; }
    float getVelocityRads(int slot) { assert(slot==1); return speed; }
    float getCurrent(int slot) { assert(slot==1); return 1.0f; }
    uint8_t getTemperature(int slot) { assert(slot==1); return 25; }
    float getAddAngleDeg(int slot) { assert(slot==1); return 390.0f; }
} gimbal_motor;
struct {
    float angle_deg,angle_rad,velocity_rpm,velocity_rads,current_a,delta_angle;
    uint8_t temperature;
} dm4340_state[1];
float pitch_current_angle=0,pitch_current_speed=0;
SELECTOR
int main() {
    UpdatePitchMotorFeedback();
    assert(pitch_current_angle==0.4f && pitch_current_speed==-0.6f);
    gimbal_motor.angle=0.8f; gimbal_motor.speed=1.2f;
    UpdatePitchMotorFeedback();
    assert(pitch_current_angle==0.8f && pitch_current_speed==1.2f);
}
'''.replace("SELECTOR", selector)
        with tempfile.TemporaryDirectory(prefix="pitch_encoder_") as directory:
            cpp = Path(directory) / "test.cpp"
            exe = Path(directory) / "test.exe"
            cpp.write_text(code, encoding="utf-8")
            subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                            str(cpp), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
