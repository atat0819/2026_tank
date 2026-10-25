from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
TASK = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
PROJECT = (ROOT / "MDK-ARM/tank_gimbal.uvprojx").read_text(encoding="utf-8")


class YawAutoIntegrationTest(unittest.TestCase):
    def test_removed_yaw_has_no_control_or_motor_definition(self):
        self.assertNotIn("YawAutoLqrEso_", TASK)
        self.assertNotIn("yaw_gimbal_fsm", TASK)
        self.assertNotIn("yaw_motor_online", TASK)
        self.assertIn("J4340<1> gimbal_motor", TASK)
        self.assertNotIn("gimbal_motor.ctrl_Mit(2,", TASK)

    def test_pitch_keeps_mode_reset_and_final_guard(self):
        self.assertIn("pitch_mode_changed != 0U || pitch_limit_reset != 0U", TASK)
        self.assertIn("pitch_gimbal_fsm.Limit_Torque(pitch_control_output)", TASK)

    def test_keil_compiles_upstream_controller(self):
        self.assertIn("yaw_auto_lqr_eso_controller.c", PROJECT)


if __name__ == "__main__":
    unittest.main()
