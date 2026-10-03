from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
TASK = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
PROJECT = (ROOT / "MDK-ARM/tank_gimbal.uvprojx").read_text(encoding="utf-8")


class YawAutoIntegrationTest(unittest.TestCase):
    def test_vision_yaw_uses_all_three_reference_values_and_motor_feedback(self):
        vision = TASK.split("if (yaw_mode == GIMBAL_MODE_VISION)", 1)[1].split("\n    else", 1)[0]
        for value in (
            "yaw_target_angle",
            "vision_comm.GetYawVelocity()",
            "vision_comm.GetYawAcceleration()",
            "yaw_current_angle",
            "yaw_current_speed",
            "gimbal_motor.getTorque(1)",
            "YawAutoLqrEso_Calc",
        ):
            self.assertIn(value, vision)
        self.assertNotIn("yaw_version_angle_pid.UpDate", vision)
        self.assertNotIn("yaw_version_speed_pid.UpDate", vision)

    def test_mode_and_limit_changes_reset_controller_and_keep_final_guard(self):
        self.assertIn("YawAutoLqrEso_Reset(&yaw_auto_ctrl", TASK)
        self.assertIn("yaw_mode_changed != 0U || yaw_limit_reset != 0U", TASK)
        self.assertIn("yaw_gimbal_fsm.Limit_Yaw_Torque(yaw_control_output)", TASK)

    def test_keil_compiles_upstream_controller(self):
        self.assertIn("yaw_auto_lqr_eso_controller.c", PROJECT)


if __name__ == "__main__":
    unittest.main()
