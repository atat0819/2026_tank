from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class PitchStartGateRemovalTest(unittest.TestCase):
    def test_pitch_control_has_no_manual_release_gate(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        self.assertFalse("Update_Pitch_Start_Gate" in task, "manual release gate still active")
        self.assertFalse("pitch_hold_zero" in task, "manual release still forces zero torque")
        self.assertIn("pitch_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_STOP", task)
        self.assertIn("pitch_gimbal_fsm.Limit_Torque", task)


if __name__ == "__main__":
    unittest.main()
