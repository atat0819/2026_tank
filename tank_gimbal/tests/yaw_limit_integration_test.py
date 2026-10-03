"""检查任务把反馈和最终输出接入 FSM，边界判断保留在 FSM 内。"""
from pathlib import Path

root = Path(__file__).resolve().parents[1]
task = (root / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
fsm = (root / "feeder_fsm/gimbal_fsm.cpp").read_text(encoding="utf-8")

assert task.index("if (!pitch_motor_online || !yaw_motor_online)") < task.index("yaw_gimbal_fsm.Update_Yaw_Limit(")
assert task.index("if (imu_fault)") < task.index("yaw_gimbal_fsm.Update_Yaw_Limit(")
assert task.index("yaw_gimbal_fsm.Update(yaw_input, yaw_current_angle)") < task.index("yaw_gimbal_fsm.Update_Yaw_Limit(")
assert task.index("yaw_gimbal_fsm.Update_Yaw_Limit(") < task.index("yaw_target_angle = yaw_gimbal_fsm.Get_Target_Angle()")
assert task.index("yaw_control_output = BSP::Motor::DM::ClampMitCommandValue(") < task.index("yaw_gimbal_fsm.Limit_Yaw_Torque(")
assert task.index("yaw_gimbal_fsm.Limit_Yaw_Torque(") < task.index("gimbal_motor.ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, yaw_control_output)")
assert "getAngleDeg(1)" in task
assert "YawWouldPushOutward" in fsm
assert "YawWouldPushOutward" not in task
assert "yaw_limit.hpp" not in task
print("PASS: Yaw limit decisions live in FSM and surround angle/torque control")
