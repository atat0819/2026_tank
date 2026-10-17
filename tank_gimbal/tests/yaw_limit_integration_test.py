"""验证两轴安全检查、限位反馈、控制器复位和最终力矩拦截的调用顺序。"""
from pathlib import Path

root = Path(__file__).resolve().parents[1]
task = (root / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
fsm = (root / "feeder_fsm/gimbal_fsm.cpp").read_text(encoding="utf-8")

for axis, motor in (("yaw", 1), ("pitch", 2)):
    update = f"{axis}_gimbal_fsm.Update_Angle_Limit("
    guard = f"{axis}_gimbal_fsm.Limit_Torque("
    assert task.index(f"if (!{axis}_motor_online)") < task.index(update)
    assert task.index("const bool feedback_source_changed = UpdateGimbalFeedback(") < task.index(update)
    assert task.index(f"{axis}_gimbal_fsm.Update({axis}_input, {axis}_current_angle)") < task.index(update)
    assert task.index(update) < task.index(f"{axis}_target_angle = {axis}_gimbal_fsm.Get_Target_Angle()")
    assert task.index(f"{axis}_control_output = BSP::Motor::DM::ClampMitCommandValue(") < task.index(guard)
    assert task.index(guard) < task.index(f"gimbal_motor.ctrl_Mit({motor}, 0.0f, 0.0f, 0.0f, 0.0f, {axis}_control_output)")
    assert f"{axis}_mode_changed != 0U || {axis}_limit_reset != 0U" in task
    assert f"{axis}_gimbal_fsm.Take_Limit_Reset_Flag()" in task

assert "Update_Angle_Limit(DegreesToRadians(gimbal_motor.getAngleDeg(1)), yaw_current_angle)" in task
assert "pitch_gimbal_fsm.Update_Angle_Limit(DegreesToRadians(gimbal_motor.getAngleDeg(2)), pitch_current_angle)" in task
assert "pitch_gimbal_fsm_config.min_angle = DegreesToRadians(34.2f)" in task
assert "pitch_gimbal_fsm_config.max_angle = DegreesToRadians(67.9f)" in task
assert "Set_Angle_Limit_Enabled" not in task
assert "yaw_gimbal_fsm_config.min_angle = DegreesToRadians(-60.0f)" in task
assert "yaw_gimbal_fsm_config.max_angle = DegreesToRadians(60.0f)" in task
assert "yaw_gimbal_fsm_config.limit_target_angle = 0U" in task
assert "pitch_gimbal_fsm_config.limit_target_angle = 0U" in task
assert "pitch_gimbal_fsm_config.limit_upper_torque = 0U" in task
assert "yaw_gimbal_fsm_config.limit_upper_torque = 0U" not in task
assert task.index("pitch_control_output += pitch_gravity_output") < task.index("pitch_gimbal_fsm.Limit_Torque(")
assert "Would_Push_Outward" in fsm
assert "Would_Push_Outward" not in task
assert "60.0f" not in fsm
print("PASS: both axes use configured FSM limits after safety checks and before motor output")
