"""Verify the retired CAN board link is removed while RS485 and motor routing remain."""
from pathlib import Path
import xml.etree.ElementTree as ET

gimbal = Path(__file__).resolve().parents[1]
chassis = gimbal.parent / "tank_chassis"

legacy = {
    gimbal: (
        "communication_between_boards/boards_communication.cpp",
        "communication_between_boards/boards_communication.hpp",
        "communication_between_boards/refree_receive.cpp",
        "communication_between_boards/refree_receive.hpp",
    ),
    chassis: (
        "communication/gimbal_refree.cpp",
        "communication/gimbal_refree.hpp",
    ),
}
for root, paths in legacy.items():
    for path in paths:
        assert not (root / path).exists(), f"legacy CAN board-link file remains: {root / path}"

for root, project in (
    (gimbal, "MDK-ARM/tank_gimbal.uvprojx"),
    (chassis, "MDK-ARM/tank_chassis.uvprojx"),
):
    project_paths = {
        (entry.text or "").replace("\\", "/").lower()
        for entry in ET.parse(root / project).iter("FilePath")
    }
    for path in legacy[root]:
        assert not any(p.endswith("/" + path.lower()) for p in project_paths), path
    assert any("boards_communication_uart_485.cpp" in p for p in project_paths)

gimbal_task = (gimbal / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
gimbal_header = (gimbal / "RtosTask/can_send_task.hpp").read_text(encoding="utf-8")
chassis_task = (chassis / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
for source in (gimbal_task, gimbal_header, chassis_task):
    assert '"boards_communication.hpp"' not in source
    assert '"../communication/gimbal_refree.hpp"' not in source
    assert "Communication::GimbalRefree gimbal_refree" not in source
assert "RS485_SendBoardData(" in gimbal_task
assert "Referee485_Update(" in chassis_task
assert "fdcan2.register_rx_callback" in chassis_task and "front_4340.Parse(frame)" in chassis_task
assert "fdcan3.register_rx_callback" in chassis_task and "rear_6248.Parse(frame)" in chassis_task
assert "fdcan1.register_rx_callback" in gimbal_task and "friction_motor.Parse(frame)" in gimbal_task
assert "fdcan3.register_rx_callback" in gimbal_task and "pitch_motor.Parse(frame)" in gimbal_task
print("PASS: legacy CAN board link removed; RS485 and motor CAN routing retained")
