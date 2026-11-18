"""Check chassis arm-link integration and exercise the original wire protocol."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'communication/chassis_to_arm.cpp').read_text(encoding='utf-8')
header = (root / 'communication/chassis_to_arm.hpp').read_text(encoding='utf-8')
task = (root / 'RtosTask/remote_task.cpp').read_text(encoding='utf-8')
gimbal = (root.parent / 'tank_gimbal/RtosTask/remote_control_task.cpp').read_text(encoding='utf-8')
project = (root / 'MDK-ARM/tank_chassis.uvprojx').read_text(encoding='utf-8')
assert 'class ChassisToArm' in header
assert 'get_uart10()' in source
assert 'chassis_to_arm.SendRemoteData(' in task
assert 'gimbal_to_arm.SendRemoteData(' not in gimbal
assert 'chassis_to_arm.cpp' in project

# Reuse the earlier protocol behavior harness against the migrated source.
old = (root.parent / 'tank_gimbal/tests/gimbal_to_arm_test.py').read_text(encoding='utf-8')
fixture = old[old.index("hal = r'''"):old.index('    # Execute the actual task-loop snapshot')]
fixture = fixture.replace('get_uart7()', 'get_uart10()')
fixture = fixture.replace('GimbalToArm', 'ChassisToArm').replace('gimbal_to_arm', 'chassis_to_arm')
fixture = fixture.replace('communication_between_boards/chassis_to_arm.hpp', 'communication/chassis_to_arm.hpp')
exec(fixture)
print('PASS: chassis USART10 arm protocol and task integration')
