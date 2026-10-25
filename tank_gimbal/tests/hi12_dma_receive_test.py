"""The retained UART7 hardware wrapper must not start an IMU receiver task path."""
from pathlib import Path

root = Path(__file__).resolve().parents[1]
source = (root / 'RtosTask/remote_control_task.cpp').read_text(encoding='utf-8')
header = (root / 'RtosTask/remote_control_task.hpp').read_text(encoding='utf-8')
for marker in ('StartImuReceive', 'imu_rx_active', 'imu_dma_buffer', 'get_uart7()', 'imu.'):
    assert marker not in source, 'IMU receive dependency remains: ' + marker
assert 'HI12_imu.hpp' not in header and 'HI12_float' not in source + header
assert 'uart5.receive_dma_idle' in source
assert 'uart3_485.receive_dma_idle' in source
print('PASS: UART7 IMU reception disabled; DT7 and RS485 receive paths retained')
