from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
task = (root / 'RtosTask/can_send_task.cpp').read_text(encoding='utf-8-sig')
header = (root / 'RtosTask/can_send_task.hpp').read_text(encoding='utf-8-sig')
assert re.search(r'J4310<1>\s+yaw_motor\s*\(\s*0\s*,\s*\{7\}\s*,\s*\{7\}\s*,\s*HAL::FDCAN::FdcanDeviceId::HAL_Fdcan1', task)
assert 'extern DM4340_State_t dm4340_state[2]' in header
assert 'pitch_current_angle = dm4340_state[1].angle_rad;' in task
assert 'pitch_current_speed = dm4340_state[1].velocity_rads;' in task
assert 'dm4340_state[0].torque_nm = yaw_motor.getTorque(1);' in task
assert 'dm4340_state[1].torque_nm' in task
assert task.count('UpdateYawMotorFeedback();') >= 2
assert not re.search(r'yaw_motor\.(?:On|Off|ctrl_\w+)\s*\(', task)
start = task.index('fdcan1.register_rx_callback(')
body = task[start:].split('});', 1)[0].split('{', 1)[1]
program = '''#include <cassert>
#include <cstdint>
namespace HAL { namespace FDCAN { struct Frame { unsigned id; unsigned dlc; bool is_extended_id; bool is_remote_frame; uint8_t data[8]; }; }}
struct Motor { int count = 0; void Parse(const HAL::FDCAN::Frame&) { ++count; } } pitch_motor, yaw_motor;
void receive(const HAL::FDCAN::Frame &frame) { BODY }
int main() {
 HAL::FDCAN::Frame f{}; f.id=7; f.dlc=8; f.data[0]=0x17;
 receive(f); assert(yaw_motor.count==1 && pitch_motor.count==0);
 f.data[0]=0x18; receive(f); assert(yaw_motor.count==1);
 f.data[0]=7; f.dlc=7; receive(f); assert(yaw_motor.count==1);
 f.dlc=8; f.is_extended_id=true; receive(f); assert(yaw_motor.count==1);
 f.is_extended_id=false; f.is_remote_frame=true; receive(f); assert(yaw_motor.count==1);
 f.is_remote_frame=false; f.id=8; receive(f); assert(pitch_motor.count==1 && yaw_motor.count==1);
}
'''.replace('BODY', body)
with tempfile.TemporaryDirectory() as tmp:
    source = Path(tmp) / 'test.cpp'
    source.write_text(program)
    exe = Path(tmp) / 'test.exe'
    subprocess.run(['g++', '-std=c++17', str(source), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
