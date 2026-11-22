from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
task = (root / 'RtosTask/can_send_task.cpp').read_text(encoding='utf-8-sig')
header = (root / 'RtosTask/can_send_task.hpp').read_text(encoding='utf-8-sig')
dispatcher = (root / 'communication_between_boards/input_dispatcher.cpp').read_text(encoding='utf-8-sig')
dispatcher = dispatcher.replace('#include "../user/core/BSP/RemoteControl/DT7.hpp"', '')
assert re.search(r'J4310<1>\s+yaw_motor\s*\(\s*0\s*,\s*\{7\}\s*,\s*\{7\}\s*,\s*HAL::FDCAN::FdcanDeviceId::HAL_Fdcan1', task)
assert 'extern DM4340_State_t dm4340_state[2]' in header
assert 'pitch_current_angle = dm4340_state[1].angle_rad;' in task
assert 'pitch_current_speed = dm4340_state[1].velocity_rads;' in task
assert 'dm4340_state[0].torque_nm = yaw_motor.getTorque(1);' in task
assert 'dm4340_state[1].torque_nm' in task
assert task.count('UpdateYawMotorFeedback();') >= 2
assert 'UpdateCameraYawControl' not in task
params = re.search(
    r'ALG::PID::PID yaw_center_encoder_pid\([^\n]+\);\s+'
    r'ALG::PID::PID yaw_angle_encoder_pid\([^\n]+\);\s+'
    r'float yaw_angle_kd[^\n]+\s+float yaw_center_kd[^\n]+', task
).group()
start = task.index('fdcan1.register_rx_callback(')
body = task[start:].split('});', 1)[0].split('{', 1)[1]
start = task.index('    bool yaw_wait_for_release = false;')
control_state = task[start:task.index('    // 主循环', start)]
start = task.index('static void ResetYawControlState()')
yaw_reset = task[start:task.index('\n}', start) + 2]
start = task.index('static bool UpdateMotorEnable(')
motor_enable = task[start:task.index('\n}', start) + 2]
start = task.index('        RemoteData_t remote_input{};')
control = task[start:task.index('        gimbal_remote_offline = 0;', start)]
assert 'pitch_input.s1       = remote_input.s1;' in task
assert 'pitch_input.is_keymouse = is_keymouse;' in task
assert 'pitch_input.joystick_speed = remote_input.gimbal_pitch;' in task
assert not re.search(r'\b(?:return|continue)\s*;', control)
program = '''#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include "communication_between_boards/boards_communication_uart_485.hpp"
#include "feeder_fsm/motor_recovery_fsm.hpp"
#include "feeder_fsm/yaw_fsm.hpp"
#include "user/core/Alg/PID/pid.hpp"
constexpr float Rad(float deg) { return deg * 0.017453292519943295f; }
constexpr float PITCH_MAX_SPEED_RAD_S = Rad(95.0f);
namespace BSP::REMOTE_CONTROL {
struct RemoteController { enum { UP=1, DOWN=2, MIDDLE=3, KEY_R=1<<8, KEY_V=1<<14 }; };
}
YAW_PARAMS
namespace HAL { namespace FDCAN { struct Frame { unsigned id; unsigned dlc; bool is_extended_id; bool is_remote_frame; uint8_t data[8]; }; }}
namespace BSP { namespace Motor { namespace DM { enum Model { MIT }; }}}
struct Motor {
 int count = 0;
 int enable_count = 0, disable_count = 0, torque_count = 0;
 bool online = true;
 float torque = 0.0f;
 float angle = Rad(24.0f), speed = 0.0f;
 float target_speed = 0.0f, mit_kd = 0.0f;
 void Parse(const HAL::FDCAN::Frame&) { ++count; }
 float getAngleDeg(int) { return 24.0f; }
 float getAngleRad(int) { return angle; }
 float getVelocityRads(int) { return speed; }
 bool isConnected(int, int) { return online; }
 void On(int, BSP::Motor::DM::Model) { ++enable_count; }
 void Off(int, BSP::Motor::DM::Model) { ++disable_count; }
 void ctrl_Mit(int, float, float speed_command, float, float kd, float command) {
    ++torque_count; target_speed=speed_command; mit_kd=kd; torque=command;
 }
} pitch_motor, yaw_motor;
namespace BSP::Motor::DM { template <uint8_t> using DMMotorBase = ::Motor; }
MOTOR_ENABLE
uint8_t yaw_drive_status = 0, pitch_drive_status = 0;
void Gimbal485_UpdateYawFromISR(float) {}
Yaw_FSM yaw_fsm;
YAW_RESET
Gimbal485YawMouseInput mouse;
struct RemoteData_t {
 float chassis_vx, chassis_vy, gimbal_pitch;
 int16_t mouse_x, mouse_y, mouse_z;
 uint8_t mouse_left, mouse_right, s1, s2;
 uint16_t keyboard;
} RemoteData{};
struct RemoteStub {
 uint8_t s1=0, s2=0;
 float get_left_y() const { return 0.0f; }
 float get_left_x() const { return 0.0f; }
 float get_right_y() const { return 0.25f; }
 uint8_t get_s1() const { return s1; }
 uint8_t get_s2() const { return s2; }
 int16_t get_mouseX() const { return 0; }
 int16_t get_mouseY() const { return 7; }
 int16_t get_mouseZ() const { return 0; }
 bool get_mouseLeft() const { return false; }
 bool get_mouseRight() const { return mouse.mouse_active; }
 uint16_t get_keyboard() const { return 0; }
 void ConsumeMouseDelta(int32_t &x, int32_t &y) { x=0; y=7; }
} remoteController;
bool g_watchdog_rearm_required = false;
bool gimbal_remote_offline = false, remote_was_offline = false;
MotorRecoveryFSM pitch_recovery_fsm;
void ResetPitchControlState() {}
void UpdatePitchMotorFeedback() {}
uint32_t tick = 100;
extern "C" uint32_t HAL_GetTick() { return ++tick; }
DISPATCHER_IMPL
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
Gimbal485YawMouseInput Gimbal485_ConsumeYawMouseInput() { return mouse; }
void receive(const HAL::FDCAN::Frame &frame) { BODY }
CONTROL_STATE
void RunControlCycle(MotorRecoveryFSM &yaw_recovery_fsm) {
 // INSERT_CONTROL
    remote_was_offline = false;
 }
}
int main() {
 InputDispatcher dispatcher;
 for (uint8_t s1=Remote::UP; s1<=Remote::MIDDLE; ++s1) {
    for (uint8_t s2=Remote::UP; s2<=Remote::MIDDLE; ++s2) {
        dispatcher.Update(s1, s2, 0, false, false);
        if (s1==Remote::DOWN && s2==Remote::DOWN)
            assert(dispatcher.GetSource()==InputSource::DoubleDown);
        else if (s1==Remote::MIDDLE && s2==Remote::MIDDLE)
            assert(dispatcher.GetSource()==InputSource::KeyMouse);
        else
            assert(dispatcher.GetSource()==InputSource::Remote);
    }
 }
 HAL::FDCAN::Frame f{}; f.id=7; f.dlc=8; f.data[0]=0x17;
 receive(f); assert(yaw_motor.count==1 && pitch_motor.count==0 && yaw_drive_status==1);
 f.data[0]=0x18; receive(f); assert(yaw_motor.count==1);
 f.data[0]=7; f.dlc=7; receive(f); assert(yaw_motor.count==1);
 f.dlc=8; f.is_extended_id=true; receive(f); assert(yaw_motor.count==1);
 f.is_extended_id=false; f.is_remote_frame=true; receive(f); assert(yaw_motor.count==1);
 f.is_remote_frame=false; f.id=8; receive(f); assert(pitch_motor.count==0);
 f.data[0]=0x18; receive(f); assert(pitch_motor.count==1 && pitch_drive_status==1 && yaw_motor.count==1);

 MotorRecoveryFSM recovery;
 yaw_center_encoder_pid.setK(10.0f, 0.0f, 0.0f);
 yaw_center_kd=0.75f;
 yaw_drive_status=0;
 mouse.online=true;
 remoteController.s1=remoteController.s2=3;
 RunControlCycle(recovery);
 assert(input_dispatcher.GetSource()==InputSource::KeyMouse);
 assert(yaw_motor.enable_count==1 && yaw_motor.torque_count==0);
 assert(RemoteData.s1==3 && RemoteData.s2==3 && RemoteData.gimbal_pitch==0.25f && RemoteData.mouse_y==7);

 RunControlCycle(recovery);
 assert(yaw_motor.enable_count==1 && yaw_motor.torque_count==0);
 yaw_drive_status=1;
 yaw_motor.angle=Rad(23.0f);
 RunControlCycle(recovery);
 assert(yaw_motor.torque_count==1 && yaw_fsm.Get_Mode()==Yaw_FSM::Mode::Center);
 assert(std::fabs(yaw_fsm.Get_Target_Angle()-yaw_motor.angle)<0.00001f);
 assert(yaw_motor.target_speed==0.0f && yaw_motor.mit_kd==yaw_center_kd);
 RunControlCycle(recovery);
 assert(yaw_motor.target_speed>0.0f && yaw_fsm.Get_Target_Angle()<Rad(24.0f));
 yaw_motor.angle=Rad(24.0f);
 mouse.mouse_active=true;
 mouse.mouse_delta_x=8;
 RunControlCycle(recovery);
 assert(yaw_motor.torque_count==3 && yaw_motor.mit_kd==yaw_angle_kd && yaw_motor.torque==0.0f);
 assert(std::fabs(yaw_motor.target_speed - 26.5f*Rad(0.2f)) < 0.00001f);

 mouse.mouse_delta_x=1000;
 const float yaw_torque_limit=yaw_fsm.Limit_Torque(20.0f, yaw_motor.angle);
 yaw_motor.speed=-yaw_torque_limit/yaw_angle_kd;
 RunControlCycle(recovery);
 assert(yaw_motor.mit_kd==0.0f && std::fabs(yaw_motor.torque-yaw_torque_limit)<0.00001f);
 yaw_motor.angle=Rad(28.5f);
 yaw_motor.speed=-0.2f; // A zero speed request alone would push outward through MIT Kd.
 RunControlCycle(recovery);
 assert(yaw_motor.mit_kd==0.0f && yaw_motor.torque==0.0f);
 mouse.mouse_delta_x=-1000;
 yaw_motor.speed=0.0f;
 RunControlCycle(recovery);
 assert(yaw_motor.mit_kd==0.0f && yaw_motor.torque<0.0f);
 yaw_motor.angle=Rad(24.0f);
 const int commands_before_loss=yaw_motor.torque_count;

 mouse={}; // Losing the board link is not an operator release.
 RunControlCycle(recovery);
 assert(yaw_motor.disable_count==1 && pitch_motor.disable_count==1 && !yaw_fsm.Is_Target_Valid());
 assert(RemoteData.s1==0 && RemoteData.s2==0 && RemoteData.mouse_y==0);
 yaw_drive_status=0;
 mouse.online=true;
 remoteController.s1=remoteController.s2=3;
 mouse.mouse_active=true;
 mouse.reset=true;
 RunControlCycle(recovery);
 assert(yaw_motor.enable_count==2 && yaw_motor.torque_count==commands_before_loss);
 yaw_drive_status=1;
 mouse.mouse_delta_x=100;
 RunControlCycle(recovery);
 assert(yaw_motor.disable_count==1 && !yaw_fsm.Is_Target_Valid());
 assert(yaw_motor.target_speed==0.0f && yaw_motor.mit_kd==0.0f && yaw_motor.torque==0.0f);

 mouse.mouse_active=false;
 mouse.mouse_delta_x=0;
 mouse.reset=false;
 RunControlCycle(recovery);
 mouse.mouse_active=true;
 RunControlCycle(recovery);
 assert(yaw_motor.enable_count==2);
 assert(yaw_fsm.Get_Mode()==Yaw_FSM::Mode::Mouse);
 mouse.mouse_active=false;
 yaw_motor.angle=Rad(26.0f);
 RunControlCycle(recovery);
 assert(yaw_fsm.Is_Target_Valid());
 yaw_motor.angle=Rad(26.2f);
 RunControlCycle(recovery);
 assert(std::fabs(yaw_motor.target_speed -
                 10.0f * (yaw_fsm.Get_Target_Angle() - yaw_motor.angle)) < 0.00001f);
 yaw_center_encoder_pid.setK(26.5f, 0.0f, 0.0f);
 yaw_motor.angle=Rad(27.0f);
 yaw_motor.speed=0.0f;
 RunControlCycle(recovery);
 assert(yaw_motor.mit_kd==yaw_center_kd && std::fabs(yaw_motor.target_speed+Rad(15.0f))<0.00001f);
 yaw_motor.online=false;
 RunControlCycle(recovery);
 assert(yaw_motor.disable_count==2 && pitch_motor.disable_count==1 && !yaw_fsm.Is_Target_Valid());
 assert(yaw_motor.enable_count==2);

 // A normal RC gear centers even when there has been no mouse press since STOP.
 mouse={};
 mouse.online=true;
 remoteController.s1=1; remoteController.s2=3;
 yaw_motor.online=true;
 yaw_drive_status=1;
 yaw_motor.angle=Rad(26.0f);
 yaw_motor.speed=0.0f;
 RunControlCycle(recovery);
 assert(yaw_fsm.Get_Mode()==Yaw_FSM::Mode::Center);
 assert(input_dispatcher.GetSource()==InputSource::Remote);
 assert(std::fabs(yaw_fsm.Get_Target_Angle()-Rad(26.0f))<0.00001f);
 RunControlCycle(recovery);
 assert(yaw_motor.target_speed<0.0f && yaw_motor.mit_kd==yaw_center_kd);

 // Double-down clears the target and sends Off without further MIT commands.
 remoteController.s1=remoteController.s2=2;
 const int commands_before_stop=yaw_motor.torque_count;
 const int pitch_commands_before_stop=pitch_motor.torque_count;
 RunControlCycle(recovery);
 RunControlCycle(recovery);
 assert(input_dispatcher.GetSource()==InputSource::DoubleDown);
 assert(yaw_fsm.Get_Mode()==Yaw_FSM::Mode::Stop);
 assert(yaw_motor.torque_count==commands_before_stop && yaw_motor.enable_count==2);
 assert(yaw_motor.disable_count==3 && pitch_motor.disable_count==2);
 assert(pitch_motor.torque_count==pitch_commands_before_stop);

 // Leaving STOP for a normal RC gear requests enable if the drive is disabled.
 remoteController.s1=1; remoteController.s2=3;
 yaw_drive_status=0;
 RunControlCycle(recovery);
 assert(yaw_motor.enable_count==3 && !yaw_fsm.Is_Target_Valid());

 // A locked watchdog takes priority over enable requests.
 g_watchdog_rearm_required=true;
 remoteController.s1=remoteController.s2=2;
 RunControlCycle(recovery);
 assert(yaw_motor.disable_count==4 && pitch_motor.disable_count==3);
 assert(yaw_motor.torque_count==commands_before_stop && yaw_motor.enable_count==3);
 assert(pitch_motor.torque_count==pitch_commands_before_stop);

 // An out-of-travel encoder angle blocks motion while leaving the drive enabled.
 g_watchdog_rearm_required=false;
 remoteController.s1=1; remoteController.s2=3;
 yaw_motor.angle=Rad(-61.7f);
 RunControlCycle(recovery);
 assert(yaw_motor.enable_count==4 && yaw_motor.disable_count==4);
 yaw_drive_status=1;
 const float out_of_travel[] = {Rad(-61.7f), Rad(29.2f)};
 for (float angle : out_of_travel) {
    yaw_motor.angle=angle;
    const int previous_commands=yaw_motor.torque_count;
    RunControlCycle(recovery);
    assert(yaw_motor.enable_count==4 && yaw_motor.disable_count==4);
    assert(yaw_motor.torque_count==previous_commands+1 && !yaw_fsm.Is_Target_Valid());
    assert(yaw_motor.target_speed==0.0f && yaw_motor.mit_kd==0.0f && yaw_motor.torque==0.0f);
 }
 yaw_motor.angle=Rad(26.0f);
 RunControlCycle(recovery);
 assert(yaw_fsm.Get_Mode()==Yaw_FSM::Mode::Center);
 assert(std::fabs(yaw_fsm.Get_Target_Angle()-yaw_motor.angle)<0.00001f);

 // Returning from the limit with the right button held reanchors automatically.
 remoteController.s1=remoteController.s2=3;
 mouse.mouse_active=false;
 RunControlCycle(recovery);
 mouse.mouse_active=true;
 RunControlCycle(recovery);
 assert(yaw_fsm.Get_Mode()==Yaw_FSM::Mode::Mouse);
 yaw_motor.angle=Rad(29.2f);
 mouse.mouse_delta_x=100;
 RunControlCycle(recovery);
 assert(yaw_motor.disable_count==4 && !yaw_fsm.Is_Target_Valid());
 assert(yaw_motor.target_speed==0.0f && yaw_motor.mit_kd==0.0f && yaw_motor.torque==0.0f);
 yaw_motor.angle=Rad(26.0f);
 mouse.mouse_delta_x=0;
 RunControlCycle(recovery);
 assert(yaw_motor.disable_count==4 && yaw_fsm.Get_Mode()==Yaw_FSM::Mode::Mouse);
 assert(std::fabs(yaw_fsm.Get_Target_Angle()-yaw_motor.angle)<0.00001f);

 yaw_motor.angle=std::nanf("");
 RunControlCycle(recovery);
 assert(yaw_motor.disable_count==5 && !yaw_fsm.Is_Target_Valid());

 remoteController.s1=0;
 yaw_drive_status=0;
 RunControlCycle(recovery);
 assert(gimbal_remote_offline && yaw_motor.enable_count==4);
 assert(yaw_motor.disable_count==6 && pitch_motor.disable_count==4);
 assert(RemoteData.s1==0 && RemoteData.s2==0);
}
'''.replace('YAW_PARAMS', params).replace('YAW_RESET', yaw_reset).replace('MOTOR_ENABLE', motor_enable).replace('BODY', body).replace(
    'CONTROL_STATE', control_state).replace('// INSERT_CONTROL', control).replace('DISPATCHER_IMPL', dispatcher)
with tempfile.TemporaryDirectory() as tmp:
    source = Path(tmp) / 'test.cpp'
    source.write_text(program)
    exe = Path(tmp) / 'test.exe'
    subprocess.run(['g++', '-std=c++17', '-I.', '-Itests/stubs', '-Icommunication_between_boards', source.as_posix(),
                    'feeder_fsm/yaw_fsm.cpp', 'user/core/Alg/PID/pid.cpp',
                    '-o', exe.as_posix()], cwd=root, check=True)
    subprocess.run([str(exe)], check=True)
