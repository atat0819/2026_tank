"""Exercise cached UART5 mouse and stick input through the chassis rotation FSM."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
header = (ROOT / "RtosTask/can_send_task.hpp").read_text(encoding="utf-8-sig")
assert "gimbal_mouse_delta_pending" not in source, "chassis must hold latest mouse X state instead of consuming a delta"
assert "phase_comp = (0.01f * wz_cmd)" not in source, "relative gimbal rotation compensation remains"
assert "-input.wz * MAX_WZ_RAD_S" not in source, "joystick yaw calculation must live in the FSM"
assert "wz_cmd = -static_cast<float>(input.mouse_x)" not in source, "mouse yaw calculation must live in the FSM"
assert "gyro_enabled" not in source, "CTRL must not select a gyro rotation output"
keyboard_header = (ROOT / "fsm/chassis_keyboard_fsm.hpp").read_text(encoding="utf-8-sig")
keyboard_source = (ROOT / "fsm/chassis_keyboard_fsm.cpp").read_text(encoding="utf-8-sig")
assert "gyro_enabled" not in keyboard_header + keyboard_source, "remove the keyboard gyro toggle"
snapshot_type = header[header.index("struct ControlInputSnapshot"):header.index("ControlInputSnapshot GetControlInputSnapshot")]
publication = source[source.index("float yaw_offset_deg ="):source.index("StairCommand stair_command;")]
begin = source.index("chassis_fsm.StateUpdate(")
turn = source[begin:source.index("        // 2. 获取电机当前反馈", begin)]
turn_limit = re.search(r"^\s*wz_cmd = std::clamp\(wz_cmd, -MAX_WZ_RAD_S, MAX_WZ_RAD_S\);", source, re.M).group(0)
turn_constant = re.search(r"constexpr float MAX_WZ_RAD_S = [^;]+;", source).group(0)
harness = r'''
#include <cassert>
#include <algorithm>
#include <cmath>
#include "communication/boards_communication_uart_485.hpp"
#include "fsm/control_input_selector.hpp"
#include "fsm/chassis_fsm.hpp"
#include "fsm/chassis_keyboard_fsm.hpp"
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
TURN_CONSTANT
constexpr float MOUSE_RATE = 1.0f * M_PI / 180.0f;
uint32_t tick=100;
uint32_t HAL_GetTick() { return tick; }
bool local_remote_received=false;
volatile uint32_t g_watchdog_rearm_required=0;
uint32_t local_remote_last_tick=0;
uint16_t local_remote_keyboard=0;
int16_t local_remote_mouse_x=0;
bool local_remote_yaw_mouse_active=false;
struct { uint8_t s1=1,s2=1; float vx=0,vy=0,wz=0; } remoteData;
SNAPSHOT_TYPE
PUBLICATION
Chassis_FSM chassis_fsm;
ChassisKeyboardFSM keyboard_fsm;
struct { float wz; } ChassisData;
void publish(float stick,int16_t mouse,uint8_t s1=3,uint8_t s2=3,uint16_t keys=0) {
    local_remote_received=s1>=1 && s1<=3 && s2>=1 && s2<=3;
    local_remote_last_tick=tick;
    remoteData.s1=s1; remoteData.s2=s2; remoteData.wz=stick;
    local_remote_mouse_x=mouse; local_remote_keyboard=keys;
}
float step(bool source_changed=false) {
    const uint32_t now_tick=tick;
    const auto input=GetControlInputSnapshot();
    const bool keyboard_mode=input.s1==3 && input.s2==3;
    keyboard_fsm.Update(input.keyboard,keyboard_mode,input.keyboard_online && !source_changed,now_tick);
    const auto& keyboard_cmd=keyboard_fsm.GetCommand();
    float yaw_offset_rad=0,wz_cmd=0;
    TURN
    TURN_LIMIT
    return wz_cmd;
}
bool near(float a,float b) { return std::fabs(a-b)<0.00001f; }
int main() {

    chassis_fsm.Init();
    publish(0,10);
    for (int i=0;i<5;++i) {
        const auto snapshot=GetControlInputSnapshot();
        assert(snapshot.source==ControlInputSource::LOCAL && snapshot.mouse_x==10);
        assert(near(step(),-10*MOUSE_RATE));
    }
    publish(0,10); assert(near(step(),-10*MOUSE_RATE)); // Equal packets never sum.
    local_remote_yaw_mouse_active=true;
    assert(step()==0);
    local_remote_yaw_mouse_active=false;
    assert(near(step(),-10*MOUSE_RATE));
    publish(0,-3); assert(near(step(),3*MOUSE_RATE));
    assert(near(step(),3*MOUSE_RATE));
    publish(0,0); assert(step()==0); assert(step()==0);
    // Multiple packets between task cycles retain only the newest sample.
    publish(0,30000); publish(0,-1);
    assert(near(step(),MOUSE_RATE));
    publish(0,32767); assert(step()==-MAX_WZ_RAD_S); assert(step()==-MAX_WZ_RAD_S);
    publish(0,-32768); assert(step()==MAX_WZ_RAD_S); assert(step()==MAX_WZ_RAD_S);
    publish(1.0f,0,2,3);
    assert(GetControlInputSnapshot().wz==1);
    assert(step()==-MAX_WZ_RAD_S); assert(step()==-MAX_WZ_RAD_S);
    publish(-0.5f,0,2,3); assert(step()==0.5f*MAX_WZ_RAD_S);
    publish(0,40); publish(0,0,2,3); publish(0,0); assert(step()==0);
    publish(0,40); publish(0,0,0,0);
    assert(GetControlInputSnapshot().source==ControlInputSource::NONE);
    assert(GetControlInputSnapshot().mouse_x==0 && step()==0);
    publish(0,0); assert(step()==0);
    publish(0,40); tick+=100; assert(step()==0);
    publish(0,0); assert(step()==0);
    publish(0,40); tick+=100; publish(0,2);
    assert(near(step(),-2*MOUSE_RATE)); assert(near(step(),-2*MOUSE_RATE));
    publish(0,30); Chassis485_ApplyData({17.0f});
    assert(near(step(),-30*MOUSE_RATE)); // Yaw replies do not affect chassis input.
    tick+=100; Chassis485_ApplyData({18.0f});
    assert(yaw_offset_deg==18.0f && step()==0);
    publish(0,30); assert(step(true)==0); // Reconnection gate pauses one cycle.
    assert(near(step(),-30*MOUSE_RATE));
    // CTRL press/hold/release and a second press cannot override the mouse output.
    for(uint16_t keys : {uint16_t(0),uint16_t(ChassisKeyboardFSM::KEY_CTRL),
                         uint16_t(ChassisKeyboardFSM::KEY_CTRL),uint16_t(0),
                         uint16_t(ChassisKeyboardFSM::KEY_CTRL)}) {
        ++tick; publish(1,10,3,3,keys); assert(near(step(),-10*MOUSE_RATE));
        tick+=20; publish(1,10,3,3,keys); assert(near(step(),-10*MOUSE_RATE));
        assert(chassis_fsm.Get_Mode()==CHASSIS_MOUSE_ROTATE);
    }
    publish(1,0,3,3,ChassisKeyboardFSM::KEY_CTRL); assert(step()==0);
    tick=0xfffffff0U; publish(0,3); tick=0x20;
    assert(near(step(),-3*MOUSE_RATE)); assert(near(step(),-3*MOUSE_RATE));
    tick=0x54; assert(step()==0);
    // Z is ignored; the real keyboard FSM still accepts CTRL without creating an action.
    ChassisKeyboardFSM keyboard;
    keyboard.Init(); keyboard.Update(0,true,true,0); keyboard.Update(0,true,true,20);
    assert(!keyboard.GetCommand().follow_enabled);
    keyboard.Update(ChassisKeyboardFSM::KEY_Z,true,true,21);
    keyboard.Update(ChassisKeyboardFSM::KEY_Z,true,true,41);
    assert(!keyboard.GetCommand().follow_enabled);
    keyboard.Update(ChassisKeyboardFSM::KEY_CTRL,true,true,42);
    keyboard.Update(ChassisKeyboardFSM::KEY_CTRL,true,true,62);
    assert(keyboard.GetCommand().valid);
    assert(keyboard.GetCommand().vx==0 && keyboard.GetCommand().vy==0);
    assert(!keyboard.GetCommand().stair_toggle && !keyboard.GetCommand().imu_mode_request);
}
'''.replace("SNAPSHOT_TYPE", snapshot_type).replace("PUBLICATION", publication).replace("TURN_CONSTANT", turn_constant).replace("TURN_LIMIT", turn_limit).replace("TURN", turn)
with tempfile.TemporaryDirectory(prefix="chassis_yaw_") as directory:
    work=Path(directory)
    (work / "main.h").write_text('#pragma once\n#include <cstdint>\nuint32_t HAL_GetTick();\n', encoding="utf-8")
    cpp,exe=work/"test.cpp",work/"test.exe"
    cpp.write_text(harness, encoding="utf-8")
    subprocess.run(["g++","-std=c++17","-I",work.as_posix(),"-I",ROOT.as_posix(),
                    "-I",(ROOT/"user/core").as_posix(),cpp.as_posix(),
                    (ROOT/"fsm/chassis_fsm.cpp").as_posix(),
                    (ROOT/"fsm/chassis_keyboard_fsm.cpp").as_posix(),
                    (ROOT/"user/core/Alg/FSM/alg_fsm.cpp").as_posix(),
                    (ROOT/"user/core/Alg/PID/pid.cpp").as_posix(),"-o",exe.as_posix()],check=True)
    subprocess.run([str(exe)],check=True)
print("PASS: UART5 chassis yaw, mouse cache, independent Yaw replies, expiry and CTRL/Z")
