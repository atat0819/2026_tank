"""Compile actual board publication, mouse consumption and chassis yaw selection."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
header = (ROOT / "RtosTask/can_send_task.hpp").read_text(encoding="utf-8-sig")
assert "int16_t right_stick_x" in header, "missing chassis right-stick storage"
assert "gimbal_mouse_delta_pending" not in source, "chassis must hold latest mouse X state instead of consuming a delta"
assert "phase_comp = (0.01f * wz_cmd)" not in source, "relative gimbal rotation compensation remains"
state_type = re.search(r"typedef struct\s*\{[^}]+\}\s*Gimbal_Chassis_communicate_t;", header).group(0)
snapshot_type = header[header.index("struct ControlInputSnapshot"):header.index("ControlInputSnapshot GetControlInputSnapshot")]
publication = source[source.index("float yaw_offset_deg ="):source.index("StairCommand stair_command;")]
begin = source.index("yaw_offset_rad = input.yaw_offset_deg")
turn = source[begin:source.index("        // 2. 获取电机当前反馈", begin)]
harness = r'''
#include <cassert>
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
uint32_t tick=100;
uint32_t HAL_GetTick() { return tick; }
bool local_remote_received=false;
uint32_t local_remote_last_tick=0;
uint16_t local_remote_keyboard=0;
int16_t local_remote_mouse_x=0;
struct { uint8_t s1=1,s2=1; float vx=0,vy=0,wz=0; } remoteData;
STATE_TYPE
SNAPSHOT_TYPE
PUBLICATION
Chassis_FSM chassis_fsm;
float c=3.0f;
void publish(int16_t stick,int16_t mouse,uint8_t s1=3,uint8_t s2=3) {
    Chassis485Data data={};
    data.right_stick_x=stick; data.mouse_x=mouse; data.s1=s1; data.s2=s2;
    Chassis485_ApplyData(data);
}
float step(bool source_changed=false,bool gyro=false) {
    const uint32_t now_tick=tick;
    const auto input=GetControlInputSnapshot(now_tick);
    const bool keyboard_mode=input.s1==3 && input.s2==3;
    KeyboardMotionCommand keyboard_cmd={};
    keyboard_cmd.valid=keyboard_mode && input.source!=ControlInputSource::NONE && !source_changed;
    keyboard_cmd.gyro_enabled=gyro;
    float yaw_offset_rad=0,wz_cmd=0;
    TURN
    return wz_cmd;
}
bool near(float a,float b) { return std::fabs(a-b)<0.00001f; }
int main() {

    chassis_fsm.Init();
    publish(0,10);
    for (int i=0;i<5;++i) {
        const auto snapshot=GetControlInputSnapshot(tick);
        assert(snapshot.source==ControlInputSource::GIMBAL && snapshot.mouse_x==10);
        assert(near(step(),-20*M_PI/180));
    }
    publish(0,10); assert(near(step(),-20*M_PI/180)); // Equal packets never sum.
    publish(0,-3); assert(near(step(),6*M_PI/180));
    assert(near(step(),6*M_PI/180));
    publish(0,0); assert(step()==0); assert(step()==0);
    // Multiple packets between task cycles retain only the newest sample.
    publish(0,30000); publish(0,-1);
    assert(near(step(),2*M_PI/180));
    publish(0,32767); assert(step()==-4); assert(step()==-4);
    publish(0,-32768); assert(step()==4); assert(step()==4);
    publish(660,0,2,3);
    assert(GetControlInputSnapshot(tick).wz==1);
    assert(step()==-3); assert(step()==-3);
    publish(-330,0,2,3); assert(step()==1.5f);
    publish(0,40); publish(0,0,2,3); publish(0,0); assert(step()==0);
    publish(0,40); publish(0,0,0,0);
    assert(GetControlInputSnapshot(tick).source==ControlInputSource::NONE);
    assert(GetControlInputSnapshot(tick).mouse_x==0 && step()==0);
    publish(0,0); assert(step()==0);
    publish(0,40); tick+=100; assert(step()==0);
    publish(0,0); assert(step()==0);
    publish(0,40); tick+=100; publish(0,2);
    assert(near(step(),-4*M_PI/180)); assert(near(step(),-4*M_PI/180));
    // LOCAL and GIMBAL now use identical cached values and the exact same yaw expression.
    const int16_t values[]={0,1,-1,10,-10,32767,-32768};
    for (int16_t value : values) {
        local_remote_received=false; publish(0,value);
        const float gimbal_wz=step();
        local_remote_received=true; local_remote_last_tick=tick;
        remoteData.s1=remoteData.s2=3; local_remote_mouse_x=value;
        assert(GetControlInputSnapshot(tick).source==ControlInputSource::LOCAL);
        assert(step()==gimbal_wz && step()==gimbal_wz);
    }
    publish(0,0); assert(step()==4); // LOCAL still has -32768 and takes priority.
    local_remote_received=false; assert(step()==0); // Cloud zero superseded the old state.
    publish(0,30); assert(step(true)==0); // Source-change gate pauses one cycle.
    assert(near(step(),-60*M_PI/180));
    assert(step(false,true)==chassis_fsm.gyro_spin_speed);
    publish(0,0); assert(step(false,true)==chassis_fsm.gyro_spin_speed);
    assert(step()==0); // Latest zero also wins while gyro owns the output.
    tick=0xfffffff0U; publish(0,3); tick=0x20;
    assert(near(step(),-6*M_PI/180)); assert(near(step(),-6*M_PI/180));
    tick=0x54; assert(step()==0);
    // Z is ignored; CTRL gyro and V/C/B actions remain handled by the real FSM.
    ChassisKeyboardFSM keyboard;
    keyboard.Init(); keyboard.Update(0,true,true,0); keyboard.Update(0,true,true,20);
    assert(!keyboard.GetCommand().follow_enabled);
    keyboard.Update(ChassisKeyboardFSM::KEY_Z,true,true,21);
    keyboard.Update(ChassisKeyboardFSM::KEY_Z,true,true,41);
    assert(!keyboard.GetCommand().follow_enabled);
    keyboard.Update(ChassisKeyboardFSM::KEY_CTRL,true,true,42);
    keyboard.Update(ChassisKeyboardFSM::KEY_CTRL,true,true,62);
    assert(keyboard.GetCommand().gyro_enabled);
}
'''.replace("STATE_TYPE", state_type).replace("SNAPSHOT_TYPE", snapshot_type).replace("PUBLICATION", publication).replace("TURN", turn)
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
print("PASS: chassis yaw sources, latest mouse cache, LOCAL/GIMBAL equivalent yaw, zero/expiry gates and Z disabled")
