"""Execute actual force telemetry capture/publication and USB JustFloat transport."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
source=(ROOT/"RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
code=re.sub(r"/\*.*?\*/|//[^\n]*","",source,flags=re.S)
assert "vofa_sendN(force_vofa_channels, 24);" in code, "24-channel force USB telemetry is missing"
assert "vofa_sendN(power_channels, 15);" in source, "original power telemetry must be retained"
assert "vofa_sendN(power_channels, 15);" not in code, "old power stream is still active"

def function(anchor):
    start=source.index(anchor)
    pos=source.index("{",start)+1
    end,depth=pos,1
    while depth:
        depth+=(source[end]=="{")-(source[end]=="}")
        end+=1
    return source[start:end]

usb=function("void vofa_sendN(const float *data, uint8_t count)")
start=source.index("// 力控 VOFA：")
publish=source[start:source.index("// 原功率辨识 VOFA 暂停",start)]
# Publication must remain in the common loop, outside drive-enable/STOP branches.
prefix=re.sub(r"/\*.*?\*/|//[^\n]*","",source[:start],flags=re.S)
assert prefix.count("{")-prefix.count("}")==2
assert "force_vofa_channels[" not in prefix, "all channel assignments belong in the send block"
assert "last_force_vofa_tick" not in code, "send once per task iteration"
assert "power_meter" not in publish and "PowerData" not in publish, \
    "force stream must not depend on power-meter frames"

host=r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include "user/core/Alg/ChassisCalculation/MecanumForceControl.hpp"
using Alg::CalculationBase::MecanumForceControl;
MecanumForceControl chassis_force;
float current_speed_rads[4]={};
int critical=0;
#define taskENTER_CRITICAL() (++critical)
#define taskEXIT_CRITICAL() (--critical)
struct USBD_CDC_HandleTypeDef { unsigned TxState=0; } cdc;
enum { USBD_STATE_DEFAULT,USBD_STATE_CONFIGURED };
struct { void *pClassData=nullptr;int dev_state=USBD_STATE_DEFAULT; } hUsbDeviceHS;
uint8_t *active=nullptr;
unsigned sends=0,length=0;
void CDC_Transmit_HS(uint8_t *data,uint16_t size) {
    assert(critical==1);active=data;length=size;++sends;cdc.TxState=1;
}
'''+usb+r'''
void sample(uint32_t now_tick,bool force_control_active,bool wheel_feedback_valid,float delta=0) {
    float force_target[3]={},force_feedback[3]={};
    const float chassis_velocity_feedback[3]={.24f,-.13f,.37f};
    float motor_output[4]={};
    float motor_target_speed[4]={};
    if(force_control_active) {
        const float target[3]={.3f+delta,-.2f,.4f};
        const float filtered[3]={.2f,-.1f,.3f};
        chassis_force.sync_current[0]=-2.25f;
        motor_output[0]=1.5f*MecanumForceControl::RAW_PER_AMP;
        motor_target_speed[0]=-5;
        motor_target_speed[1]=6;
        motor_target_speed[2]=7;
        motor_target_speed[3]=-8;
        for(unsigned axis=0;axis<3;++axis) {
            force_target[axis]=target[axis];
            force_feedback[axis]=filtered[axis];
            chassis_force.force[axis]=10+axis;
            chassis_force.integral[axis]=1+axis;
        }
    } else chassis_force.Reset();
'''+publish+r'''
    assert(critical==0);
}
float channel(unsigned index) {
    float value;std::memcpy(&value,active+index*sizeof(float),sizeof(value));return value;
}
void near(float actual,float expected) {
    assert(std::isfinite(actual)&&std::fabs(actual-expected)<.0001f);
}
int main() {
    sample(5,true,true);assert(sends==0); // USB unconfigured.
    hUsbDeviceHS.dev_state=USBD_STATE_CONFIGURED;
    sample(10,true,true);assert(sends==0); // No CDC class instance.
    hUsbDeviceHS.pClassData=&cdc;
    current_speed_rads[0]=-4.5f;current_speed_rads[1]=5.5f;
    current_speed_rads[2]=6.5f;current_speed_rads[3]=-7.5f;
    sample(10,true,true);assert(sends==1&&length==100);
    const float expected[24]={
        .3f,.24f,.2f,-.2f,-.13f,-.1f,.4f,.37f,.3f,10,11,12,1,.010f,-2.25f,1.5f,
        -5,-4.5f,6,5.5f,7,6.5f,-8,-7.5f
    };
    for(unsigned i=0;i<24;++i) near(channel(i),expected[i]);
    const uint8_t tail[4]={0,0,0x80,0x7f};assert(std::memcmp(active+96,tail,4)==0);
    uint8_t saved[100];std::memcpy(saved,active,100);
    sample(10,true,true,1);assert(sends==1&&std::memcmp(active,saved,100)==0);
    cdc.TxState=0;
    sample(10,true,true,1);assert(sends==2);near(channel(0),1.3f);
    cdc.TxState=0;
    const float moving[3]={1,-2,.5f};MecanumForceControl::InverseKinematics(moving,current_speed_rads);
    sample(11,false,true);assert(sends==3); // STOP keeps genuine moving feedback.
    for(unsigned axis=0;axis<3;++axis) {
        near(channel(3*axis),0);near(channel(3*axis+1),moving[axis]);near(channel(3*axis+2),0);
        near(channel(9+axis),0);
    }
    near(channel(12),0);near(channel(13),.011f);
    near(channel(14),0);near(channel(15),0);
    for(unsigned i=0;i<4;++i) {
        near(channel(16+2*i),0);
        near(channel(17+2*i),current_speed_rads[i]);
    }
    current_speed_rads[2]=NAN;
    cdc.TxState=0;sample(12,false,false);assert(sends==4);
    for(unsigned i=0;i<12;++i) near(channel(i),0);
    near(channel(12),-1); // Invalid wheel feedback is explicitly marked.
    near(channel(14),0);near(channel(15),0);
    for(unsigned i=16;i<24;++i) near(channel(i),0);
    current_speed_rads[2]=0;
    cdc.TxState=0;sample(13,true,true);assert(sends==5);near(channel(12),1);
    cdc.TxState=0;
    const float too_many[31]={};
    vofa_sendN(nullptr,24);vofa_sendN(expected,0);vofa_sendN(too_many,31);
    assert(sends==5&&critical==0);
}
'''
with tempfile.TemporaryDirectory(prefix="force-vofa-") as directory:
    cpp=Path(directory)/"test.cpp"
    exe=Path(directory)/"test.exe"
    cpp.write_text(host,encoding="utf-8")
    subprocess.run(["g++","-std=c++17","-Wall","-Wextra","-I",ROOT.as_posix(),
                    cpp.as_posix(),"-o",exe.as_posix()],check=True)
    subprocess.run([str(exe)],check=True)
print("PASS: 24 force channels including four wheel speed pairs, USB framing/busy and STOP/fault/recovery")
