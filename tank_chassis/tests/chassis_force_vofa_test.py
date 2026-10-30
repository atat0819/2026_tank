"""Execute actual force telemetry capture/publication and USB JustFloat transport."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
source=(ROOT/"RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
code=re.sub(r"/\*.*?\*/|//[^\n]*","",source,flags=re.S)
assert "vofa_sendN(force_vofa_channels, 15);" in code, "force USB telemetry is missing"
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
init=re.search(r"float force_vofa_channels\[15\] = \{\};",source).group(0)
start=source.index("// 保存本周期力控目标与反馈")
capture=source[start:source.index("// 仅保留目标轮速",start)]
start=source.index("// 力控 VOFA：")
publish=source[start:source.index("// 原功率辨识 VOFA 暂停",start)]
# Publication must remain in the common loop, outside drive-enable/STOP branches.
prefix=re.sub(r"/\*.*?\*/|//[^\n]*","",source[:start],flags=re.S)
assert prefix.count("{")-prefix.count("}")==2
assert "power_meter" not in publish, "force stream must not depend on power-meter frames"

host=r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include "user/core/Alg/ChassisCalculation/MecanumForceControl.hpp"
using Alg::CalculationBase::MecanumForceControl;
MecanumForceControl chassis_force;
float current_speed_rads[4]={};
uint32_t last_force_vofa_tick=0;
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
void sample(uint32_t now_tick,bool enabled,bool wheel_feedback_valid,float delta=0) {
'''+init+r'''
    if(enabled) {
        const float force_target[3]={.3f+delta,-.2f,.4f};
        const float force_feedback[3]={.2f,-.1f,.3f};
        for(unsigned axis=0;axis<3;++axis) {
            chassis_force.force[axis]=10+axis;
            chassis_force.integral[axis]=1+axis;
        }
        chassis_force.output_scale=.4f;
'''+capture+r'''
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
    sample(14,true,true);assert(sends==0); // Four milliseconds since last attempt.
    sample(15,true,true);assert(sends==1&&length==64);
    const float expected[15]={.3f,.2f,-.2f,-.1f,.4f,.3f,10,11,12,1,2,3,.4f,1,.015f};
    for(unsigned i=0;i<15;++i) near(channel(i),expected[i]);
    const uint8_t tail[4]={0,0,0x80,0x7f};assert(std::memcmp(active+60,tail,4)==0);
    uint8_t saved[64];std::memcpy(saved,active,64);
    sample(20,true,true,1);assert(sends==1&&std::memcmp(active,saved,64)==0);
    cdc.TxState=0;
    const float moving[3]={1,-2,.5f};MecanumForceControl::InverseKinematics(moving,current_speed_rads);
    sample(25,false,true);assert(sends==2); // STOP keeps genuine moving feedback.
    for(unsigned axis=0;axis<3;++axis) {
        near(channel(2*axis),0);near(channel(2*axis+1),moving[axis]);
        near(channel(6+axis),0);near(channel(9+axis),0);
    }
    near(channel(12),0);near(channel(13),0);near(channel(14),.025f);
    cdc.TxState=0;sample(30,false,false);assert(sends==3);
    for(unsigned i=0;i<13;++i) near(channel(i),0);
    near(channel(13),-1); // Invalid wheel feedback is explicitly marked.
    cdc.TxState=0;sample(35,true,true);assert(sends==4);near(channel(13),1);
    cdc.TxState=0;last_force_vofa_tick=0xfffffffdU;
    sample(1,false,true);assert(sends==4);
    sample(2,false,true);assert(sends==5);near(channel(14),.002f);
    cdc.TxState=0;
    vofa_sendN(nullptr,15);vofa_sendN(expected,0);vofa_sendN(expected,16);
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
print("PASS: 15 force channels, USB framing/busy state, 200 Hz timing, STOP/fault/recovery and tick wrap")
