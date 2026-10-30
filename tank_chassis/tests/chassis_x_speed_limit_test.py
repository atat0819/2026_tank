"""Exercise real input transforms and force targets with the real ramp/filter."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
source=(ROOT/"RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
limits="\n".join(re.search(r"constexpr float "+name+r"\s*=[^;]+;",source).group(0)
                   for name in ("CHASSIS_MAX_VX_M_S","MAX_VY_M_S","MAX_WZ_RAD_S"))
filters="\n".join(re.search(r"SecondOrderLPFFilter chassis_"+axis+r"_filter\([^;]+;",source).group(0)
                  for axis in ("vx","vy","wz"))
start=source.index("if (keyboard_cmd.valid)",source.index("// --- A. 运动学逆解算"))
transform=source[start:source.index("/**************************************************************** */",start)]
start=source.index("// 四轮车体力控")
target=source[start:source.index("chassis_force.Update(",start)]
start=source.index("wz_cmd = chassis_fsm.Get_wz_cmd(")
yaw_selection=source[start:source.index("phase_comp = 0.0f;",start)]
host=r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include "user/core/Alg/ChassisCalculation/MecanumForceControl.hpp"
#include "user/core/Alg/Filter/Filter.hpp"
#include "user/core/Alg/UtilityFunction/SlopePlanning.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
using Alg::CalculationBase::MecanumForceControl;
'''+limits+r'''
void run(float in_x,float in_y,float in_w,float angle,bool keyboard,float result[3]) {
    struct { bool valid;float vx,vy; } keyboard_cmd={keyboard,in_x,in_y};
    struct { float vx,vy; } input={in_x,in_y};
    float vx_gimbal=0,vy_gimbal=0,vx_body=0,vy_body=0;
    float yaw_offset_rad=angle,phase_comp=0,wz_cmd=in_w;
'''+transform+r'''
    MecanumForceControl chassis_force;
    float current_speed_rads[4]={},chassis_velocity_feedback[3]={};
'''+filters+r'''
    Alg::Utility::SlopePlanning ramp_vx(100,100),ramp_vy(100,100);
'''+target+r'''
    for(unsigned i=0;i<3;++i) result[i]=force_target[i];
}
void near(float actual,float expected) { assert(std::isfinite(actual)&&std::fabs(actual-expected)<.0001f); }
enum class ControlInputSource { LOCAL, GIMBAL, NONE };
enum { CHASSIS_NOT_FOLLOW, CHASSIS_GYRO_SPIN };
float selected_yaw(float stick,int16_t mouse,bool keyboard,bool gyro,bool spin,ControlInputSource source) {
    struct { ControlInputSource source;float wz;int16_t mouse_x; } input={source,stick,mouse};
    struct { bool valid,gyro_enabled; } keyboard_cmd={keyboard,gyro};
    struct State {
        int mode;
        float gyro_spin_speed=4;
        float Get_wz_cmd(float) { return mode==CHASSIS_GYRO_SPIN?gyro_spin_speed:0; }
        int Get_Mode() { return mode; }
    } chassis_fsm={spin?CHASSIS_GYRO_SPIN:CHASSIS_NOT_FOLLOW};
    const bool keyboard_mode=keyboard;
    float yaw_offset_rad=0,wz_cmd=0;
'''+yaw_selection+r'''
    float out[3];run(0,0,wz_cmd,0,keyboard,out);return out[2];
}
int main() {
    float out[3];
    for(bool keyboard:{false,true}) {
        for(float x:{-1.f,-.5f,0.f,.5f,1.f}) {
            run(x,0,0,0,keyboard,out);
            near(out[0],x*2.56f);near(out[1],0);near(out[2],0);
        }
        for(float y:{-1.f,-.5f,0.f,.5f,1.f}) {
            run(0,y,0,0,keyboard,out);
            near(out[0],0);near(out[1],y*1.7f);near(out[2],0);
        }
        run(1,1,3,0,keyboard,out);
        near(out[0],2.56f);
        near(out[1],1.7f);
        near(out[2],1.5f);
        run(2,0,0,0,keyboard,out);near(out[0],2.56f);
        run(-2,0,0,0,keyboard,out);near(out[0],-2.56f);
        run(0,2,0,0,keyboard,out);near(out[1],1.7f);
        run(0,-2,0,0,keyboard,out);near(out[1],-1.7f);
        for(float w:{-100.f,-1.5f,-.75f,0.f,.75f,1.5f,100.f}) {
            run(0,0,w,0,keyboard,out);near(out[2],std::clamp(w,-1.5f,1.5f));
        }
        // Rotated input can combine two axes; body X must still obey the cap.
        run(1,1,0,.7853981634f,keyboard,out);near(out[0],2.56f);near(out[1],0);
        run(-1,-1,0,.7853981634f,keyboard,out);near(out[0],-2.56f);near(out[1],0);
        run(1,0,0,.7853981634f,keyboard,out);
        near(out[0],2.56f/std::sqrt(2.f));
        near(out[1],-1.7f/std::sqrt(2.f));
        run(-1,1,0,.7853981634f,keyboard,out);near(out[0],0);near(out[1],1.7f);
        run(1,-1,0,.7853981634f,keyboard,out);near(out[0],0);near(out[1],-1.7f);
    }
    for(auto source:{ControlInputSource::LOCAL,ControlInputSource::GIMBAL}) {
        for(float stick:{-1.f,-.5f,0.f,.5f,1.f})
            near(selected_yaw(stick,0,false,false,false,source),-stick*1.5f);
        near(selected_yaw(0,0,false,false,true,source),1.5f);
        near(selected_yaw(0,0,true,true,false,source),1.5f);
        near(selected_yaw(0,32767,true,false,false,source),-1.5f);
        near(selected_yaw(0,-32768,true,false,false,source),1.5f);
        near(selected_yaw(0,1,true,false,false,source),-3.14159265358979323846f/180.f);
    }
}
'''
with tempfile.TemporaryDirectory(prefix="chassis-vx-limit-") as directory:
    cpp=Path(directory)/"test.cpp"
    exe=Path(directory)/"test.exe"
    cpp.write_text(host,encoding="utf-8")
    subprocess.run(["g++","-std=c++17","-I",ROOT.as_posix(),cpp.as_posix(),
                    (ROOT/"user/core/Alg/Filter/Filter.cpp").as_posix(),
                    (ROOT/"user/core/Alg/UtilityFunction/SlopPlanning.cpp").as_posix(),
                    "-o",exe.as_posix()],check=True)
    subprocess.run([str(exe)],check=True)
print("PASS: X/Y/yaw limits, linear local/gimbal sticks, keyboard/mouse/spin and rotated combined inputs")
