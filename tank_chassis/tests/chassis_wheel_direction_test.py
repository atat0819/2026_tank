"""Exercise actual task targets/force output and preserve the existing mouse input."""
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
mouse=re.search(r"wz_cmd = -static_cast<float>\(input.mouse_x\)[^;]+;",source).group(0)
start=source.index("// 四轮车体力控")
drive=source[start:source.index("// 保存功率控制前的电流",start)]
host=limits+r'''
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
MecanumForceControl chassis_force;
''' + filters + r'''
Alg::Utility::SlopePlanning ramp_vx(100,100),ramp_vy(100,100);
float current_speed_rads[4]={},chassis_velocity_feedback[3],motor_output[4],motor_target_speed[4];
float force_vofa_channels[15]={};
float mouse_yaw(int16_t count) {
    struct { int16_t mouse_x; } input={count};
    float wz_cmd=0;
''' + mouse + r'''
    return wz_cmd;
}
void step(float vx_body,float vy_body,float wz_cmd) {
''' + drive + r'''
}
int main() {
    // Explicit test gains, independent of firmware tuning.
    for(unsigned axis=0;axis<3;++axis) {
        chassis_force.kp[axis]=axis==2?10:100;
        chassis_force.ki[axis]=0;
        chassis_force.integral_limit[axis]=0;
    }
    assert(mouse_yaw(0)==0);
    assert(std::fabs(mouse_yaw(1)+M_PI/180)<1e-6);
    assert(std::fabs(mouse_yaw(-1)-M_PI/180)<1e-6);
    step(0,0,mouse_yaw(32767));assert(force_vofa_channels[4]==-1.5f);
    step(0,0,mouse_yaw(-32768));assert(force_vofa_channels[4]==1.5f);
    const float commands[][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},
                              {0,0,1},{0,0,-1},{0,0,4},{0,0,-4},{.4f,-.7f,.2f},{0,0,0}};
    const float sx[4]={-1,1,1,-1},sy[4]={-1,-1,1,1};
    for(const auto &command:commands) {
        chassis_force.Reset();chassis_vx_filter.Reset();chassis_vy_filter.Reset();chassis_wz_filter.Reset();
        step(command[0],command[1],command[2]);
        for(unsigned i=0;i<4;++i) {
            const float expected=(sx[i]*command[0]+
                sy[i]*std::clamp(command[1],-1.7f,1.7f)+
                .458f*std::clamp(command[2],-1.5f,1.5f))/.076f;
            assert(std::fabs(motor_target_speed[i]-expected)<.0001f);
            assert(std::fabs(motor_output[i])<=16384.01f);
            if(expected==0) assert(std::fabs(motor_output[i])<.001f);
            else assert(motor_output[i]*expected>0);
        }
    }
    // Use the actual task's feedback and VOFA capture, not a stand-alone filter.
    for(float yaw:{2.f,-2.f}) {
        chassis_force.Reset();chassis_wz_filter.Reset();
        const float velocity[3]={0,0,yaw};
        chassis_force.InverseKinematics(velocity,current_speed_rads);
        step(0,0,0);
        assert(force_vofa_channels[5]*yaw>0);
        assert(std::fabs(force_vofa_channels[5])<std::fabs(yaw)*.5f);
        for(unsigned n=0;n<1000;++n) step(0,0,0);
        assert(std::fabs(force_vofa_channels[5]-yaw)<.0001f);
        assert(force_vofa_channels[5]==chassis_wz_filter.getOutput());
        assert(std::fabs(chassis_force.force[2]+chassis_force.kp[2]*force_vofa_channels[5])<.0001f);
    }
    // Test well above the actual cutoff; firmware filtering is tuned independently.
    chassis_wz_filter.Reset();
    const float noise_hz=std::min(4.f*chassis_wz_filter.getCutoffFrequency(),
                                  .4f/chassis_wz_filter.getSamplePeriod());
    float input_energy=0,output_energy=0;
    for(unsigned n=0;n<2000;++n) {
        const float yaw=std::sin(2.f*M_PI*noise_hz*chassis_wz_filter.getSamplePeriod()*n);
        const float velocity[3]={0,0,yaw};
        chassis_force.InverseKinematics(velocity,current_speed_rads);
        step(0,0,0);
        if(n>=1000) {
            input_energy+=yaw*yaw;
            output_energy+=force_vofa_channels[5]*force_vofa_channels[5];
        }
    }
    assert(output_energy<input_energy*.01f);
}
'''
with tempfile.TemporaryDirectory(prefix="wheel-force-direction-") as directory:
    cpp=Path(directory)/"test.cpp"
    executable=Path(directory)/"test.exe"
    cpp.write_text(host,encoding="utf-8")
    subprocess.run(["g++","-std=c++17","-I",ROOT.as_posix(),cpp.as_posix(),
                    (ROOT/"user/core/Alg/Filter/Filter.cpp").as_posix(),
                    (ROOT/"user/core/Alg/UtilityFunction/SlopPlanning.cpp").as_posix(),
                    "-o",executable.as_posix()],check=True)
    subprocess.run([str(executable)],check=True)
print("PASS: task wheel directions, mouse limits, filtered yaw PI/VOFA feedback and noise attenuation")
