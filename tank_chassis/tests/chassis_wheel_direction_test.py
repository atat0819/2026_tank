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
start=source.index("// 四轮车体力控")
drive=source[start:source.index("// 保存功率控制前的电流",start)]
start=source.index("// 力控 VOFA：")
publish=source[start:source.index("// 原功率辨识 VOFA 暂停",start)]
host=limits+r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include "fsm/chassis_fsm.hpp"
#include "user/core/Alg/ChassisCalculation/MecanumForceControl.hpp"
#include "user/core/Alg/Filter/Filter.hpp"
#include "user/core/Alg/UtilityFunction/SlopePlanning.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
using Alg::CalculationBase::MecanumForceControl;
MecanumForceControl chassis_force;
uint32_t HAL_GetTick() { return 100; }
''' + filters + r'''
Alg::Utility::SlopePlanning ramp_vx(100,100),ramp_vy(100,100);
float current_speed_rads[4]={},chassis_velocity_feedback[3],motor_output[4],motor_target_speed[4];
float keyboard_turn_limit_blend=0;
float force_vofa_channels[24]={};
void vofa_sendN(const float *data,uint8_t count) {
    assert(count==24);
    std::memcpy(force_vofa_channels,data,sizeof(force_vofa_channels));
}
float mouse_yaw(int16_t count) {
    Chassis_FSM fsm;
    fsm.Init();
    fsm.StateUpdate(3,3,true,true);
    return fsm.Get_wz_cmd(0,0,count,MAX_WZ_RAD_S);
}
void step(float vx_body,float vy_body,float wz_cmd) {
    struct { bool valid; } keyboard_cmd={false}; // Exercise remote targets without keyboard acceleration.
    const bool stair_retract_power_split=false; // This fixture exercises ordinary synchronized driving.
    const bool wheel_feedback_valid=true;
    const uint32_t now_tick=HAL_GetTick();
    float force_target[3]={},force_feedback[3]={};
    bool force_control_active=false;
''' + drive + publish + r'''
}
int main() {
    // Explicit test gains, independent of firmware tuning.
    for(unsigned axis=0;axis<3;++axis) {
        chassis_force.kp[axis]=axis==2?10:100;
        chassis_force.ki[axis]=0;
        chassis_force.integral_limit[axis]=0;
    }
    assert(mouse_yaw(0)==0);
    assert(mouse_yaw(6)==0 && mouse_yaw(-6)==0);
    assert(std::fabs(mouse_yaw(7)+7*M_PI/180)<1e-6);
    assert(std::fabs(mouse_yaw(-7)-7*M_PI/180)<1e-6);
    step(0,0,mouse_yaw(32767));assert(force_vofa_channels[6]==-MAX_WZ_RAD_S);
    step(0,0,mouse_yaw(-32768));assert(force_vofa_channels[6]==MAX_WZ_RAD_S);
    const float commands[][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},
                              {0,0,1},{0,0,-1},{0,0,4},{0,0,-4},{.4f,-.7f,.2f},{0,0,0}};
    const float sx[4]={-1,1,1,-1},sy[4]={-1,-1,1,1};
    for(const auto &command:commands) {
        chassis_force.Reset();chassis_vx_filter.Reset();chassis_vy_filter.Reset();chassis_wz_filter.Reset();
        step(command[0],command[1],command[2]);
        for(unsigned i=0;i<4;++i) {
            const float expected=(sx[i]*command[0]+
                sy[i]*std::clamp(command[1],-MAX_VY_M_S,MAX_VY_M_S)+
                .458f*std::clamp(command[2],-MAX_WZ_RAD_S,MAX_WZ_RAD_S))/.076f;
            assert(std::fabs(motor_target_speed[i]-expected)<.0001f);
            assert(std::fabs(force_vofa_channels[16+2*i]-expected)<.0001f);
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
        assert(std::fabs(force_vofa_channels[7]-yaw)<.0001f);
        for(unsigned i=0;i<4;++i) {
            assert(force_vofa_channels[16+2*i]==0);
            assert(force_vofa_channels[17+2*i]==current_speed_rads[i]);
        }
        assert(force_vofa_channels[8]*yaw>0);
        assert(std::fabs(force_vofa_channels[8])<std::fabs(yaw)*.5f);
        for(unsigned n=0;n<1000;++n) step(0,0,0);
        assert(std::fabs(force_vofa_channels[8]-yaw)<.0001f);
        assert(force_vofa_channels[8]==chassis_wz_filter.getOutput());
        assert(std::fabs(chassis_force.force[2]+chassis_force.kp[2]*force_vofa_channels[8])<.0001f);
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
            output_energy+=force_vofa_channels[8]*force_vofa_channels[8];
        }
    }
    assert(output_energy<input_energy*.01f);
}
'''
with tempfile.TemporaryDirectory(prefix="wheel-force-direction-") as directory:
    (Path(directory)/"main.h").write_text('#pragma once\n#include <cstdint>\nuint32_t HAL_GetTick();\n',encoding="utf-8")
    cpp=Path(directory)/"test.cpp"
    executable=Path(directory)/"test.exe"
    cpp.write_text(host,encoding="utf-8")
    subprocess.run(["g++","-std=c++17","-I",Path(directory).as_posix(),
                    "-I",ROOT.as_posix(),"-I",(ROOT/"user/core").as_posix(),cpp.as_posix(),
                    (ROOT/"fsm/chassis_fsm.cpp").as_posix(),
                    (ROOT/"user/core/Alg/FSM/alg_fsm.cpp").as_posix(),
                    (ROOT/"user/core/Alg/PID/pid.cpp").as_posix(),
                    (ROOT/"user/core/Alg/Filter/Filter.cpp").as_posix(),
                    (ROOT/"user/core/Alg/UtilityFunction/SlopPlanning.cpp").as_posix(),
                    "-o",executable.as_posix()],check=True)
    subprocess.run([str(executable)],check=True)
print("PASS: task wheel directions, mouse limits, filtered yaw PI/VOFA feedback and noise attenuation")
