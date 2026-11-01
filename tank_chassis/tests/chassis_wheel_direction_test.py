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
planners="\n".join(re.search(r"Alg::Utility::SecondOrderSetpointPlanner ramp_v"+axis+r"\([^;]+;",source).group(0)
                   for axis in ("x","y"))
mouse=re.search(r"wz_cmd = -static_cast<float>\(input.mouse_x\)[^;]+;",source).group(0)
start=source.index("// 四轮车体力控")
drive=source[start:source.index("// 保存功率控制前的电流",start)]
host=limits+r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include "user/core/Alg/ChassisCalculation/MecanumForceControl.hpp"
#include "user/core/Alg/Filter/Filter.hpp"
#include "user/core/Alg/UtilityFunction/SecondOrderSetpointPlanner.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
using Alg::CalculationBase::MecanumForceControl;
MecanumForceControl chassis_force;
''' + filters + "\n" + planners + r'''
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
void reset() {
    chassis_force.Reset();ramp_vx.Reset();ramp_vy.Reset();
    chassis_vx_filter.Reset();chassis_vy_filter.Reset();chassis_wz_filter.Reset();
    for(float &value:current_speed_rads) value=0;
}
void near(float actual,float expected,float tolerance=.0001f) {
    assert(std::isfinite(actual)&&std::fabs(actual-expected)<tolerance);
}
int main() {
    // Zero PID gains leave only the actual task's 13 kg translational feedforward.
    for(unsigned wheel=0;wheel<4;++wheel) {
        chassis_force.kp[wheel]=chassis_force.ki[wheel]=chassis_force.kd[wheel]=0;
        chassis_force.integral_limit[wheel]=0;
    }
    step(1,0,0);
    near(force_vofa_channels[0],.00002f,.0000001f);
    near(force_vofa_channels[9],.02f,.000001f);
    near(chassis_force.force[0],13.f*force_vofa_channels[9]);
    near(chassis_force.force[1],0);near(chassis_force.force[2],0);
    const float sx[4]={-1,1,1,-1},sy[4]={-1,-1,1,1};
    for(unsigned wheel=0;wheel<4;++wheel) {
        const float expected=sx[wheel]*.076f/4.f*.26f/MecanumForceControl::WHEEL_TORQUE_PER_AMP;
        near(chassis_force.feedforward_current[wheel],expected);
        near(chassis_force.feedback_current[wheel],0);
        near(motor_output[wheel],expected*MecanumForceControl::RAW_PER_AMP);
    }
    // An in-flight zero command plans braking, rather than clearing moving state.
    for(unsigned n=0;n<199;++n) step(1,0,0);
    bool braking=false;
    for(unsigned n=0;n<200&&!braking;++n) {
        step(0,0,0);
        braking=force_vofa_channels[9]<-.01f;
    }
    assert(braking&&force_vofa_channels[0]>0&&chassis_force.force[0]<0);
    for(unsigned wheel=0;wheel<4;++wheel) assert(motor_output[wheel]*sx[wheel]<0);
    reset();
    // Explicit test gains in A/(wheel rad/s), independent of firmware tuning.
    for(float &gain:chassis_force.kp) gain=1;
    assert(mouse_yaw(0)==0);
    assert(std::fabs(mouse_yaw(1)+M_PI/180)<1e-6);
    assert(std::fabs(mouse_yaw(-1)-M_PI/180)<1e-6);
    step(0,0,mouse_yaw(32767));assert(force_vofa_channels[4]==-1.5f);
    step(0,0,mouse_yaw(-32768));assert(force_vofa_channels[4]==1.5f);
    const float commands[][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},
                              {0,0,1},{0,0,-1},{0,0,4},{0,0,-4},{.4f,-.7f,.2f},{0,0,0}};
    for(const auto &command:commands) {
        reset();
        for(unsigned n=0;n<1500;++n) step(command[0],command[1],command[2]);
        for(unsigned i=0;i<4;++i) {
            const float expected=(sx[i]*std::clamp(command[0],-2.56f,2.56f)+
                sy[i]*std::clamp(command[1],-1.7f,1.7f)+
                .458f*std::clamp(command[2],-1.5f,1.5f))/.076f;
            assert(std::fabs(motor_target_speed[i]-expected)<.0001f);
            assert(std::fabs(motor_output[i])<=16384.01f);
            if(expected==0) assert(std::fabs(motor_output[i])<.001f);
            else assert(motor_output[i]*expected>0);
        }
    }
    // Body yaw filtering is telemetry only; raw wheel feedback brakes immediately.
    for(float yaw:{2.f,-2.f}) {
        reset();
        const float velocity[3]={0,0,yaw};
        chassis_force.InverseKinematics(velocity,current_speed_rads);
        step(0,0,0);
        assert(chassis_force.force[2]==0);
        for(unsigned wheel=0;wheel<4;++wheel) {
            near(chassis_force.feedforward_current[wheel],0);
            near(chassis_force.feedback_current[wheel],-current_speed_rads[wheel]);
            assert(motor_output[wheel]*current_speed_rads[wheel]<0);
        }
        assert(force_vofa_channels[5]*yaw>0);
        assert(std::fabs(force_vofa_channels[5])<std::fabs(yaw)*.5f);
        for(unsigned n=0;n<1000;++n) step(0,0,0);
        assert(std::fabs(force_vofa_channels[5]-yaw)<.0001f);
        assert(force_vofa_channels[5]==chassis_wz_filter.getOutput());
        assert(chassis_force.force[2]==0);
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
                    "-o",executable.as_posix()],check=True)
    subprocess.run([str(executable)],check=True)
print("PASS: task ramp/13kg feedforward/braking, wheel directions, mouse limits, wheel yaw braking and telemetry filtering")
