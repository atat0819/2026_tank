"""Exercise actual task fragments: four-wheel force drive in every stair phase."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
assert "rear_wheel_drive_only" not in source, "obsolete rear-only wheel policy remains"
assert "motor_pid[0].UpDate" not in re.sub(r"//[^\n]*", "", source), "legacy wheel PID still owns output"

def body(anchor):
    start = source.index("{", source.index(anchor))
    end, depth = start + 1, 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start + 1:end - 1]

start = source.index("taskENTER_CRITICAL();", source.index("一次 B 同时发布两个明确目标"))
publish = source[start:source.index("taskEXIT_CRITICAL();", start)]
start = source.index("current_speed_rads[0] =")
fault_anchor = "if (input.source == ControlInputSource::NONE || source_changed || !wheel_feedback_valid)"
feedback = source[start:source.index(fault_anchor, start)]
fault = body(fault_anchor)
stop = body("if (chassis_fsm.Get_Mode() == CHASSIS_STOP)")
mode_reset = body("if (chassis_fsm.Get_Mode() != last_mode)")
start = source.index("// 四轮车体力控")
drive = source[start:source.index("// 保存功率控制前的电流", start)]
save_pre = re.search(r"memcpy\(motor_output_pre, motor_output, sizeof\(motor_output_pre\)\);", source).group(0)
start = source.index("float I[4], V[4], I_other[4];")
power = source[start:source.index("// ========== 功率控制结束", start)].rstrip()
assert power.endswith("}")
power = power[:-1]
start = source.index("for (int i = 0; i < 4; i++) {", source.index("chassis_power_pred = post_power"))
send = source[start:source.index("ChassisMotorSendCANChecked();", start)]
coefficients = re.search(r"float poly_coeffs\[6\] = \{.*?\};", source, re.S).group(0)
limits = "\n".join(re.search(r"constexpr float "+name+r"\s*=[^;]+;", source).group(0)
                     for name in ("CHASSIS_MAX_VX_M_S", "MAX_VY_M_S", "MAX_WZ_RAD_S"))
filters = "\n".join(re.search(r"SecondOrderLPFFilter chassis_"+axis+r"_filter\([^;]+;", source).group(0)
                    for axis in ("vx", "vy", "wz"))
planners = "\n".join(re.search(r"Alg::Utility::SecondOrderSetpointPlanner ramp_v"+axis+r"\([^;]+;", source).group(0)
                     for axis in ("x", "y"))
host = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include "fsm/stair_mode_policy.hpp"
#include "fsm/control_input_selector.hpp"
#include "fsm/chassis_fsm.hpp"
#include "user/core/Alg/ChassisCalculation/MecanumForceControl.hpp"
#include "user/core/Alg/PowerControl/PowerControl.hpp"
#include "user/core/Alg/Filter/Filter.hpp"
#include "user/core/Alg/UtilityFunction/SecondOrderSetpointPlanner.hpp"
#define taskENTER_CRITICAL() ((void)0)
using Alg::CalculationBase::MecanumForceControl;
uint32_t HAL_GetTick() { return 1000; }
StairCommand stair_command;
uint32_t stair_command_generation=0;
struct Keys { bool imu_mode_request=false,normal_mode_request=false,stair_toggle=false,valid=true; };
Alg::CalculationBase::MecanumForceControl chassis_force;
float current_speed_rads[4],motor_output[4],motor_output_pre[4],motor_target_speed[4];
float force_vofa_channels[15]={};
float chassis_velocity_feedback[3];
float vx_body=.5f,vy_body=0,wz_cmd=0;
''' + filters + "\n" + planners + r'''
Chassis_FSM chassis_fsm(8,0,0,100,25,2.5f);
Enum_Chassis_Mode last_mode=CHASSIS_STOP;
struct Motor {
    float velocity[4]={},sent[4]={};
    unsigned online_mask=15,checked_mask=0;
    float getVelocityRads(int id) { return velocity[id-1]; }
    bool isConnected(int id,int) { checked_mask|=1U<<(id-1);return online_mask&(1U<<(id-1)); }
    void setCAN(int16_t value,int id) { sent[id-1]=value; }
} chassis_motor;
void ChassisMotorSendCANChecked() {}
ALG::PowerControl::PowerControl<4> chassis_power_ctrl;
'''
host += limits + "\n" + coefficients
host += "\nvoid reset_mode() {\n" + mode_reset + "\n}\n"
host += r'''
void step(Keys keyboard_cmd={},bool stopped=false,bool limited=true,
          ControlInputSource source=ControlInputSource::LOCAL,bool source_changed=false,
          float requested_vx=.5f) {
    const auto stair_sample_generation=stair_command_generation;
    struct { bool zero_all_torque; } stair_policy={stopped};
    struct { ControlInputSource source; } input={source};
'''
host += publish + "\n" + feedback + "\n" + fault_anchor + " {\n" + fault + "\nreturn;\n}\n"
host += "if(stopped) {\n" + stop + "\nreturn;\n}\n"
host += "vx_body=requested_vx;vy_body=0;wz_cmd=0;\n" + drive + "\n" + save_pre
host += "\nif(limited) {\nconst float PowerMax=50;\n" + power + "\n}\n"
host += send + "\n}\n"
host += r'''
float predicted_power(const float output[4]) {
    float power=0;
    for(unsigned i=0;i<4;++i) {
        const float a=output[i]*(20.f/16384),w=chassis_motor.velocity[i];
        power+=poly_coeffs[0]+poly_coeffs[1]*a+poly_coeffs[2]*fabsf(w)+
            poly_coeffs[3]*a*w+poly_coeffs[4]*a*a+poly_coeffs[5]*w*w;
    }
    return power;
}
void check_drive(bool limited=true) {
    const float signs[4]={-1,1,1,-1};
    for(unsigned i=0;i<4;++i) {
        assert(std::isfinite(motor_output[i]));
        assert(motor_output[i]*signs[i]>0); // Front wheels remain driven in all stair phases.
        assert(std::fabs(motor_output[i])<=16384.01f);
        assert(chassis_motor.sent[i]*signs[i]>0);
    }
    if(limited) assert(predicted_power(motor_output)<=50.01f);
}
void check_reset_state() {
    for(float value:chassis_force.integral) assert(value==0);
    for(float value:chassis_force.feedforward_current) assert(value==0);
    for(float value:chassis_force.feedback_current) assert(value==0);
    assert(ramp_vx.GetOut()==0&&ramp_vy.GetOut()==0);
    assert(ramp_vx.GetAcceleration()==0&&ramp_vy.GetAcceleration()==0);
    assert(chassis_vx_filter.getOutput()==0&&chassis_vy_filter.getOutput()==0);
    assert(chassis_wz_filter.getOutput()==0);
}
void check_zero() {
    for(unsigned i=0;i<4;++i) {
        assert(motor_output[i]==0&&motor_output_pre[i]==0);
        assert(motor_target_speed[i]==0&&chassis_motor.sent[i]==0);
    }
    check_reset_state();
}
int main() {
    for(unsigned wheel=0;wheel<4;++wheel) {
        chassis_force.kp[wheel]=chassis_force.ki[wheel]=chassis_force.kd[wheel]=0;
        chassis_force.integral_limit[wheel]=0;
    }
    step();
    check_drive(); // Planning acceleration still drives through feedforward with zero PID gains.
    assert(chassis_force.force[0]>0&&chassis_force.force[2]==0);
    for(unsigned wheel=0;wheel<4;++wheel) {
        assert(chassis_force.feedback_current[wheel]==0);
        assert(chassis_force.feedforward_current[wheel]!=0);
    }
    step({},true);check_zero();
    step({},false,true,ControlInputSource::LOCAL,false,0);
    for(float value:motor_output) assert(value==0); // Zero target, acceleration and feedback yield no drive.
    // Explicit test gains; no production or historical PID values are inherited.
    for(unsigned wheel=0;wheel<4;++wheel) {
        chassis_force.kp[wheel]=4;
        chassis_force.ki[wheel]=.5f;
        chassis_force.integral_limit[wheel]=2;
    }
    step();check_drive();
    for(unsigned n=0;n<100;++n) {step();check_drive();}
    // Actual task power limiting must constrain the already-summed FF + wheel PID.
    assert(predicted_power(motor_output_pre)>50.01f&&chassis_force.output_scale<1);
    for(unsigned wheel=0;wheel<4;++wheel) {
        assert(std::fabs(chassis_force.feedforward_current[wheel])>.1f);
        assert(std::fabs(chassis_force.feedback_current[wheel])>.1f);
        const float requested=chassis_force.feedforward_current[wheel]+chassis_force.feedback_current[wheel];
        assert(std::fabs(motor_output[wheel]-requested*MecanumForceControl::RAW_PER_AMP*
            chassis_force.output_scale)<.01f);
    }
    step({true,false,false,true});check_drive(); // V: leveling
    step({false,false,true,true});check_drive(); // B: front raised
    assert(stair_command.front_target_requested);
    for(unsigned i=0;i<20;++i) {step();check_drive();}
    step({false,false,true,true});check_drive(); // B: retract
    assert(stair_command.rear_retract_requested);
    step({false,false,true,true});check_drive(); // B: reverse
    step({false,true,false,true});check_drive(); // C: normal
    step({true,false,false,true},false,false);check_drive(false);
    step({false,false,true,true},false,false);check_drive(false); // Power bypass also stays four-wheel.
    chassis_wz_filter.Reset(2);step({},true);check_zero();
    step();check_drive();chassis_wz_filter.Reset(2);step({},false,true,ControlInputSource::NONE);check_zero();
    step();check_drive();chassis_wz_filter.Reset(2);step({},false,true,ControlInputSource::LOCAL,true);check_zero();
    for(unsigned missing=0;missing<4;++missing) {
        chassis_motor.online_mask=15;step();check_drive();
        chassis_wz_filter.Reset(2);
        chassis_motor.online_mask=15^(1U<<missing);chassis_motor.checked_mask=0;
        step();check_zero();assert(chassis_motor.checked_mask==15);
    }
    chassis_motor.online_mask=15;
    chassis_wz_filter.Reset(2);chassis_motor.velocity[2]=NAN;step();check_zero();
    chassis_motor.velocity[2]=0;step();check_drive();
    last_mode=CHASSIS_GYRO_SPIN;reset_mode();
    check_reset_state();
}
'''
with tempfile.TemporaryDirectory(prefix="stair-force-") as directory:
    work=Path(directory)
    (work/"main.h").write_text("#pragma once\n#include <cstdint>\nuint32_t HAL_GetTick();\n",encoding="utf-8")
    cpp=work/"test.cpp"
    cpp.write_text(host,encoding="utf-8")
    executable=work/"test.exe"
    subprocess.run(["g++","-std=c++17","-I",ROOT.as_posix(),"-I",(ROOT/"user").as_posix(),
                    "-I",(ROOT/"user/core").as_posix(),"-I",work.as_posix(),
                    cpp.as_posix(),(ROOT/"fsm/chassis_fsm.cpp").as_posix(),
                    (ROOT/"user/core/Alg/FSM/alg_fsm.cpp").as_posix(),
                    (ROOT/"user/core/Alg/PID/pid.cpp").as_posix(),
                    (ROOT/"user/core/Alg/Filter/Filter.cpp").as_posix(),
                    "-o",executable.as_posix()],check=True)
    subprocess.run([str(executable)],check=True)
print("PASS: actual four-wheel FF/PID task, V/B/C, combined power limit, planner/controller mode/STOP/input/wheel-feedback resets")
