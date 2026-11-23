"""Exercise the task's real keyboard ramp, remote bypass and safety resets."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
constants = "\n".join(re.findall(
    r"constexpr float (?:CHASSIS_MAX_VX_M_S|MAX_VY_M_S|MAX_WZ_RAD_S|KEYBOARD_SPEED_RAMP_STEP)\s*=[^;]+;",
    source))
ramps = "\n".join(re.findall(r"Alg::Utility::SlopePlanning ramp_v[xy]\([^;]+;", source))
filters = "\n".join(re.findall(r"SecondOrderLPFFilter chassis_(?:vx|vy|wz)_filter\([^;]+;", source))
start = source.index("// 四轮车体力控")
target = source[start:source.index("// 收腿分组限功率期间", start)]


def body(anchor):
    start = source.index("{", source.index(anchor))
    end, depth = start + 1, 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start + 1:end - 1]


fault = body("if (input.source == ControlInputSource::NONE || source_changed || !wheel_feedback_valid)")
stop = body("if (chassis_fsm.Get_Mode() == CHASSIS_STOP)")
host = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include "user/core/Alg/ChassisCalculation/MecanumForceControl.hpp"
#include "user/core/Alg/Filter/Filter.hpp"
#include "user/core/Alg/UtilityFunction/SlopePlanning.hpp"
using Alg::CalculationBase::MecanumForceControl;
''' + constants + "\n" + ramps + "\n" + filters + r'''
MecanumForceControl chassis_force;
float current_speed_rads[4]={},chassis_velocity_feedback[3]={};
float motor_target_speed[4]={},motor_output[4]={},motor_output_pre[4]={};
float keyboard_turn_limit_blend=0;
struct Motor {
    float sent[4]={};
    void setCAN(int value,int id) { sent[id-1]=value; }
} chassis_motor;
struct FollowPID { void reset() {} };
struct State {
    FollowPID pid;
    FollowPID &Get_Follow_PID() { return pid; }
} chassis_fsm;
void ChassisMotorSendCANChecked() {}
void near(float actual,float expected) {
    if(!std::isfinite(actual)||std::fabs(actual-expected)>.0001f) {
        std::fprintf(stderr,"actual=%g expected=%g\n",actual,expected);
        assert(false);
    }
}
void step(float vx_body,float vy_body,float wz_cmd,bool keyboard=true) {
    struct { bool valid; } keyboard_cmd={keyboard};
    float force_target[3]={},force_feedback[3]={};
''' + target + r'''
    chassis_force.Update(force_target,force_feedback,motor_output);
    // Observe the actual values delivered to the force controller.
    chassis_velocity_feedback[0]=force_target[0];
    chassis_velocity_feedback[1]=force_target[1];
    chassis_velocity_feedback[2]=force_target[2];
}
void fault_reset() {
''' + fault + r'''
}
void stop_reset() {
''' + stop + r'''
}
const auto initial_vx=ramp_vx,initial_vy=ramp_vy;
void fresh() {
    ramp_vx=initial_vx;ramp_vy=initial_vy;
    keyboard_turn_limit_blend=0;
    chassis_vx_filter.Reset();chassis_vy_filter.Reset();chassis_wz_filter.Reset();
    std::fill_n(current_speed_rads,4,0.f);
    chassis_force.Reset();
}
void set_feedback(float x,float y) {
    const float velocity[3]={x,y,0};
    chassis_force.InverseKinematics(velocity,current_speed_rads);
    chassis_vx_filter.Reset(x);chassis_vy_filter.Reset(y);
}
int main() {
    // Both axes ramp in either direction, converge and never exceed their target.
    for(float sign:{-1.f,1.f}) {
        fresh();
        for(unsigned tick=1;tick<=500;++tick) {
            step(sign*CHASSIS_MAX_VX_M_S,sign*MAX_VY_M_S,0);
            near(chassis_velocity_feedback[0],sign*std::min(tick*.008f,CHASSIS_MAX_VX_M_S));
            near(chassis_velocity_feedback[1],sign*std::min(tick*.008f,MAX_VY_M_S));
            near(chassis_velocity_feedback[2],0);
        }
        step(sign*.1f,sign*.2f,0);
        near(chassis_velocity_feedback[0],sign*.1f);
        near(chassis_velocity_feedback[1],sign*.2f);
        // Releasing each axis clears its target without disturbing the other axis.
        step(0,sign*.2f,0);
        near(chassis_velocity_feedback[0],0);near(chassis_velocity_feedback[1],sign*.2f);
        step(0,0,0);
        near(chassis_velocity_feedback[0],0);near(chassis_velocity_feedback[1],0);
        step(sign*1.f,sign*1.f,0);
        near(chassis_velocity_feedback[0],sign*.008f);
        near(chassis_velocity_feedback[1],sign*.008f);
    }
    fresh();
    // Real speed may run ahead of the command; it must not skip target planning.
    set_feedback(CHASSIS_MAX_VX_M_S,-MAX_VY_M_S);
    step(CHASSIS_MAX_VX_M_S,-MAX_VY_M_S,-MAX_WZ_RAD_S);
    near(chassis_velocity_feedback[0],.008f);near(chassis_velocity_feedback[1],-.008f);
    near(chassis_velocity_feedback[2],-MAX_WZ_RAD_S);
    step(CHASSIS_MAX_VX_M_S,-MAX_VY_M_S,0);
    near(chassis_velocity_feedback[0],.016f);near(chassis_velocity_feedback[1],-.016f);
    fresh();
    step(.003f,-.005f,0);
    near(chassis_velocity_feedback[0],.003f);near(chassis_velocity_feedback[1],-.005f);
    fresh();
    for(unsigned tick=0;tick<500;++tick) step(CHASSIS_MAX_VX_M_S,0,0);
    set_feedback(1.5f,0);
    step(CHASSIS_MAX_VX_M_S,0,1);
    near(chassis_velocity_feedback[0],CHASSIS_MAX_VX_M_S);
    near(chassis_velocity_feedback[2],1);
    float previous=chassis_velocity_feedback[0];
    for(unsigned tick=0;tick<105;++tick) {
        step(CHASSIS_MAX_VX_M_S,0,0);
        assert(chassis_velocity_feedback[0]>=previous-.0001f);
        assert(chassis_velocity_feedback[0]-previous<.04f);
        previous=chassis_velocity_feedback[0];
    }
    near(previous,CHASSIS_MAX_VX_M_S);
    step(CHASSIS_MAX_VX_M_S,0,1);
    step(0,0,1);
    near(chassis_velocity_feedback[0],0);
    step(-CHASSIS_MAX_VX_M_S,0,1);
    near(chassis_velocity_feedback[0],-.008f);
    fresh();
    for(unsigned tick=0;tick<125;++tick) step(1,-1,-1);
    near(chassis_velocity_feedback[0],1.f);
    near(chassis_velocity_feedback[1],-1.f);
    // Reversal discards the old direction and starts the new direction with one step.
    for(float sign:{-1.f,1.f}) {
        fresh();
        for(unsigned tick=0;tick<100;++tick) step(sign,sign,0);
        step(-sign,-sign,0);
        near(chassis_velocity_feedback[0],-sign*.008f);
        near(chassis_velocity_feedback[1],-sign*.008f);
        step(-sign,-sign,0);
        near(chassis_velocity_feedback[0],-sign*.016f);
    }
    // Remote sticks always bypass the ramp, including reversals and limits.
    fresh();
    for(unsigned tick=0;tick<20;++tick) step(1,1,0);
    for(float x:{CHASSIS_MAX_VX_M_S,-CHASSIS_MAX_VX_M_S,.1f,0.f}) {
        step(x,-MAX_VY_M_S,MAX_WZ_RAD_S,false);
        near(chassis_velocity_feedback[0],x);near(chassis_velocity_feedback[1],-MAX_VY_M_S);
        near(chassis_velocity_feedback[2],MAX_WZ_RAD_S);
        near(ramp_vx.GetOut(),0);near(ramp_vy.GetOut(),0);
    }
    step(1,1,0);
    near(chassis_velocity_feedback[0],.008f);near(chassis_velocity_feedback[1],.008f);
    // Every actual safety branch must discard old ramp state.
    for(auto reset:{fault_reset,stop_reset}) {
        fresh();
        for(unsigned tick=0;tick<100;++tick) step(1,1,1);
        std::fill_n(motor_output,4,100.f);
        std::fill_n(motor_output_pre,4,100.f);
        std::fill_n(motor_target_speed,4,100.f);
        reset();
        near(ramp_vx.GetOut(),0);near(ramp_vy.GetOut(),0);
        near(keyboard_turn_limit_blend,0);
        for(unsigned i=0;i<4;++i) {
            near(motor_output[i],0);near(motor_output_pre[i],0);
            near(motor_target_speed[i],0);near(chassis_motor.sent[i],0);
        }
        step(1,1,0);
        near(chassis_velocity_feedback[0],.008f);near(chassis_velocity_feedback[1],.008f);
    }
}
'''
with tempfile.TemporaryDirectory(prefix="chassis-keyboard-ramp-") as directory:
    cpp = Path(directory) / "test.cpp"
    executable = Path(directory) / "test.exe"
    cpp.write_text(host, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-I", ROOT.as_posix(),
                    cpp.as_posix(), (ROOT / "user/core/Alg/Filter/Filter.cpp").as_posix(),
                    (ROOT / "user/core/Alg/UtilityFunction/SlopPlanning.cpp").as_posix(),
                    "-o", executable.as_posix()], check=True)
    subprocess.run([str(executable)], check=True)
print("PASS: keyboard acceleration/reversal, immediate release, remote bypass, yaw and safety resets")
