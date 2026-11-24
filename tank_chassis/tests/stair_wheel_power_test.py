"""Exercise actual task fragments: four-wheel force drive in every stair phase."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
assert (
    "chassis_force.UpdateWheelSynchronization(current_speed_rads, force_target, force_feedback);" in source
), "wheel synchronization must use the task's target and filtered feedback"
assert "rear_wheel_drive_only" not in source, "obsolete rear-only wheel policy remains"
assert "motor_pid[0].UpDate" not in re.sub(r"//[^\n]*", "", source), "wheel-speed PID still owns output"

def body(anchor):
    start = source.index("{", source.index(anchor))
    end, depth = start + 1, 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start + 1:end - 1]

start = source.index("taskENTER_CRITICAL();", source.index("一次 B 同时发布两个明确目标"))
publish = source[start:source.index("taskEXIT_CRITICAL();", start)]
start = source.index("const bool keyboard_mode =")
keyboard_update = source[start:source.index("const StairModePolicy stair_policy", start)]
start = source.index("current_speed_rads[0] =")
fault_anchor = "if (input.source == ControlInputSource::NONE || source_changed || !wheel_feedback_valid)"
feedback = source[start:source.index(fault_anchor, start)]
fault = body(fault_anchor)
stop = body("if (chassis_fsm.Get_Mode() == CHASSIS_STOP)")
mode_reset = body("if (chassis_fsm.Get_Mode() != last_mode)")
start = source.index("// 四轮车体力控")
drive = source[start:source.index("// ========== 底盘功率控制", start)]
start = source.index("float I[4], V[4], I_other[4];")
power = source[start:source.index("// ========== 功率控制结束", start)].rstrip()
assert power.endswith("}")
power = power[:-1]
start = source.index("for (int i = 0; i < 4; i++) {", source.index("chassis_power_pred = post_power"))
send = source[start:source.index("ChassisMotorSendCANChecked();", start)]
coefficients = re.search(r"float poly_coeffs\[6\] = \{.*?\};", source, re.S).group(0)
controllers = "\n".join(re.findall(r"ALG::PowerControl::PowerControl<\d+> chassis_\w*power_ctrl;",source))
shares = "\n".join(re.findall(r"constexpr float STAIR_RETRACT_FRONT_POWER_SHARE\s*=[^;]+;",source))
limits = "\n".join(re.search(r"constexpr float "+name+r"\s*=[^;]+;", source).group(0)
                     for name in ("CHASSIS_MAX_VX_M_S", "MAX_VY_M_S", "MAX_WZ_RAD_S"))
filters = "\n".join(re.search(r"SecondOrderLPFFilter chassis_"+axis+r"_filter\([^;]+;", source).group(0)
                    for axis in ("vx", "vy", "wz"))
host = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include "fsm/stair_mode_policy.hpp"
#include "fsm/control_input_selector.hpp"
#include "fsm/chassis_fsm.hpp"
#include "fsm/chassis_keyboard_fsm.hpp"
#include "user/core/Alg/ChassisCalculation/MecanumForceControl.hpp"
#include "user/core/Alg/PowerControl/PowerControl.hpp"
#include "user/core/Alg/PowerControl/ChassisPowerAllocator.hpp"
#include "user/core/Alg/Filter/Filter.hpp"
#include "user/core/Alg/UtilityFunction/SlopePlanning.hpp"
#define taskENTER_CRITICAL() ((void)0)
using Alg::CalculationBase::MecanumForceControl;
uint32_t HAL_GetTick() { return 1000; }
StairCommand stair_command;
ChassisKeyboardFSM keyboard_fsm;
uint32_t stair_command_generation=0;
struct Keys { bool imu_mode_request=false,normal_mode_request=false,stair_toggle=false,valid=true; };
Alg::CalculationBase::MecanumForceControl chassis_force;
float current_speed_rads[4],motor_output[4],motor_output_pre[4],motor_target_speed[4];
float chassis_velocity_feedback[3];
float vx_body=.5f,vy_body=0,wz_cmd=0;
ALG::PowerControl::ChassisPowerAllocator chassis_power_allocator;
''' + filters + r'''
Alg::Utility::SlopePlanning ramp_vx(100,100),ramp_vy(100,100);
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
'''
host += limits + "\n" + coefficients + "\n" + controllers + "\n" + shares
host += "\nvoid reset_mode() {\n" + mode_reset + "\n}\n"
host += r'''
void step(Keys keyboard_cmd={},bool stopped=false,bool limited=true,
          ControlInputSource source=ControlInputSource::LOCAL,bool source_changed=false,float yaw=0) {
    const bool CHASSIS_POWER_LIMIT_ENABLED=limited;
    const auto stair_sample_generation=stair_command_generation;
    struct { bool zero_all_torque; } stair_policy={stopped};
    struct { ControlInputSource source; } input={source};
    float force_target[3]={},force_feedback[3]={};
    bool force_control_active=false;
'''
host += publish + "\n" + feedback + "\n" + fault_anchor + " {\n" + fault + "\nreturn;\n}\n"
host += "if(stopped) {\n" + stop + "\nreturn;\n}\n"
host += "vx_body=.5f;vy_body=0;wz_cmd=yaw;\n" + drive
host += "\nif(limited) {\nconst float PowerMax=50;\n" + power + "\n}\n"
host += send + "\n}\n"
host += r'''
void step_switch(uint8_t s1,uint8_t s2,uint16_t keys,uint32_t now_tick,bool online=true) {
    struct { uint8_t s1,s2; uint16_t keyboard; bool keyboard_online; } input={s1,s2,keys,online};
    const bool source_changed=false;
''' + keyboard_update + r'''
    const auto policy=EvaluateStairModePolicy(s1,s2,true,online);
    step({keyboard_cmd.imu_mode_request,keyboard_cmd.normal_mode_request,
          keyboard_cmd.stair_toggle,keyboard_cmd.valid},policy.zero_all_torque);
}
'''
host += r'''
void limit_currents(const float requested[4],const float rotor[4],float PowerMax,
                    bool stair_retract_power_split) {
    chassis_force.Reset();
    for(unsigned i=0;i<4;++i) {
        motor_output_pre[i]=motor_output[i]=requested[i]*(16384.f/20);
        chassis_motor.velocity[i]=rotor[i];
    }
''' + power + "\n}\n"
host += r'''
float wheel_power(unsigned i) {
    const float a=motor_output[i]*(20.f/16384),w=chassis_motor.velocity[i];
    return poly_coeffs[0]+poly_coeffs[1]*a+poly_coeffs[2]*fabsf(w)+
        poly_coeffs[3]*a*w+poly_coeffs[4]*a*a+poly_coeffs[5]*w*w;
}
void check_budget(bool split,float budget=50) {
    const float front=wheel_power(0)+wheel_power(1),rear=wheel_power(2)+wheel_power(3);
    assert(front+rear<=budget+.01f);
    if(split) {
        assert(front<=budget*.75f+.01f);assert(rear<=budget*.25f+.01f);
    }
}
void check_drive(bool limited=true,bool keyboard_valid=true) {
    const float signs[4]={-1,1,1,-1};
    float power=0;
    for(unsigned i=0;i<4;++i) {
        assert(std::isfinite(motor_output[i]));
        assert(motor_output[i]*signs[i]>0); // Front wheels remain driven in all stair phases.
        assert(std::fabs(motor_output[i])<=16384.01f);
        assert(chassis_motor.sent[i]*signs[i]>0);
        const float a=motor_output[i]*(20.f/16384),w=chassis_motor.velocity[i];
        power+=poly_coeffs[0]+poly_coeffs[1]*a+poly_coeffs[2]*fabsf(w)+
            poly_coeffs[3]*a*w+poly_coeffs[4]*a*a+poly_coeffs[5]*w*w;
    }
    if(limited) check_budget(keyboard_valid&&stair_command.mode==StairMode::IMU&&stair_command.rear_retract_requested);
    else for(unsigned i=0;i<4;++i) assert(motor_output[i]==motor_output_pre[i]);
}
void check_zero() {
    for(unsigned i=0;i<4;++i) {
        assert(motor_output[i]==0&&motor_output_pre[i]==0);
        assert(motor_target_speed[i]==0&&chassis_motor.sent[i]==0);
    }
    for(float value:chassis_force.integral) assert(value==0);
    for(unsigned i=0;i<4;++i) {
        assert(chassis_force.sync_target_speed[i]==0);
        assert(chassis_force.sync_error[i]==0&&chassis_force.sync_current[i]==0);
    }
    assert(chassis_vx_filter.getOutput()==0&&chassis_vy_filter.getOutput()==0);
    assert(chassis_wz_filter.getOutput()==0);
}
int main() {
    for(unsigned axis=0;axis<3;++axis) {
        chassis_force.kp[axis]=0;
        chassis_force.ki[axis]=0;
        chassis_force.integral_limit[axis]=0;
    }
    step();
    for(float value:motor_output) assert(value==0); // Explicit zero gains cannot drive any wheel.
    // Explicit test gains; no production or historical PID values are inherited.
    for(unsigned axis=0;axis<3;++axis) {
        chassis_force.kp[axis]=axis==2?10:100;
        chassis_force.ki[axis]=axis==2?3:30;
        chassis_force.integral_limit[axis]=axis==2?5:20;
    }
    chassis_force.kp[0]=600; // Ensure the fixture reaches the power budget.
    step();check_drive();
    step({true,false,false,true});check_drive(); // V: leveling
    step({false,false,true,true});check_drive(); // B: front raised
    assert(stair_command.front_target_requested);
    for(unsigned i=0;i<20;++i) {step();check_drive();}
    step({false,false,true,true});check_drive(); // B: retract
    assert(stair_command.rear_retract_requested);
    assert(fabsf(wheel_power(0)+wheel_power(1)-37.5f)<.01f);
    assert(fabsf(wheel_power(2)+wheel_power(3)-12.5f)<.01f);
    for(unsigned i=0;i<20;++i) {step();check_drive();}
    step({false,false,false,false});check_drive(true,false); // Exit double-middle / keyboard loss.
    // Re-enter retained retract state, then a fresh B restores leveling/global scaling.
    step();check_drive();
    step({false,false,true,true});check_drive(); // B: reverse
    step({false,false,true,true});check_drive();
    step({true,false,false,true});check_drive(); // V resets retract.
    step({false,false,true,true});check_drive();
    step({false,false,true,true});check_drive();
    step({false,true,false,true});check_drive(); // C: normal
    step({true,false,false,true},false,false);check_drive(false);
    step({false,false,true,true},false,false);check_drive(false);
    step({false,false,true,true},false,false);check_drive(false);
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
    last_mode=CHASSIS_JOYSTICK_ROTATE;reset_mode();
    for(float value:chassis_force.integral) assert(value==0);
    // Exercise synchronization through the actual task, with the body PI disabled.
    for(unsigned i=0;i<3;++i) {chassis_force.kp[i]=0;chassis_force.ki[i]=0;}
    const float rpm[4]={-6000,6000,2000,-2000};
    for(unsigned i=0;i<4;++i) chassis_motor.velocity[i]=rpm[i]*6.28318530718f/60;
    step({},false,false);
    assert(motor_output[0]>0&&motor_output[1]<0);
    assert(motor_output[2]>0&&motor_output[3]<0);
    assert(chassis_force.sync_current[0]>0);
    step({},true);check_zero();
    step({},false,false);assert(chassis_force.sync_current[0]>0);
    step({},false,true,ControlInputSource::NONE);check_zero();
    // Unequal wheel speeds must not leave any synchronization current in retract.
    step({true,false,false,true});assert(chassis_force.sync_current[0]>0);
    step({false,false,true,true});assert(chassis_force.sync_current[0]>0);
    step({false,false,true,true});check_budget(true);
    auto check_sync_off=[]() {
        for(unsigned i=0;i<4;++i) {
            assert(chassis_force.sync_target_speed[i]==0);
            assert(chassis_force.sync_error[i]==0&&chassis_force.sync_current[i]==0);
            assert(motor_output[i]==0); // Body PI disabled: no hidden correction may drive a wheel.
        }
    };
    check_sync_off();
    for(unsigned i=0;i<20;++i) {step();check_budget(true);check_sync_off();}
    step({},false,false);check_sync_off(); // Sync stays disabled even with power limiting bypassed.
    step({false,false,true,true});assert(chassis_force.sync_current[0]>0);check_budget(false);
    step({false,false,true,true});check_sync_off();
    step({true,false,false,true});assert(chassis_force.sync_current[0]>0);check_budget(false);
    step({false,false,true,true});step({false,false,true,true});check_sync_off();
    step({false,true,false,true});assert(chassis_force.sync_current[0]>0);check_budget(false);
    step({true,false,false,true});step({false,false,true,true});step({false,false,true,true});check_sync_off();
    step_switch(UP,UP,0,10000);assert(chassis_force.sync_current[0]>0);check_budget(false);
    step({true,false,false,true});step({false,false,true,true});step({false,false,true,true});check_sync_off();
    step_switch(MIDDLE,MIDDLE,0,10001,false);assert(chassis_force.sync_current[0]>0);check_budget(false);
    for(unsigned i=0;i<4;++i) chassis_motor.velocity[i]=0;
    chassis_force.kp[0]=600;
    for(uint8_t s1:{UP,DOWN,MIDDLE}) for(uint8_t s2:{UP,DOWN,MIDDLE}) {
        if(s1==MIDDLE&&s2==MIDDLE) continue;
        keyboard_fsm.Init();uint32_t tick=0;
        step_switch(MIDDLE,MIDDLE,0,tick);
        step_switch(MIDDLE,MIDDLE,0,tick+=20);
        auto press=[&](uint16_t key) {
            step_switch(MIDDLE,MIDDLE,key,++tick);
            step_switch(MIDDLE,MIDDLE,key,tick+=20);
            step_switch(MIDDLE,MIDDLE,0,++tick);
            step_switch(MIDDLE,MIDDLE,0,tick+=20);
        };
        press(ChassisKeyboardFSM::KEY_V);
        press(ChassisKeyboardFSM::KEY_B);press(ChassisKeyboardFSM::KEY_B);
        check_drive();assert(stair_command.rear_retract_requested);
        for(unsigned i=0;i<5;++i) {
            step_switch(s1,s2,0,++tick);
            if(s1==DOWN&&s2==DOWN) check_zero();else check_drive(true,false);
        }
    }
    step({true,false,false,true});step({false,false,true,true});step({false,false,true,true});
    step_switch(MIDDLE,MIDDLE,0,10000,false);check_drive(true,false);
    // Forward motion at the 50 W limit must retain the requested yaw torque.
    stair_command={};chassis_force.Reset();
    for(unsigned axis=0;axis<3;++axis) chassis_force.ki[axis]=0;
    chassis_force.kp[0]=600;chassis_force.kp[2]=10;
    for(unsigned i=0;i<4;++i) chassis_motor.velocity[i]=0;
    step({},false,true,ControlInputSource::LOCAL,false,1);
    assert(chassis_power_allocator.beta==1&&chassis_power_allocator.alpha<1);
    float yaw_current=0;
    for(float raw:motor_output) yaw_current+=raw/MecanumForceControl::RAW_PER_AMP;
    const float actual_yaw=yaw_current*MecanumForceControl::WHEEL_TORQUE_PER_AMP*
        MecanumForceControl::LEVER_ARM/MecanumForceControl::WHEEL_RADIUS;
    assert(fabsf(actual_yaw-chassis_force.force[2])<.001f);
    check_budget(false);
    // Unequal wheel speeds require synchronization while moving and turning.
    const float unequal_rotor[4]={-40,40,10,-10};
    for(unsigned i=0;i<4;++i) chassis_motor.velocity[i]=unequal_rotor[i];
    step({},false,true,ControlInputSource::LOCAL,false,1);
    assert(chassis_power_allocator.beta==chassis_power_allocator.gamma);
    assert(chassis_power_allocator.gamma>0&&fabsf(chassis_force.sync_current[0])>.01f);
    const float sync_sign[4]={1,-1,1,-1};
    float applied_sync=0;
    for(unsigned i=0;i<4;++i)
        applied_sync+=sync_sign[i]*motor_output[i]/(4*MecanumForceControl::RAW_PER_AMP);
    assert(fabsf(applied_sync-chassis_power_allocator.gamma*chassis_force.sync_current[0])<.001f);
    check_budget(false);
    const float cases[][4]={{-20,20,20,-20},{0,0,20,-20},{-20,20,0,0},
        {-2,1,18,-17},{-18,16,1,-2},{0,0,0,0},{20,-15,-12,17}};
    for(float budget:{20.f,50.f,100.f}) for(float speed:{0.f,80.f,200.f})
    for(float direction:{-1.f,1.f}) for(const auto &request:cases) {
        float requested[4],rotor[4]={-speed,speed,speed,-speed};
        for(unsigned i=0;i<4;++i) requested[i]=request[i]*direction;
        limit_currents(requested,rotor,budget,true);check_budget(true,budget);
        float scale[2]={1,1};
        for(unsigned group=0;group<2;++group) {
            for(unsigned j=0;j<2;++j) {
                const unsigned i=2*group+j;const float actual=motor_output[i]*(20.f/16384);
                assert(std::isfinite(actual)&&fabsf(actual)<=fabsf(requested[i])+.0001f);
                assert(actual*requested[i]>=0);
                if(requested[i]!=0) scale[group]=actual/requested[i];else assert(actual==0);
            }
            for(unsigned j=0;j<2;++j) {
                const unsigned i=2*group+j;
                assert(fabsf(motor_output[i]*(20.f/16384)-requested[i]*scale[group])<.0001f);
            }
        }
        assert(fabsf(chassis_force.output_scale-fminf(scale[0],scale[1]))<.0001f);
    }
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
                    (ROOT/"fsm/chassis_keyboard_fsm.cpp").as_posix(),
                    (ROOT/"user/core/Alg/FSM/alg_fsm.cpp").as_posix(),
                    (ROOT/"user/core/Alg/PID/pid.cpp").as_posix(),
                    (ROOT/"user/core/Alg/Filter/Filter.cpp").as_posix(),
                    (ROOT/"user/core/Alg/UtilityFunction/SlopPlanning.cpp").as_posix(),
                    "-o",executable.as_posix()],check=True)
    subprocess.run([str(executable)],check=True)
print("PASS: force+sync, retract75/25 without borrowing, V/B/C and all switch exits, group ratios and safety resets")
