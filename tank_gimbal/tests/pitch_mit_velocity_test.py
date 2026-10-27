"""Compile the task's actual PID declarations, MIT branches and FSM protection."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class PitchMitVelocityTest(unittest.TestCase):
    def test_actual_control_and_protection(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        self.assertIn("PITCH_MAX_SPEED_RAD_S = 95.0f * DEG_TO_RAD", task)
        self.assertIn("PITCH_REMOTE_ANGLE_MIT_KD = 1.3f", task)
        self.assertIn("PITCH_REMOTE_SPEED_MIT_KD = 1.3f", task)
        self.assertIn("PITCH_KEYMOUSE_MIT_KD = 1.3f", task)
        for name in ("pitch_angle_to_speed_encoder_pid", "pitch_keymouse_angle_to_speed_encoder_pid",
                     "pitch_remote_speed_encoder_pid"):
            self.assertNotIn(name, task)
        constants = "\n".join(re.findall(r"^constexpr float .*?;", task, re.M))
        pids = "\n".join(re.findall(r"^ALG::PID::PID .*?;", task, re.M))
        begin = task.index(" float pitch_mit_kd = 0.0f;")
        block = task[begin:task.index("        // UART10", begin)]
        harness = r'''
#include <cassert>
#include <cmath>
#define DT7_HPP
namespace BSP::REMOTE_CONTROL {
class RemoteController { public: enum { UP=1, DOWN=2, MIDDLE=3 }; };
}
#include "feeder_fsm/gimbal_fsm.cpp"
#include "user/core/Alg/PID/pid.hpp"
namespace BSP::Motor::DM {
float ClampMitCommandValue(float v,float lo,float hi) { return v<lo?lo:v>hi?hi:v; }
}
CONSTANTS
PIDS
enum class InputSource { Remote, KeyMouse };
struct { InputSource source=InputSource::Remote;
    InputSource GetSource() { return source; }
} input_dispatcher;
struct {
    float velocity=0,kd=0,ff=0;
    void ctrl_Mit(int slot,float position,float speed,float kp,float damping,float torque) {
        assert(slot==1 && position==0 && kp==0);
        velocity=speed; kd=damping; ff=torque;
    }
} gimbal_motor;
Class_Gimbal_FSM pitch_gimbal_fsm;
Struct_Gimbal_FSM_Config pitch_gimbal_fsm_config;
float pitch_target_speed=0,pitch_current_speed=0,pitch_error=0,pitch_control_output=0;
float pitch_current_angle=0.8f;
void step(bool pitch_motor_online=true,float pitch_gravity_output=0) {
BLOCK
}
void init(unsigned status,bool mouse=false,float angle=0.8f) {
    pitch_gimbal_fsm_config={};
    auto &config=pitch_gimbal_fsm_config;
    config.speed_scale=PITCH_MAX_SPEED_RAD_S;
    config.min_angle=34.2f*DEG_TO_RAD; config.max_angle=67.9f*DEG_TO_RAD;
    config.limit_angle=1; config.limit_upper_torque=0;
    pitch_gimbal_fsm.Init(config,status);
    pitch_current_angle=angle;
    pitch_gimbal_fsm.Update_Angle_Limit(angle,angle);
    pitch_remote_angle_encoder_pid.reset(); pitch_keymouse_angle_encoder_pid.reset();
    input_dispatcher.source=mouse?InputSource::KeyMouse:InputSource::Remote;
    pitch_current_speed=0; pitch_error=0;
}
bool near(float a,float b) { return std::fabs(a-b)<0.00001f; }
int main() {
    // Real gains and output caps, in rad/s; outer loops remain independent.
    init(GIMBAL_STATUS_ANGLE);
    pitch_error=0.01f; step();
    assert(near(gimbal_motor.velocity,0.32015f));
    assert(gimbal_motor.kd==PITCH_REMOTE_ANGLE_MIT_KD && gimbal_motor.ff==0);
    assert(pitch_keymouse_angle_encoder_pid.getOutput()==0);
    init(GIMBAL_STATUS_ANGLE,true);
    pitch_error=0.01f; step();
    assert(near(gimbal_motor.velocity,0.265f));
    assert(gimbal_motor.kd==PITCH_KEYMOUSE_MIT_KD);
    assert(pitch_remote_angle_encoder_pid.getOutput()==0);
    for (bool mouse : {false,true}) {
        for (float sign : {-1.0f,1.0f}) {
            init(GIMBAL_STATUS_ANGLE,mouse);
            pitch_error=sign; step();
            assert(near(gimbal_motor.velocity,sign*PITCH_MAX_SPEED_RAD_S));
            assert(gimbal_motor.kd==(mouse?PITCH_KEYMOUSE_MIT_KD:PITCH_REMOTE_ANGLE_MIT_KD));
        }
    }
    // Remote velocity mode bypasses the angle PID and uses MIT velocity feedback.
    init(GIMBAL_STATUS_SPEED);
    Struct_Gimbal_Input input={};
    input.s1=Remote::MIDDLE; input.s2=Remote::DOWN; input.joystick_speed=-0.5f;
    pitch_gimbal_fsm.Update(input,0.8f); step();
    assert(near(gimbal_motor.velocity,-0.5f*PITCH_MAX_SPEED_RAD_S) && gimbal_motor.kd==PITCH_REMOTE_SPEED_MIT_KD);
    assert(pitch_remote_angle_encoder_pid.getOutput()==0);
    // STOP/offline must remove Kd as well as feedforward, even while moving.
    init(GIMBAL_STATUS_STOP);
    pitch_current_speed=1; step(true,2);
    assert(gimbal_motor.velocity==0 && gimbal_motor.kd==0 && gimbal_motor.ff==0);
    init(GIMBAL_STATUS_ANGLE);
    pitch_error=1; pitch_current_speed=-1; step(false,2);
    assert(gimbal_motor.velocity==0 && gimbal_motor.kd==0 && gimbal_motor.ff==0);
    // Lower endpoint: positive target speed can still request negative braking torque.
    init(GIMBAL_STATUS_ANGLE,false,34.2f*DEG_TO_RAD);
    pitch_error=0.01f; pitch_current_speed=1; step();
    assert(gimbal_motor.kd==0 && gimbal_motor.ff==0 && pitch_control_output==0);
    init(GIMBAL_STATUS_ANGLE,false,34.2f*DEG_TO_RAD);
    pitch_error=0.01f; pitch_current_speed=-1; step();
    assert(gimbal_motor.kd==0 && gimbal_motor.ff>0 && pitch_control_output==gimbal_motor.ff);
    // Even zero estimated torque must not leave a live MIT speed loop at the lower bound.
    init(GIMBAL_STATUS_ANGLE,false,34.2f*DEG_TO_RAD); step();
    assert(gimbal_motor.kd==0 && gimbal_motor.ff==0);
    // Existing upper policy allows positive support/braking torque.
    init(GIMBAL_STATUS_ANGLE,false,67.9f*DEG_TO_RAD);
    pitch_current_speed=-1; step();
    assert(gimbal_motor.kd==PITCH_REMOTE_ANGLE_MIT_KD && pitch_control_output>0);
    // Protect combined Kd*(v_des-v_fb)+feedforward, rather than t_ff alone.
    init(GIMBAL_STATUS_ANGLE);
    pitch_current_speed=-13.0f/PITCH_REMOTE_ANGLE_MIT_KD; step(true,2);
    assert(near(pitch_control_output,15) && gimbal_motor.kd==PITCH_REMOTE_ANGLE_MIT_KD && gimbal_motor.ff==2);
    init(GIMBAL_STATUS_ANGLE);
    pitch_current_speed=13.0f/PITCH_REMOTE_ANGLE_MIT_KD; step(true,-2);
    assert(near(pitch_control_output,-15) && gimbal_motor.kd==PITCH_REMOTE_ANGLE_MIT_KD && gimbal_motor.ff==-2);
    init(GIMBAL_STATUS_ANGLE);
    pitch_current_speed=-10; step(true,20);
    assert(pitch_control_output==27 && gimbal_motor.kd==0 && gimbal_motor.ff==27);
    init(GIMBAL_STATUS_ANGLE);
    pitch_current_speed=10; step(true,-20);
    assert(pitch_control_output==-27 && gimbal_motor.kd==0 && gimbal_motor.ff==-27);
    init(GIMBAL_STATUS_ANGLE,false,34.2f*DEG_TO_RAD);
    step(true,-2);
    assert(gimbal_motor.kd==0 && gimbal_motor.ff==0);
    init(GIMBAL_STATUS_ANGLE);
    step(true,2);
    assert(gimbal_motor.kd==PITCH_REMOTE_ANGLE_MIT_KD && gimbal_motor.ff==2);
}
'''.replace("CONSTANTS", constants).replace("PIDS", pids).replace("BLOCK", block)
        harness = '#include <initializer_list>\n' + harness
        with tempfile.TemporaryDirectory(prefix="pitch_mit_velocity_") as directory:
            cpp = Path(directory) / "test.cpp"
            exe = Path(directory) / "test.exe"
            # Tune each Kd separately to verify that the three paths are independent.
            for source in (harness, harness.replace("PITCH_KEYMOUSE_MIT_KD = 1.3f",
                                                     "PITCH_KEYMOUSE_MIT_KD = 0.9f"),
                           harness.replace("PITCH_REMOTE_ANGLE_MIT_KD = 1.3f",
                                           "PITCH_REMOTE_ANGLE_MIT_KD = 0.9f"),
                           harness.replace("PITCH_REMOTE_SPEED_MIT_KD = 1.3f",
                                           "PITCH_REMOTE_SPEED_MIT_KD = 0.7f")):
                cpp.write_text(source, encoding="utf-8")
                subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra",
                                "-I", ROOT.as_posix(), "-I", (ROOT / "user/core/Alg/PID").as_posix(),
                                cpp.as_posix(), (ROOT / "user/core/Alg/PID/pid.cpp").as_posix(),
                                (ROOT / "user/core/Alg/FSM/alg_fsm.cpp").as_posix(),
                                "-o", exe.as_posix()], check=True)
                subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
