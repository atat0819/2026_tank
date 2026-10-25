"""Compile actual single-Pitch dropout/recovery branches and the recovery FSM."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class RecoveryTest(unittest.TestCase):
    def test_pitch_dropout_and_recovery(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        begin = task.index("  // 电机掉线按轴处理：")
        dropout = task[begin:task.index("   // 电机恢复按轴处理：", begin)]
        begin = task.index("   // 电机恢复按轴处理：")
        recovery = task[begin:task.index("   // 遥控恢复：", begin)]
        self.assertNotIn("continue;", dropout)
        self.assertNotIn("yaw_motor_was_offline", task)
        inputs = task.split("Struct_Gimbal_Input pitch_input", 1)[1].split(
            "pitch_gimbal_fsm.Update(pitch_input", 1)[0]
        for value in ("if (!pitch_motor_online)", "pitch_input = {};",
                      "pitch_input.s1 = Remote::DOWN;", "pitch_input.s2 = Remote::DOWN;"):
            self.assertIn(value, inputs)
        harness = r'''
#include <cassert>
bool pitch_motor_was_offline=false;
int pitch_resets=0;
float pitch_current_angle=2;
void ResetPitchControlState() { ++pitch_resets; }
struct Fsm { int anchors=0; float target=0;
    void ReAnchor(float angle) { ++anchors; target=angle; }
} pitch_gimbal_fsm;
void step(bool pitch_motor_online) {
DROPOUT
RECOVERY
}
int main() {
    step(true);
    assert(pitch_resets==0 && pitch_gimbal_fsm.anchors==0);
    step(false); step(false);
    assert(pitch_resets==2 && pitch_gimbal_fsm.anchors==0);
    step(true);
    assert(pitch_resets==3 && pitch_gimbal_fsm.anchors==1);
    assert(pitch_gimbal_fsm.target==pitch_current_angle);
    step(true);
    assert(pitch_resets==3 && pitch_gimbal_fsm.anchors==1);
    pitch_current_angle=4;
    step(false); step(true);
    assert(pitch_resets==5 && pitch_gimbal_fsm.anchors==2);
    assert(pitch_gimbal_fsm.target==4);
}
'''.replace("DROPOUT", dropout).replace("RECOVERY", recovery)
        self.compile_and_run(harness)

    def test_task_recovery(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        self.assertIn("MotorRecoveryFSM pitch_recovery_fsm;", task)
        remote_fault = task.split("gimbal_remote_offline = 1;", 1)[1].split("continue;", 1)[0]
        self.assertIn("pitch_recovery_fsm.Force_Enable_On_Next_Check();", remote_fault)
        self.assertIn("pitch_recovery_fsm.Init(", task)
        begin = task.index("  // 电机恢复使能：")
        branch = task[begin:task.index("  // 电机掉线按轴处理：", begin)]
        harness = r'''
#include <cassert>
#include "feeder_fsm/motor_recovery_fsm.hpp"
namespace Remote { enum { UP=1, DOWN=2, MIDDLE=3 }; }
enum class InputSource { Remote, KeyMouse };
struct { int s1=Remote::UP, s2=Remote::UP; } RemoteData;
struct { InputSource GetSource() { return InputSource::Remote; } } input_dispatcher;
namespace BSP::Motor::DM { enum { MIT=0 }; }
struct { int calls=0; void On(int slot, int) { assert(slot==1); ++calls; } } gimbal_motor;
MotorRecoveryFSM pitch_recovery_fsm;
unsigned now_tick=0;
unsigned HAL_GetTick() { return now_tick; }
void step(bool pitch_motor_online) {
BRANCH
}
int main() {
    pitch_recovery_fsm.Init(0);
    step(true); step(true);
    assert(gimbal_motor.calls==1);
    now_tick=10; step(false);
    assert(gimbal_motor.calls==2);
    now_tick=109; step(false);
    assert(gimbal_motor.calls==2);
    now_tick=110; step(false);
    assert(gimbal_motor.calls==3);
    now_tick=111; step(true);
    assert(gimbal_motor.calls==4);
    RemoteData.s1=Remote::DOWN; RemoteData.s2=Remote::DOWN;
    now_tick=500; step(false); step(true);
    assert(gimbal_motor.calls==4);
    RemoteData.s1=Remote::UP; RemoteData.s2=Remote::UP;
    step(true); step(true);
    assert(gimbal_motor.calls==5);
    pitch_recovery_fsm.Force_Enable_On_Next_Check();
    step(true);
    assert(gimbal_motor.calls==6);
    MotorRecoveryFSM wrap;
    wrap.Init(0xfffffff0U);
    assert(wrap.Should_Enable(false,0xfffffff0U));
    assert(!wrap.Should_Enable(false,0x53U));
    assert(wrap.Should_Enable(false,0x54U));
}
'''.replace("BRANCH", branch)
        self.compile_and_run(harness)

    def compile_and_run(self, harness):
        with tempfile.TemporaryDirectory(prefix="gimbal_recovery_") as temp:
            cpp = Path(temp) / "test.cpp"
            exe = Path(temp) / "test.exe"
            cpp.write_text(harness, encoding="utf-8")
            subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                            "-I", str(ROOT), str(cpp), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
