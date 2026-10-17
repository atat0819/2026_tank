"""Compile the task's recovery branch and exercise the real recovery FSM."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class RecoveryTest(unittest.TestCase):
    maxDiff = 1000

    def test_independent_dropout_and_recovery(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        self.assertTrue("static bool yaw_motor_was_offline" in task,
                        "dropout state must be independent per axis")
        begin = task.index("  // 电机掉线按轴处理：")
        end = task.index("  if (feedback_source_changed)", begin)
        dropout = task[begin:end]
        self.assertNotIn("continue;", dropout)
        begin = task.index("   // 电机恢复按轴处理：")
        end = task.index("   // 遥控恢复：", begin)
        recovery = task[begin:end]
        for axis, other in (("Yaw", "pitch"), ("Pitch", "yaw")):
            body = task.split(f"static void Reset{axis}ControlState()", 1)[1].split("\n}", 1)[0]
            self.assertNotIn(other + "_", body)
        # Compile the actual dropout/recovery branches to check partial recovery.
        harness = r'''
#include <cassert>
bool yaw_motor_was_offline=false, pitch_motor_was_offline=false;
int yaw_resets=0, pitch_resets=0;
float yaw_current_angle=1, pitch_current_angle=2;
void ResetYawControlState() { ++yaw_resets; }
void ResetPitchControlState() { ++pitch_resets; }
struct Fsm { int anchors=0; float target=0;
    void ReAnchor(float angle) { ++anchors; target=angle; }
} yaw_gimbal_fsm, pitch_gimbal_fsm;
void step(bool yaw_motor_online, bool pitch_motor_online) {
DROPOUT
RECOVERY
}
int main() {
    step(false,true);
    assert(yaw_resets==1 && pitch_resets==0);
    step(false,true);
    assert(yaw_resets==2 && pitch_resets==0);
    step(true,true);
    assert(yaw_gimbal_fsm.anchors==1 && pitch_gimbal_fsm.anchors==0);
    assert(yaw_gimbal_fsm.target==yaw_current_angle);
    step(true,false);
    assert(yaw_resets==3 && pitch_resets==1);
    step(false,false);
    assert(yaw_resets==4 && pitch_resets==2);
    step(false,true);
    assert(yaw_gimbal_fsm.anchors==1 && pitch_gimbal_fsm.anchors==1);
    assert(pitch_gimbal_fsm.target==pitch_current_angle);
    step(true,true);
    assert(yaw_gimbal_fsm.anchors==2 && pitch_gimbal_fsm.anchors==1);
    step(true,true);
    assert(yaw_resets==6 && pitch_resets==3);
}
'''.replace("DROPOUT", dropout).replace("RECOVERY", recovery)
        self.compile_and_run(harness)
        for axis in ("yaw", "pitch"):
            input_guard = f"if (!{axis}_motor_online)"
            input_begin = task.index(f"Struct_Gimbal_Input {axis}_input")
            input_end = task.index(f"{axis}_gimbal_fsm.Update({axis}_input", input_begin)
            axis_input = task[input_begin:input_end]
            self.assertIn(input_guard, axis_input)
            self.assertIn(f"{axis}_input = {{}};", axis_input)
            self.assertIn(f"{axis}_input.s1 = Remote::DOWN;", axis_input)
            self.assertIn(f"{axis}_input.s2 = Remote::DOWN;", axis_input)

    def test_task_recovery(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        self.assertIn("MotorRecoveryFSM gimbal_recovery_fsm[2]", task)
        self.assertIn('"../feeder_fsm/motor_recovery_fsm.hpp"', task)
        remote_fault = task.split("gimbal_remote_offline = 1;", 1)[1].split("continue;", 1)[0]
        for axis in (0, 1):
            self.assertIn(f"gimbal_recovery_fsm[{axis}].Force_Enable_On_Next_Check();", remote_fault)
            self.assertIn(f"gimbal_recovery_fsm[{axis}].Init(", task)
        begin = task.index("  // 电机恢复使能：")
        end = task.index("  // 电机掉线按轴处理：", begin)
        branch = task[begin:end]
        self.assertLess(begin, task.index("if (yaw_motor_online && yaw_motor_was_offline)"))
        harness = r'''
#include <cassert>
#include "feeder_fsm/motor_recovery_fsm.hpp"
namespace Remote { enum { UP=1, DOWN=2, MIDDLE=3 }; }
enum class InputSource { Remote, KeyMouse };
struct { int s1=Remote::UP, s2=Remote::UP; } RemoteData;
struct { InputSource GetSource() { return InputSource::Remote; } } input_dispatcher;
namespace BSP { namespace Motor { namespace DM { enum { MIT=0 }; } } }
struct { int calls[2]={0,0}; void On(int id, int) { ++calls[id-1]; } } gimbal_motor;
MotorRecoveryFSM gimbal_recovery_fsm[2];
unsigned now_tick;
unsigned HAL_GetTick() { return now_tick; }
void step(bool yaw_motor_online, bool pitch_motor_online) {
BRANCH
}
int main() {
    for (auto &fsm : gimbal_recovery_fsm) fsm.Init(0);
    step(true,true); // initial online edge
    step(true,true);
    assert(gimbal_motor.calls[0]==1 && gimbal_motor.calls[1]==1);
    now_tick=10; step(false,true); // first dropout: only yaw retries
    assert(gimbal_motor.calls[0]==2 && gimbal_motor.calls[1]==1);
    now_tick=109; step(false,true);
    assert(gimbal_motor.calls[0]==2);
    now_tick=110; step(false,true);
    assert(gimbal_motor.calls[0]==3);
    now_tick=111; step(true,false); // yaw recovery and pitch dropout
    assert(gimbal_motor.calls[0]==4 && gimbal_motor.calls[1]==2);
    RemoteData.s1=Remote::DOWN; RemoteData.s2=Remote::DOWN;
    now_tick=500; step(false,false); step(true,true);
    assert(gimbal_motor.calls[0]==4 && gimbal_motor.calls[1]==2);
    RemoteData.s1=Remote::UP; RemoteData.s2=Remote::UP;
    step(true,true); step(true,true);
    assert(gimbal_motor.calls[0]==5 && gimbal_motor.calls[1]==3);
    // Same re-arm operation as the remote-fault branch, even with live feedback.
    for (auto &fsm : gimbal_recovery_fsm) fsm.Force_Enable_On_Next_Check();
    step(true,true);
    assert(gimbal_motor.calls[0]==6 && gimbal_motor.calls[1]==4);
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
