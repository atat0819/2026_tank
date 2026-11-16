"""Compile actual single-Pitch dropout/recovery branches and the recovery FSM."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class RecoveryTest(unittest.TestCase):
    def test_remote_fault_reaches_common_loop_tail_without_normal_control(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        begin = task.index("         if(!remoteController.isConnected()")
        end = task.index("         else {", begin)
        branch = task[begin:end]
        harness = r'''
#include <cassert>
struct Remote {
    bool connected = true; float value = 0;
    bool isConnected() { return connected; }
    float get_left_y() { return value; }
    float get_left_x() { return value; }
    float get_right_x() { return value; }
    float get_right_y() { return value; }
} remoteController;
struct Recovery { int calls = 0; void Force_Enable_On_Next_Check() { ++calls; } } pitch_recovery_fsm;
struct Data { int value; } RemoteData = {9};
int gimbal_remote_offline = 0, remote_was_offline = 0, g_watchdog_rearm_required = 0;
int resets = 0, zero_frames = 0, zero_motor = 0, normal = 0, tail = 0, delays = 0;
void RS485_SendBoardData(float a, int b, float c, float d, unsigned e, unsigned f, unsigned g) {
    assert(a == 0 && b == 0 && c == 0 && d == 0 && e == 0 && f == 0 && g == 0); ++zero_frames;
}
void ResetGimbalControlState() { ++resets; }
struct Motor { void ctrl_Mit(int id, float a, float b, float c, float d, float e) {
    assert(id == 1 && a == 0 && b == 0 && c == 0 && d == 0 && e == 0); ++zero_motor;
} } pitch_motor;
void vTaskDelay(int) { ++delays; }
void step() {
    for (int i = 0; i < 1; ++i) {
BRANCH
        else { ++normal; }
        ++tail;
        vTaskDelay(1);
    }
}
int main() {
    remoteController.connected = false; step();
    assert(gimbal_remote_offline && remote_was_offline && RemoteData.value == 0);
    assert(resets == 1 && zero_frames == 1 && zero_motor == 1 && normal == 0 && tail == 1 && delays == 1);
    remoteController.connected = true; remoteController.value = -1; step();
    assert(resets == 2 && zero_frames == 2 && zero_motor == 2 && normal == 0 && tail == 2 && delays == 2);
    remoteController.value = 0; step();
    assert(normal == 1 && tail == 3 && delays == 3);
    g_watchdog_rearm_required = 1; step();
    assert(resets == 3 && zero_frames == 3 && zero_motor == 3 && normal == 1 && tail == 4 && delays == 4);
    assert(pitch_recovery_fsm.calls == 3);
}
'''.replace("BRANCH", branch)
        self.compile_and_run(harness)

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
        remote_fault = task.split("gimbal_remote_offline = 1;", 1)[1].split("         else {", 1)[0]
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
struct { int calls=0; void On(int slot, int) { assert(slot==1); ++calls; } } pitch_motor;
MotorRecoveryFSM pitch_recovery_fsm;
unsigned now_tick=0;
unsigned HAL_GetTick() { return now_tick; }
void step(bool pitch_motor_online) {
BRANCH
}
int main() {
    pitch_recovery_fsm.Init(0);
    step(true); step(true);
    assert(pitch_motor.calls==1);
    now_tick=10; step(false);
    assert(pitch_motor.calls==2);
    now_tick=109; step(false);
    assert(pitch_motor.calls==2);
    now_tick=110; step(false);
    assert(pitch_motor.calls==3);
    now_tick=111; step(true);
    assert(pitch_motor.calls==4);
    RemoteData.s1=Remote::DOWN; RemoteData.s2=Remote::DOWN;
    now_tick=500; step(false); step(true);
    assert(pitch_motor.calls==4);
    RemoteData.s1=Remote::UP; RemoteData.s2=Remote::UP;
    step(true); step(true);
    assert(pitch_motor.calls==5);
    pitch_recovery_fsm.Force_Enable_On_Next_Check();
    step(true);
    assert(pitch_motor.calls==6);
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
