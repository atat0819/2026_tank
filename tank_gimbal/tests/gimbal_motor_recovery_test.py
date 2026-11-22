"""Compile the two-axis stop path and Pitch drive-status recovery."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class RecoveryTest(unittest.TestCase):
    def test_recovery_implementation_is_project_local(self):
        header = (ROOT / "feeder_fsm/motor_recovery_fsm.hpp").read_text(encoding="utf-8-sig")
        self.assertIn("class MotorRecoveryFSM", header)
        self.assertIn("Request Update(bool control_allowed, uint8_t drive_status, uint32_t now_tick)", header)

    def test_pitch_starts_disabled(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        startup = task.split('extern "C" void can_send_task', 1)[1].split("for(;;)", 1)[0]
        self.assertIn("pitch_motor.Off(1, BSP::Motor::DM::MIT);", startup)
        self.assertNotIn("pitch_motor.On(", startup)

    def test_remote_fault_reaches_common_loop_tail_without_normal_control(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        gate_begin = task.index("        const bool remote_enabled =")
        gate = task[gate_begin:task.index("\n", gate_begin)]
        switches_begin = task.index("        const bool valid_switches =", gate_begin)
        gate += "\n" + task[switches_begin:task.index(";", switches_begin) + 1]
        double_down_begin = task.index("        const bool double_down =", switches_begin)
        gate += "\n" + task[double_down_begin:task.index(";", double_down_begin) + 1]
        begin = task.index("         if (!remote_enabled ||", gate_begin)
        end = task.index("\n         else", begin)
        branch = task[begin:end]
        resume_begin = task.index("        if (remote_was_offline)", end)
        resume = task[resume_begin:task.index("        input_dispatcher.Update(", resume_begin)]
        harness = r'''
#include <cassert>
#include "feeder_fsm/motor_recovery_fsm.hpp"
struct { bool online = true; } yaw_mouse;
struct Remote { enum { UP=1, DOWN=2, MIDDLE=3 }; };
struct { unsigned s1=1, s2=3; } remote_input;
MotorRecoveryFSM pitch_recovery_fsm, yaw_recovery_fsm;
struct { bool Is_Target_Valid() { return true; } } yaw_fsm;
bool yaw_wait_for_release = false;
bool yaw_was_ready = true, pitch_was_ready = true;
int gimbal_remote_offline = 0, remote_was_offline = 0, g_watchdog_rearm_required = 0;
int resets = 0, yaw_resets = 0, normal = 0, tail = 0, delays = 0;
uint32_t tick = 500U, yaw_last_off_tick = 0U;
uint32_t HAL_GetTick() { return tick; }
struct Dispatcher {
    int clears = 0;
    void Update(unsigned s1, unsigned s2, unsigned keys, bool left, bool right) {
        assert(s1 == 0 && s2 == 0 && keys == 0 && !left && !right);
        ++clears;
    }
} input_dispatcher;
void ResetPitchControlState() { ++resets; }
void ResetYawControlState() { ++yaw_resets; }
namespace BSP::Motor::DM { enum { MIT=0 }; }
struct Motor {
    int off_frames = 0, control_frames = 0;
    void Off(int id, int mode) { assert(id == 1 && mode == BSP::Motor::DM::MIT); ++off_frames; }
    void ctrl_Mit(int, float, float, float, float, float) { ++control_frames; }
} pitch_motor, yaw_motor;
void vTaskDelay(int) { ++delays; }
void step() {
    for (int i = 0; i < 1; ++i) {
GATE
BRANCH
        else {
RESUME
            remote_was_offline = false;
            ++normal;
        }
        ++tail;
        vTaskDelay(1);
    }
}
int main() {
    yaw_mouse.online = false; step();
    assert(gimbal_remote_offline && remote_was_offline);
    assert(pitch_motor.off_frames == 1 && yaw_motor.off_frames == 1 && normal == 0);
    assert(yaw_wait_for_release && !yaw_was_ready && !pitch_was_ready);
    tick = 599U; step();
    assert(pitch_motor.off_frames == 1 && yaw_motor.off_frames == 1);
    tick = 600U; step();
    assert(pitch_motor.off_frames == 2 && yaw_motor.off_frames == 2);
    yaw_mouse.online = true; step();
    assert(pitch_motor.off_frames == 2 && yaw_motor.off_frames == 2 && normal == 1);
    tick = 601U;
    g_watchdog_rearm_required = 1; step();
    assert(pitch_motor.off_frames == 3 && yaw_motor.off_frames == 3 && normal == 1);
    assert(pitch_motor.control_frames == 0 && yaw_motor.control_frames == 0);
    assert(resets == 4 && input_dispatcher.clears == 4 && yaw_resets == 4);
    assert(tail == 5 && delays == 5);
}
'''.replace("GATE", gate).replace("BRANCH", branch).replace("RESUME", resume)
        self.compile_and_run(harness)

    def test_pitch_dropout_and_recovery(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        begin = task.index("  // Pitch 驱动未确认使能时")
        recovery = task[begin:task.index("  remote_was_offline = false;", begin) + len("  remote_was_offline = false;")]
        self.assertNotIn("continue;", recovery)
        inputs = task.split("Struct_Gimbal_Input pitch_input", 1)[1].split(
            "pitch_gimbal_fsm.Update(pitch_input", 1)[0]
        for value in ("if (!pitch_motor_ready)", "pitch_input = {};",
                      "pitch_input.s1 = Remote::DOWN;", "pitch_input.s2 = Remote::DOWN;"):
            self.assertIn(value, inputs)
        harness = r'''
#include <cassert>
bool pitch_motor_was_unready=false, remote_was_offline=false;
int pitch_resets=0;
int mouse_delta_y=0;
float pitch_current_angle=2;
void ResetPitchControlState() { ++pitch_resets; }
struct Fsm { int anchors=0; float target=0;
    void ReAnchor(float angle) { ++anchors; target=angle; }
} pitch_gimbal_fsm;
void step(bool pitch_motor_ready) {
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
    remote_was_offline=true;
    mouse_delta_y=7;
    step(true);
    assert(pitch_resets==6 && pitch_gimbal_fsm.anchors==3);
    assert(!remote_was_offline && mouse_delta_y==0);
}
'''.replace("RECOVERY", recovery)
        self.compile_and_run(harness)

    def test_task_recovery(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        self.assertIn("MotorRecoveryFSM pitch_recovery_fsm;", task)
        remote_fault = task.split("if (!remote_enabled ||", 1)[1].split("\n         else", 1)[0]
        self.assertIn("pitch_recovery_fsm.Update(false,", remote_fault)
        self.assertIn("pitch_recovery_fsm.Reset();", task)
        begin = task.index("  const bool pitch_motor_ready =")
        branch = task[begin:task.index(";", begin) + 1]
        begin = task.index("static bool UpdateMotorEnable(")
        motor_enable = task[begin:task.index("\n}", begin) + 2]
        harness = r'''
#include <cassert>
#include "feeder_fsm/motor_recovery_fsm.hpp"
namespace BSP::Motor::DM {
enum { MIT=0 };
template <unsigned N> struct DMMotorBase {
    int calls=0, stops=0, polls=0;
    void On(int slot, int) { assert(slot==1); ++calls; }
    void Off(int slot, int) { assert(slot==1); ++stops; }
    void ctrl_Mit(int slot, float p, float v, float kp, float kd, float torque) {
        assert(slot==1 && p==0 && v==0 && kp==0 && kd==0 && torque==0);
        ++polls;
    }
};
}
BSP::Motor::DM::DMMotorBase<1> pitch_motor, yaw_motor;
MOTOR_ENABLE
MotorRecoveryFSM pitch_recovery_fsm;
bool pitch_was_ready=false;
unsigned now_tick=0;
unsigned HAL_GetTick() { return now_tick; }
bool step(bool pitch_motor_online, uint8_t pitch_status) {
BRANCH
    return pitch_motor_ready;
}
int main() {
    assert(!step(false, 0));
    assert(pitch_motor.calls==0 && pitch_motor.polls==1);
    assert(!step(true, 0) && pitch_motor.calls==1 && pitch_recovery_fsm.HasPendingOn());
    now_tick=99;
    assert(!step(true, 0) && pitch_motor.calls==1);
    now_tick=100;
    assert(!step(true, 0) && pitch_motor.calls==2);
    assert(step(true, 1) && pitch_motor.calls==2 && !pitch_recovery_fsm.HasPendingOn());
    assert(!step(true, 0) && pitch_motor.calls==3); // Still online, but the driver restarted.
    assert(!step(true, 0xA) && pitch_motor.calls==3);
    assert(step(true, 1) && pitch_motor.calls==3);
    assert(!step(false, 1) && pitch_motor.stops==1 && pitch_motor.calls==3);
    assert(!step(false, 0) && pitch_motor.polls==2 && pitch_motor.calls==3);
    assert(step(true, 1) && pitch_motor.calls==3 && yaw_motor.calls==0);
    MotorRecoveryFSM wrap;
    using Request = MotorRecoveryFSM::Request;
    assert(wrap.Update(true,0U,0xfffffff0U)==Request::On);
    assert(wrap.Update(true,0U,0x53U)==Request::None);
    assert(wrap.Update(true,0U,0x54U)==Request::On);
}
'''.replace("MOTOR_ENABLE", motor_enable).replace("BRANCH", branch)
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
