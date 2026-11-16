"""Check the watchdog-reset lock and remote rearm against task source."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class WatchdogRearmTest(unittest.TestCase):
    def test_remote_rearm_requires_valid_double_down(self):
        source = (ROOT / "RtosTask/remote_control_task.cpp").read_text(encoding="utf-8-sig")
        start = source.index("        taskENTER_CRITICAL();", source.index("for (;;)") )
        end = source.index("        taskEXIT_CRITICAL();", start) + len("        taskEXIT_CRITICAL();")
        branch = source[start:end]
        harness = r'''
#include <cassert>
#include <cstdint>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
namespace BSP::REMOTE_CONTROL { struct RemoteController { enum { DOWN = 2 }; }; }
struct Controller {
    bool connected = false; uint8_t s1 = 0, s2 = 0; float axis = 0;
    bool isConnected() { return connected; }
    uint8_t get_s1() { return s1; } uint8_t get_s2() { return s2; }
    float get_left_x() { return axis; } float get_left_y() { return axis; }
    float get_right_x() { return axis; } float get_right_y() { return axis; }
    uint16_t get_keyboard() { return 0; }
} remoteController;
bool arm_remote_received = false;
volatile uint32_t g_watchdog_rearm_required = 1;
bool step() {
    bool remote_online, arm_online; uint8_t s1, s2; uint16_t keyboard;
BRANCH
    return arm_online;
}
int main() {
    remoteController.s1 = remoteController.s2 = 2;
    assert(!step() && g_watchdog_rearm_required);
    remoteController.connected = true; assert(!step() && g_watchdog_rearm_required);
    arm_remote_received = true; remoteController.axis = -1;
    assert(!step() && g_watchdog_rearm_required);
    remoteController.axis = 0; remoteController.s1 = 1;
    assert(!step() && g_watchdog_rearm_required);
    remoteController.s1 = 2; assert(step() && !g_watchdog_rearm_required);
    remoteController.s1 = 1; assert(step() && !g_watchdog_rearm_required);
}
'''.replace("BRANCH", branch)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)
            (path / "test.cpp").write_text(harness)
            subprocess.run(["g++", "-std=c++17", str(path / "test.cpp"), "-o", str(path / "test.exe")], check=True)
            subprocess.run([str(path / "test.exe")], check=True)

    def test_lock_covers_motor_and_board_paths(self):
        pitch = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        gantry = (ROOT / "RtosTask/gantry_task.cpp").read_text(encoding="utf-8-sig")
        fire = (ROOT / "RtosTask/gimbal_task.cpp").read_text(encoding="utf-8-sig")
        remote = (ROOT / "RtosTask/remote_control_task.cpp").read_text(encoding="utf-8-sig")
        self.assertIn("if (!g_watchdog_rearm_required && !g_watchdog_reset_detected)", pitch)
        self.assertIn("if(!remoteController.isConnected() || g_watchdog_rearm_required", pitch)
        self.assertIn("configured && remote_online && !g_watchdog_rearm_required", gantry)
        self.assertIn("if (!remoteController.isConnected() || g_watchdog_rearm_required)", fire)
        self.assertIn("if (g_watchdog_rearm_required)", fire)
        self.assertIn("arm_remote_received && remote_online && !g_watchdog_rearm_required", remote)


if __name__ == "__main__":
    unittest.main()
