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
    uint8_t s1 = 0, s2 = 0;
    uint8_t get_s1() { return s1; } uint8_t get_s2() { return s2; }
} remoteController;
bool rs485_online = false;
bool Gimbal485_RemoteOnline() { return rs485_online; }
volatile uint32_t g_watchdog_rearm_required = 1;
bool step() {
    bool remote_online; uint8_t s1, s2;
BRANCH
    return remote_online && !g_watchdog_rearm_required;
}
int main() {
    remoteController.s1 = remoteController.s2 = 2;
    assert(!step() && g_watchdog_rearm_required);
    rs485_online = true;
    remoteController.s2 = 1;
    assert(!step() && g_watchdog_rearm_required);
    remoteController.s2 = 2; remoteController.s1 = 1;
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
        self.assertIn("const bool remote_enabled = yaw_mouse.online && !g_watchdog_rearm_required;", pitch)
        pitch_lock = pitch.split("if (!remote_enabled ||", 1)[1].split("\n         else", 1)[0]
        self.assertIn("pitch_motor.Off(1, BSP::Motor::DM::MIT);", pitch_lock)
        self.assertIn("configured && remote_online && !g_watchdog_rearm_required", gantry)
        self.assertIn("if (!Gimbal485_RemoteOnline() || g_watchdog_rearm_required)", fire)
        self.assertIn("if (g_watchdog_rearm_required)", fire)


if __name__ == "__main__":
    unittest.main()
