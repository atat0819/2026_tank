"""Exercise the UART5 input snapshot and watchdog rearm gate."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class WatchdogRearmTest(unittest.TestCase):
    def test_only_fresh_uart5_double_down_can_rearm(self):
        source = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        header = (ROOT / "RtosTask/can_send_task.hpp").read_text(encoding="utf-8-sig")
        self.assertNotIn("ControlInputSource::GIMBAL", source)
        snapshot_type = header[header.index("struct ControlInputSnapshot"):header.index("ControlInputSnapshot GetControlInputSnapshot")]
        publication = source[source.index("float yaw_offset_deg ="):source.index("StairCommand stair_command;")]
        harness = r'''
#include <cassert>
#include <cstdint>
#include "communication/boards_communication_uart_485.hpp"
#include "fsm/control_input_selector.hpp"
#include "fsm/stair_mode_policy.hpp"
uint32_t tick=100;
bool in_critical=false;
unsigned locked_clock_reads=0;
#define taskENTER_CRITICAL() do { assert(!in_critical); in_critical=true; } while (0)
#define taskEXIT_CRITICAL() do { assert(in_critical); in_critical=false; } while (0)
uint32_t HAL_GetTick() { if (in_critical) ++locked_clock_reads; return tick; }
volatile uint32_t g_watchdog_rearm_required=1;
bool local_remote_received=false; uint32_t local_remote_last_tick=0;
uint16_t local_remote_keyboard=0; int16_t local_remote_mouse_x=0;
bool local_remote_yaw_mouse_active=false;
struct { uint8_t s1=1,s2=1; float vx=0,vy=0,wz=0; } remoteData;
SNAPSHOT
PUBLICATION
void publish_local(uint8_t s1,uint8_t s2) {
    local_remote_received=true;
    local_remote_last_tick=tick;
    remoteData.s1=s1;
    remoteData.s2=s2;
}
void stopped() {
    const auto input=GetControlInputSnapshot();
    assert(g_watchdog_rearm_required && input.source==ControlInputSource::NONE);
    assert(input.vx==0 && input.vy==0 && input.wz==0 && input.keyboard==0);
    auto policy=EvaluateStairModePolicy(input.s1,input.s2,false,false);
    assert(policy.zero_all_torque && policy.control_fault);
}
int main() {
    stopped();
    Chassis485_ApplyData({3.0f});
    assert(yaw_offset_deg==3.0f);
    stopped();
    publish_local(1,1); stopped();
    publish_local(2,2);
    tick+=100;
    Chassis485_ApplyData({4.0f});
    stopped(); // Fresh Yaw feedback cannot authorize stale remote input.
    publish_local(2,2);
    auto input=GetControlInputSnapshot();
    assert(!g_watchdog_rearm_required && input.source==ControlInputSource::LOCAL);
    assert(input.s1==2 && input.s2==2);

    // A fresh ISR frame arrives after the caller samples time.
    g_watchdog_rearm_required=1; tick=1000;
    const uint32_t sampled_tick=tick;
    ++tick; publish_local(2,2);
    const unsigned reads_before=locked_clock_reads;
    input=GetControlInputSnapshot();
    assert(tick-sampled_tick==1 && locked_clock_reads==reads_before+1);
    assert(!g_watchdog_rearm_required && input.source==ControlInputSource::LOCAL);

    // Unsigned freshness also holds across the 32-bit tick rollover.
    g_watchdog_rearm_required=1; tick=UINT32_MAX; publish_local(2,2);
    ++tick;
    input=GetControlInputSnapshot();
    assert(!g_watchdog_rearm_required && input.source==ControlInputSource::LOCAL);
    tick=99;
    Chassis485_ApplyData({5.0f});
    assert(GetControlInputSnapshot().source==ControlInputSource::NONE);
}
'''.replace("SNAPSHOT", snapshot_type).replace("PUBLICATION", publication)
        with tempfile.TemporaryDirectory() as temp:
            work = Path(temp)
            (work / "test.cpp").write_text(harness, encoding="utf-8")
            exe = work / "test.exe"
            subprocess.run(["g++", "-std=c++17", "-I", str(ROOT), str(work / "test.cpp"), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
