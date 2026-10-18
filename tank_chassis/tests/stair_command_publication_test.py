"""Exercise real producer/stop critical sections with an interleaved schedule."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
producer = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
consumer = (ROOT / "RtosTask/up_stair.cpp").read_text(encoding="utf-8")

def critical_section(source, anchor):
    start = source.index("taskENTER_CRITICAL();", source.index(anchor))
    end = source.index("taskEXIT_CRITICAL();", start) + len("taskEXIT_CRITICAL();")
    return source[start:end]

publish = critical_section(producer, "一次 B 同时发布两个明确目标")
stop = critical_section(consumer, "锁存 NORMAL")
assert producer.index("const uint32_t stair_sample_generation") < producer.index(
    "const uint32_t now_tick = HAL_GetTick();", producer.index("const uint32_t stair_sample_generation"))

host = r'''
#include "fsm/stair_mode_policy.hpp"
#include <cassert>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
StairCommand stair_command;
uint32_t stair_command_generation = 0;
struct Keys { bool imu_mode_request, normal_mode_request, stair_toggle, valid; };
void publish(uint32_t stair_sample_generation, Keys keyboard_cmd,
             bool source_changed=false) {
    struct { bool zero_all_torque=false; } stair_policy;
'''
host += publish + "\n}\nvoid stop() {\n" + stop + "\n}\n"
host += r'''
int main() {
    publish(0, {true,false,false,true});
    assert(stair_command.mode == StairMode::IMU);
    publish(0, {false,false,true,true});
    publish(0, {false,false,true,true});
    assert(stair_command.rear_retract_requested);
    const auto old_generation = stair_command_generation;
    // Producer samples V on source A, then consumer sees B and stops.
    stop();
    assert(stair_command.mode == StairMode::NORMAL);
    publish(old_generation, {true,false,false,true});
    assert(stair_command.mode == StairMode::NORMAL);
    publish(old_generation, {false,false,true,true});
    assert(!stair_command.front_target_requested);
    publish(stair_command_generation, {false,false,false,true});
    assert(stair_command.mode == StairMode::NORMAL);
    // Only a new valid V restores IMU after the safety latch.
    publish(stair_command_generation, {true,false,false,true});
    assert(stair_command.mode == StairMode::IMU);
    publish(stair_command_generation, {false,false,false,true}, true);
    assert(stair_command.mode == StairMode::NORMAL);
    publish(stair_command_generation, {true,false,false,true});
    publish(stair_command_generation, {false,false,true,true});
    publish(stair_command_generation, {false,false,false,false});
    assert(!stair_command.front_target_requested);
}
'''
with tempfile.TemporaryDirectory(prefix="stair-publication-") as directory:
    work = Path(directory)
    test = work / "test.cpp"
    test.write_text(host, encoding="utf-8")
    executable = work / "test.exe"
    subprocess.run(["g++", "-std=c++17", "-I", ROOT.as_posix(),
                    test.as_posix(), "-o", executable.as_posix()], check=True)
    subprocess.run([str(executable)], check=True)
print("PASS: atomic mechanism commands, safety latch and stale publication rejection")
