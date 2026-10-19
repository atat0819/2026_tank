"""Run the real UART-owned HI14 control snapshot without an IMU/VOFA task."""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
HOST = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include "remote_task.hpp"
using UBaseType_t=unsigned;
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define taskENTER_CRITICAL_FROM_ISR() (0U)
#define taskEXIT_CRITICAL_FROM_ISR(x) ((void)(x))
uint32_t tick=0;
uint32_t HAL_GetTick() { return tick; }
'''
MAIN = r'''
int main() {
    ImuControlSnapshot s;
    GetImuControlSnapshot(s); assert(!s.valid);
    IMU_Float_t data{}; data.pitch=12.5f; data.gyr_pitch=-7.25f; data.roll=1.5f;
    tick=200; PublishHi14ImuFromISR(data,tick,true);
    GetImuControlSnapshot(s);
    assert(s.valid && s.pitch_deg==12.5f && s.pitch_rate_dps==-7.25f && s.tick==200);
    assert(s.roll_deg==1.5f);
    tick=219; GetImuControlSnapshot(s); assert(s.valid);
    tick=220; GetImuControlSnapshot(s); assert(s.valid);
    tick=299; GetImuControlSnapshot(s); assert(s.valid);
    tick=300; GetImuControlSnapshot(s); assert(!s.valid && s.tick==200);
    tick=0xfffffff8U; PublishHi14ImuFromISR(data,tick,true);
    tick=5; GetImuControlSnapshot(s); assert(s.valid);
    tick=91; GetImuControlSnapshot(s); assert(s.valid);
    tick=92; GetImuControlSnapshot(s); assert(!s.valid);
    PublishHi14ImuFromISR(data,tick,true);
    PublishHi14ImuFromISR(data,tick,false);
    GetImuControlSnapshot(s); assert(!s.valid);
    data.pitch=NAN; PublishHi14ImuFromISR(data,tick,true);
    GetImuControlSnapshot(s); assert(!s.valid);
    data.pitch=1; data.gyr_pitch=NAN; PublishHi14ImuFromISR(data,tick,true);
    GetImuControlSnapshot(s); assert(!s.valid);
    data.gyr_pitch=-2; PublishHi14ImuFromISR(data,tick,true);
    GetImuControlSnapshot(s); assert(s.valid && s.pitch_deg==1 && s.pitch_rate_dps==-2);
    data.roll=NAN; PublishHi14ImuFromISR(data,tick,true);
    GetImuControlSnapshot(s); assert(!s.valid);
    data.roll=-3; PublishHi14ImuFromISR(data,tick,true);
    GetImuControlSnapshot(s); assert(s.valid && s.roll_deg==-3);
}
'''
remote=(ROOT/"RtosTask/remote_task.cpp").read_text(encoding="utf-8")
start=remote.index("// ================= HI14 快照")
end=remote.index("// ================= 中断回调函数", start)
with tempfile.TemporaryDirectory(prefix="hi14-snapshot-") as directory:
    work=Path(directory)
    (work/"RtosTask").mkdir()
    shutil.copy2(ROOT/"RtosTask/remote_task.hpp", work/"RtosTask/remote_task.hpp")
    header=work/"user/core/BSP/Hi14/HI14.hpp"
    header.parent.mkdir(parents=True)
    shutil.copy2(ROOT/"user/core/BSP/Hi14/HI14.hpp", header)
    for name, content in (
        ("FreeRTOS.h", "#pragma once\n#include <stdint.h>\n"),
        ("queue.h", "#pragma once\nusing QueueHandle_t=void*;\n"),
        ("cmsis_os.h", "#pragma once\n"),
        ("user/core/BSP/RemoteControl/DT7.hpp", "#pragma once\nnamespace BSP { namespace REMOTE_CONTROL { class RemoteController; } }\n"),
    ):
        path=work/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    (work/"test.cpp").write_text(HOST+remote[start:end]+MAIN, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-I", ".", "-I", "RtosTask", "test.cpp", "-o", "test.exe"], cwd=work, check=True)
    subprocess.run([str(work/"test.exe")], check=True)
print("PASS: HI14 control snapshot remains functional without IMU task, including expiry and error recovery")
