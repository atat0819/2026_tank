"""编译实际 RS485 驱动，验证帧内容、CRC、发送忙时缓冲区不被覆盖。"""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
task = (root / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
assert "chassis_mouse_delta_pending" not in task, "mouse X must publish latest state, not consume a pending delta"
mouse_snapshot = task[task.index("        RemoteData.mouse_x"):task.index("\n", task.index("        RemoteData.mouse_x"))]
normal_call = task[task.index("const bool chassis_remote_mode"):task.index(" // tick 用于分频发送")]
mouse_collection = task[task.index("         int32_t mouse_delta_x = 0;"):task.index("         if(!remoteController.isConnected()")]
remote_source = (root / "user/core/BSP/RemoteControl/DT7.cpp").read_text(encoding="utf-8")
begin = remote_source.index("void RemoteController::ConsumeMouseDelta(")
consume_mouse = remote_source[begin:remote_source.index("\n}", begin) + 2]
consume_mouse = consume_mouse.replace("void RemoteController::ConsumeMouseDelta", "void ConsumeMouseDelta")
offline_call = task[task.index("            // 遥控失联仍发送完整快照"):task.index("            ResetGimbalControlState();", task.index("            RS485_SendBoardData("))]
task_cases = r'''
enum class InputSource { Remote, KeyMouse, Other };
uint32_t __get_PRIMASK() { return 0; }
void __disable_irq() {}
void __enable_irq() {}
int32_t observed_pitch_delta_y = 0;
struct { float chassis_vx=0.75f, chassis_vy=-0.25f; uint8_t s1=3, s2=3; int16_t mouse_x=0; } RemoteData;
struct {
    InputSource source=InputSource::Remote;
    InputSource GetSource() { return source; }
    uint16_t GetKeyboardMask() { return 0x4321; }
} input_dispatcher;
struct {
    int16_t stick=-660, mouse=321;
    int16_t get_right_stick_x() { return stick; }
    int16_t get_mouseX() { return mouse; }
    int32_t pending_mouse_x_=0, pending_mouse_y_=0;
    void receive_mouse(int16_t dx,int16_t dy=0) { mouse=dx; pending_mouse_x_+=dx; pending_mouse_y_+=dy; }
''' + consume_mouse + r'''
} remoteController;
void task_snapshot(bool offline) {
''' + mouse_collection + r'''
    observed_pitch_delta_y = mouse_delta_y;
    if (offline) {
''' + offline_call + r'''
        return;
    }
''' + mouse_snapshot + normal_call + r'''
}
'''
stub = r'''
#pragma once
#include <cstdint>
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY };
enum { HAL_UART_STATE_READY, HAL_UART_STATE_BUSY_TX };
struct UART_HandleTypeDef { unsigned gState=HAL_UART_STATE_READY; };
extern uint32_t now;
inline uint32_t HAL_GetTick() { return now; }
'''
uart = r'''
#pragma once
#include "stm32h7xx_hal.h"
namespace HAL::UART {
struct Data { uint8_t *buffer; uint16_t size; };
extern UART_HandleTypeDef handle;
extern unsigned attempts;
extern bool accept;
extern uint8_t *active;
extern uint16_t active_size;
struct Device {
    UART_HandleTypeDef *get_handle() { return &handle; }
    bool transmit_dma(const Data &d) {
        ++attempts;
        if (!accept) return false;
        active=d.buffer; active_size=d.size; handle.gState=HAL_UART_STATE_BUSY_TX;
        return true;
    }
};
struct Bus { Device device; Device &get_uart3() { return device; } };
inline Bus &get_uart_bus_instance() { static Bus bus; return bus; }
}
'''
checks = r'''
#include <cassert>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include "communication_between_boards/boards_communication_uart_485.hpp"
#include "user/core/HAL/UART/uart_hal.hpp"
uint32_t now=100;
namespace HAL::UART {
UART_HandleTypeDef handle;
unsigned attempts=0;
bool accept=true;
uint8_t *active=nullptr;
uint16_t active_size=0;
}
// 独立按正向多项式计算，避免复用生产代码 CRC 查表。
uint16_t expected_crc(const uint8_t *p, unsigned n) {
    uint16_t crc=0xffff;
    for (unsigned i=0; i<n; ++i) {
        uint8_t reversed=0;
        for (unsigned b=0; b<8; ++b) reversed|=((p[i]>>b)&1)<<(7-b);
        crc^=uint16_t(reversed)<<8;
        for (unsigned b=0; b<8; ++b) crc=(crc&0x8000)?(crc<<1)^0x1021:crc<<1;
    }
    uint16_t out=0;
    for (unsigned b=0; b<16; ++b) out|=((crc>>b)&1)<<(15-b);
    return out;
}
float read_float(unsigned position) {
    float value; memcpy(&value,HAL::UART::active+position,4); return value;
}
int16_t read_int16(unsigned position) {
    int16_t value; memcpy(&value,HAL::UART::active+position,2); return value;
}
int main() {
    using namespace HAL::UART;
    assert(RS485_SendBoardData(-660,321,0.25f,-0.5f,3,1,-30.0f,-12.5f,0x1234)==HAL_OK);
    assert(active_size==28 && attempts==1);
    assert(active[0]==0x78 && active[1]==0x91);
    assert(read_int16(2)==-660 && read_int16(4)==321);
    assert(active[2]==0x6c && active[3]==0xfd && active[4]==0x41 && active[5]==0x01);
    assert(read_float(6)==0.25f && read_float(10)==-0.5f);
    assert(active[14]==3 && active[15]==1);
    assert(read_float(16)==330.0f && read_float(20)==-12.5f);
    assert(active[24]==0x34 && active[25]==0x12);
    uint16_t crc=expected_crc(active,26);
    assert(active[26]==(crc&255) && active[27]==(crc>>8));
    uint8_t saved[28]; memcpy(saved,active,28);
    assert(RS485_SendBoardData(660,-32768,0,0,0,0,0,0,0)==HAL_BUSY);
    assert(attempts==1 && memcmp(saved,active,28)==0);
    handle.gState=HAL_UART_STATE_READY; now+=5;
    assert(RS485_SendBoardData(0,0,0,0,0,0,720.0f,0,0)==HAL_OK);
    for(unsigned i=2;i<26;++i) assert(active[i]==0);
    crc=expected_crc(active,26);
    assert(active[26]==(crc&255) && active[27]==(crc>>8));
    handle.gState=HAL_UART_STATE_READY; now+=5; accept=false;
    assert(RS485_SendBoardData(0,0,0,0,0,0,0,0,0)==HAL_ERROR);
    accept=true;
    assert(RS485_SendBoardData(660,-32768,1,2,1,2,361.5f,7,1)==HAL_OK);
    assert(read_int16(2)==660 && read_int16(4)==-32768);
    assert(read_float(16)==1.5f && read_float(20)==7);

    handle.gState=HAL_UART_STATE_READY; now+=5;
    task_snapshot(false);
    assert(read_int16(2)==-660 && read_int16(4)==0);
    assert(read_float(6)==0.75f && read_float(10)==-0.25f);
    assert(active[24]==0 && active[25]==0);
    assert(read_float(16)==0 && read_float(20)==0);
    input_dispatcher.source=InputSource::KeyMouse;
    remoteController.receive_mouse(10,7);
    handle.gState=HAL_UART_STATE_READY; now+=5;
    task_snapshot(false);
    assert(read_int16(2)==0 && read_int16(4)==10 && observed_pitch_delta_y==7);
    assert(read_float(6)==0 && read_float(10)==0);
    assert(active[14]==3 && active[15]==3 && active[24]==0x21 && active[25]==0x43);
    // Repeat the same latest DT7 X state; Pitch Y remains a one-shot increment.
    for (int i=0;i<5;++i) {
        handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
        assert(read_int16(4)==10 && observed_pitch_delta_y==0);
    }
    remoteController.receive_mouse(10,7);
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
    assert(read_int16(4)==10 && observed_pitch_delta_y==7);
    remoteController.receive_mouse(0);
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
    assert(read_int16(4)==0 && observed_pitch_delta_y==0);
    for (int16_t value : {-32768,32767}) {
        remoteController.receive_mouse(value);
        handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
        assert(read_int16(4)==value);
        handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
        assert(read_int16(4)==value);
    }
    // DMA busy preserves in-flight bytes. Later TX uses -20, not the sum 100-20.
    memcpy(saved,active,28);
    remoteController.receive_mouse(100,4); task_snapshot(false);
    assert(observed_pitch_delta_y==4);
    remoteController.receive_mouse(-20,5); task_snapshot(false);
    assert(observed_pitch_delta_y==5 && memcmp(saved,active,28)==0);
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
    assert(read_int16(4)==-20 && observed_pitch_delta_y==0);
    // Original 2ms reply gate and reply-immediate retry are unchanged.
    const unsigned previous_attempts=attempts;
    handle.gState=HAL_UART_STATE_READY; ++now;
    remoteController.receive_mouse(7); task_snapshot(false);
    assert(attempts==previous_attempts && read_int16(4)==-20);
    ++now; task_snapshot(false);
    assert(attempts==previous_attempts+1 && read_int16(4)==7);
    handle.gState=HAL_UART_STATE_READY; RS485_ReplyReceived(); task_snapshot(false);
    assert(attempts==previous_attempts+2 && active_size==28 && read_int16(4)==7);
    // Failed TX does not queue an old mouse state.
    handle.gState=HAL_UART_STATE_READY; now+=5; accept=false;
    remoteController.receive_mouse(-30); task_snapshot(false);
    remoteController.receive_mouse(45); accept=true; task_snapshot(false);
    assert(read_int16(4)==45);
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
    assert(read_int16(4)==45);
    // Zero updates during busy TX supersede the previous nonzero state.
    remoteController.receive_mouse(0); task_snapshot(false);
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
    assert(read_int16(4)==0);
    remoteController.receive_mouse(40);
    input_dispatcher.source=InputSource::Remote;
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
    assert(read_int16(4)==0);
    input_dispatcher.source=InputSource::Other;
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
    assert(read_int16(4)==0);
    input_dispatcher.source=InputSource::KeyMouse;
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(true);
    assert(read_int16(2)==0 && read_int16(4)==0 && active[14]==0 && active[15]==0);
    assert(read_float(6)==0 && read_float(10)==0 && active[24]==0 && active[25]==0);
    remoteController.receive_mouse(0);
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
    assert(read_int16(4)==0);

}
'''
with tempfile.TemporaryDirectory(prefix="boards485_") as directory:
    work = Path(directory)
    (work / "communication_between_boards").mkdir()
    for name in ("boards_communication_uart_485.cpp", "boards_communication_uart_485.hpp"):
        shutil.copyfile(root / "communication_between_boards" / name, work / "communication_between_boards" / name)
    (work / "user/core/HAL/UART").mkdir(parents=True)
    (work / "user/core/HAL/UART/uart_hal.hpp").write_text(uart, encoding="utf-8")
    (work / "stm32h7xx_hal.h").write_text(stub, encoding="utf-8")
    (work / "main.h").write_text('#include "stm32h7xx_hal.h"\n', encoding="utf-8")
    (work / "test.cpp").write_text(checks.replace("int main() {", task_cases + "\nint main() {"), encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-I", ".", "test.cpp",
                    "communication_between_boards/boards_communication_uart_485.cpp",
                    "-o", "test.exe"], cwd=work, check=True)
    subprocess.run([str(work / "test.exe")], check=True)
print("PASS: 28-byte RS485 frame, latest mouse X state, one-shot Pitch Y, busy/error latest-value TX, timing and safety")
