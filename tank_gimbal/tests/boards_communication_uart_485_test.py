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
    float stick=-0.5f;
    int16_t mouse=321;
    float get_right_x() { return stick; }
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
    assert(RS485_SendBoardData(-0.5f,321,0.25f,-0.5f,3,1,0x1234)==HAL_OK);
    assert(active_size==22 && attempts==1);
    assert(active[0]==0x78 && active[1]==0x91);
    assert(read_float(2)==-0.5f && read_int16(6)==321);
    assert(active[2]==0 && active[3]==0 && active[4]==0 && active[5]==0xbf);
    assert(active[6]==0x41 && active[7]==0x01);
    assert(read_float(8)==0.25f && read_float(12)==-0.5f);
    assert(active[16]==3 && active[17]==1);
    assert(active[18]==0x34 && active[19]==0x12);
    uint16_t crc=expected_crc(active,20);
    assert(active[20]==(crc&255) && active[21]==(crc>>8));
    uint8_t saved[22]; memcpy(saved,active,22);
    assert(RS485_SendBoardData(1.0f,-32768,0,0,0,0,0)==HAL_BUSY);
    assert(attempts==1 && memcmp(saved,active,22)==0);
    handle.gState=HAL_UART_STATE_READY; now+=5;
    assert(RS485_SendBoardData(0,0,0,0,0,0,0)==HAL_OK);
    for(unsigned i=2;i<20;++i) assert(active[i]==0);
    crc=expected_crc(active,20);
    assert(active[20]==(crc&255) && active[21]==(crc>>8));
    handle.gState=HAL_UART_STATE_READY; now+=5; accept=false;
    assert(RS485_SendBoardData(0,0,0,0,0,0,0)==HAL_ERROR);
    accept=true;
    assert(RS485_SendBoardData(1.0f,-32768,1,2,1,2,1)==HAL_OK);
    assert(read_float(2)==1.0f && read_int16(6)==-32768);

    handle.gState=HAL_UART_STATE_READY; now+=5;
    task_snapshot(false);
    assert(read_float(2)==-0.5f && read_int16(6)==0);
    assert(read_float(8)==0.75f && read_float(12)==-0.25f);
    assert(active[18]==0 && active[19]==0);
    input_dispatcher.source=InputSource::KeyMouse;
    remoteController.receive_mouse(10,7);
    handle.gState=HAL_UART_STATE_READY; now+=5;
    task_snapshot(false);
    assert(read_float(2)==0 && read_int16(6)==10 && observed_pitch_delta_y==7);
    assert(read_float(8)==0 && read_float(12)==0);
    assert(active[16]==3 && active[17]==3 && active[18]==0x21 && active[19]==0x43);
    // Repeat the same latest DT7 X state; Pitch Y remains a one-shot increment.
    for (int i=0;i<5;++i) {
        handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
        assert(read_int16(6)==10 && observed_pitch_delta_y==0);
    }
    remoteController.receive_mouse(10,7);
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
    assert(read_int16(6)==10 && observed_pitch_delta_y==7);
    remoteController.receive_mouse(0);
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
    assert(read_int16(6)==0 && observed_pitch_delta_y==0);
    for (int16_t value : {-32768,32767}) {
        remoteController.receive_mouse(value);
        handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
        assert(read_int16(6)==value);
        handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
        assert(read_int16(6)==value);
    }
    // DMA busy preserves in-flight bytes. Later TX uses -20, not the sum 100-20.
    memcpy(saved,active,22);
    remoteController.receive_mouse(100,4); task_snapshot(false);
    assert(observed_pitch_delta_y==4);
    remoteController.receive_mouse(-20,5); task_snapshot(false);
    assert(observed_pitch_delta_y==5 && memcmp(saved,active,22)==0);
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
    assert(read_int16(6)==-20 && observed_pitch_delta_y==0);
    // Original 2ms reply gate and reply-immediate retry are unchanged.
    const unsigned previous_attempts=attempts;
    handle.gState=HAL_UART_STATE_READY; ++now;
    remoteController.receive_mouse(7); task_snapshot(false);
    assert(attempts==previous_attempts && read_int16(6)==-20);
    ++now; task_snapshot(false);
    assert(attempts==previous_attempts+1 && read_int16(6)==7);
    handle.gState=HAL_UART_STATE_READY; RS485_ReplyReceived(); task_snapshot(false);
    assert(attempts==previous_attempts+2 && active_size==22 && read_int16(6)==7);
    // Failed TX does not queue an old mouse state.
    handle.gState=HAL_UART_STATE_READY; now+=5; accept=false;
    remoteController.receive_mouse(-30); task_snapshot(false);
    remoteController.receive_mouse(45); accept=true; task_snapshot(false);
    assert(read_int16(6)==45);
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
    assert(read_int16(6)==45);
    // Zero updates during busy TX supersede the previous nonzero state.
    remoteController.receive_mouse(0); task_snapshot(false);
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
    assert(read_int16(6)==0);
    remoteController.receive_mouse(40);
    input_dispatcher.source=InputSource::Remote;
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
    assert(read_int16(6)==0);
    input_dispatcher.source=InputSource::Other;
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
    assert(read_int16(6)==0);
    input_dispatcher.source=InputSource::KeyMouse;
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(true);
    assert(read_float(2)==0 && read_int16(6)==0 && active[16]==0 && active[17]==0);
    assert(read_float(8)==0 && read_float(12)==0 && active[18]==0 && active[19]==0);
    remoteController.receive_mouse(0);
    handle.gState=HAL_UART_STATE_READY; now+=5; task_snapshot(false);
    assert(read_int16(6)==0);

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
print("PASS: 22-byte RS485 frame, float right X, no IMU, latest mouse X, one-shot Pitch Y, busy/error TX, timing and safety")
