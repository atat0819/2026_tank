"""编译实际 RS485 驱动，验证帧内容、CRC、发送忙时缓冲区不被覆盖。"""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
task = (root / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
normal_call = task[task.index("const bool chassis_remote_mode"):task.index(" // tick 用于分频发送")]
offline_call = task[task.index("            RS485_SendBoardData("):task.index("            ResetGimbalControlState();", task.index("            RS485_SendBoardData("))]
task_cases = r'''
enum class InputSource { Remote, KeyMouse, Other };
struct { float chassis_vx=0.75f, chassis_vy=-0.25f; uint8_t s1=3, s2=3; } RemoteData;
struct {
    InputSource source=InputSource::Remote;
    InputSource GetSource() { return source; }
    uint16_t GetKeyboardMask() { return 0x4321; }
} input_dispatcher;
struct {
    bool online=true;
    bool isConnected() { return online; }
    float GetAngle(int axis) { assert(axis==2); return -45.0f; }
    float GetGyro(int axis) { assert(axis==2); return 12.0f; }
} imu;
void task_snapshot(bool offline) {
    if (offline) {
''' + offline_call + r'''
        return;
    }
''' + normal_call + r'''
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
motor = r'''
#pragma once
#include <cassert>
#include <cstdint>
namespace BSP::Motor::DM {
template<uint8_t N> struct J4340 {
    bool online=true;
    float angle=37.613f;
    bool isConnected(int slot,int ring) { assert(slot==1 && ring==1); return online; }
    float getAngleDeg(int slot) { assert(slot==1); return angle; }
};
}
'''
checks = r'''
#include <cassert>
#include <cmath>
#include <cstring>
#include "communication_between_boards/boards_communication_uart_485.hpp"
#include "user/core/HAL/UART/uart_hal.hpp"
#include "user/core/BSP/Motor/DM/DmMotor.hpp"
uint32_t now=100;
namespace HAL::UART {
UART_HandleTypeDef handle;
unsigned attempts=0;
bool accept=true;
uint8_t *active=nullptr;
uint16_t active_size=0;
}
BSP::Motor::DM::J4340<2> gimbal_motor;
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
int main() {
    using namespace HAL::UART;
    assert(RS485_SendBoardData(0.25f,-0.5f,3,1,-30.0f,-12.5f,0x1234)==HAL_OK);
    assert(active_size==28 && attempts==1);
    assert(active[0]==0x78 && active[1]==0x91);
    assert(std::fabs(read_float(2)-17.25f)<0.00001f && read_float(6)==0.25f && read_float(10)==-0.5f);
    assert(active[14]==3 && active[15]==1);
    assert(read_float(16)==330.0f && read_float(20)==-12.5f);
    assert(active[24]==0x34 && active[25]==0x12);
    uint16_t crc=expected_crc(active,26);
    assert(active[26]==(crc&255) && active[27]==(crc>>8));
    uint8_t saved[28]; memcpy(saved,active,28);
    gimbal_motor.online=false; // 保留旧编码器角，失联帧必须发送零偏移。
    assert(RS485_SendBoardData(0,0,0,0,0,0,0)==HAL_BUSY);
    assert(attempts==1 && memcmp(saved,active,28)==0);
    handle.gState=HAL_UART_STATE_READY; now+=5;
    assert(RS485_SendBoardData(0,0,0,0,720.0f,0,0)==HAL_OK);
    for(unsigned i=2;i<26;++i) assert(active[i]==0);
    crc=expected_crc(active,26);
    assert(active[26]==(crc&255) && active[27]==(crc>>8));
    handle.gState=HAL_UART_STATE_READY; now+=5; accept=false;
    assert(RS485_SendBoardData(0,0,0,0,0,0,0)==HAL_ERROR);
    accept=true;
    assert(RS485_SendBoardData(1,2,1,2,361.5f,7,1)==HAL_OK);
    assert(read_float(16)==1.5f && read_float(20)==7);
    handle.gState=HAL_UART_STATE_READY; now+=5;
    task_snapshot(false);
    assert(read_float(6)==0.75f && read_float(10)==-0.25f);
    assert(active[24]==0 && active[25]==0);
    assert(read_float(16)==315.0f && read_float(20)==12.0f);
    input_dispatcher.source=InputSource::KeyMouse;
    handle.gState=HAL_UART_STATE_READY; now+=5;
    task_snapshot(false);
    assert(read_float(6)==0 && read_float(10)==0);
    assert(active[14]==3 && active[15]==3 && active[24]==0x21 && active[25]==0x43);
    handle.gState=HAL_UART_STATE_READY; now+=5;
    task_snapshot(true);
    assert(read_float(6)==0 && read_float(10)==0);
    assert(active[14]==0 && active[15]==0 && active[24]==0 && active[25]==0);
    imu.online=false; handle.gState=HAL_UART_STATE_READY; now+=5;
    task_snapshot(false);
    assert(read_float(16)==0 && read_float(20)==0);
    gimbal_motor.online=true;
    gimbal_motor.angle=-170.0f;
    handle.gState=HAL_UART_STATE_READY; now+=5;
    task_snapshot(false);
    assert(std::fabs(read_float(2)-169.637f)<0.0001f);
    gimbal_motor.angle=220.0f;
    handle.gState=HAL_UART_STATE_READY; now+=5;
    task_snapshot(false);
    assert(std::fabs(read_float(2)+160.363f)<0.0001f);
}
'''
with tempfile.TemporaryDirectory(prefix="boards485_") as directory:
    work = Path(directory)
    (work / "communication_between_boards").mkdir()
    for name in ("boards_communication_uart_485.cpp", "boards_communication_uart_485.hpp"):
        shutil.copyfile(root / "communication_between_boards" / name, work / "communication_between_boards" / name)
    (work / "user/core/HAL/UART").mkdir(parents=True)
    (work / "user/core/HAL/UART/uart_hal.hpp").write_text(uart, encoding="utf-8")
    (work / "user/core/BSP/Motor/DM").mkdir(parents=True)
    (work / "user/core/BSP/Motor/DM/DmMotor.hpp").write_text(motor, encoding="utf-8")
    (work / "stm32h7xx_hal.h").write_text(stub, encoding="utf-8")
    (work / "main.h").write_text('#include "stm32h7xx_hal.h"\n', encoding="utf-8")
    (work / "test.cpp").write_text(checks.replace("int main() {", task_cases + "\nint main() {"), encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-I", ".", "test.cpp",
                    "communication_between_boards/boards_communication_uart_485.cpp",
                    "-o", "test.exe"], cwd=work, check=True)
    subprocess.run([str(work / "test.exe")], check=True)
print("28-byte RS485 frame, CRC, busy-buffer protection, input modes and offline safety passed")
