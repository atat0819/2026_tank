"""Exercise the real chassis/gimbal RS485 code with fake UART and DBUS devices.

Run with CXX set to a working host C++ compiler, or use g++ from PATH.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
hal = r'''
#pragma once
#include <stdint.h>
enum { HAL_UART_STATE_READY, HAL_UART_STATE_BUSY_TX };
struct UART_HandleTypeDef { unsigned gState = HAL_UART_STATE_READY; };
extern uint32_t now;
inline uint32_t HAL_GetTick() { return now; }
'''
uart = r'''
#pragma once
#include "stm32h7xx_hal.h"
namespace HAL { namespace UART {
struct Data { uint8_t *buffer; uint16_t size; };
extern unsigned board, attempts[2];
extern bool accept[2];
extern UART_HandleTypeDef handles[2];
extern uint8_t *active[2];
extern uint16_t lengths[2];
struct Device {
    unsigned id;
    UART_HandleTypeDef *get_handle() { return &handles[id]; }
    bool transmit_dma(const Data &data) {
        ++attempts[id];
        if (!accept[id]) return false;
        active[id] = data.buffer;
        lengths[id] = data.size;
        handles[id].gState = HAL_UART_STATE_BUSY_TX;
        return true;
    }
};
struct Bus {
    Device devices[2] = {{0}, {1}};
    Device &get_uart3() { return devices[board]; }
};
inline Bus &get_uart_bus_instance() { static Bus bus; return bus; }
} }
'''
remote = r'''
#pragma once
#include <stdint.h>
#include <string.h>
struct RemoteStub {
    unsigned parsed = 0;
    int32_t mouse_x = 0;
    uint8_t raw[18] = {};
    void parseData(const uint8_t *data) {
        memcpy(raw, data, sizeof(raw));
        ++parsed;
        mouse_x += static_cast<int16_t>(data[6] | (data[7] << 8));
    }
    uint8_t get_s1() const { return (raw[5] >> 6) & 3U; }
    uint8_t get_s2() const { return (raw[5] >> 4) & 3U; }
};
extern RemoteStub remoteController;
'''
checks = r'''
#include <cassert>
#include <cmath>
#include <cstring>
#include "rs485_protocol.hpp"
#include "chassis/communication/boards_communication_uart_485.hpp"
#include "gimbal/communication_between_boards/boards_communication_uart_485.hpp"
#include "uart_stub.hpp"
#include "remote_stub.hpp"
#include "gimbal/communication_between_boards/input_dispatcher.hpp"
uint32_t now = 100;
RemoteStub remoteController;
namespace HAL { namespace UART {
unsigned board = 0, attempts[2] = {};
bool accept[2] = {true, true};
UART_HandleTypeDef handles[2];
uint8_t *active[2] = {};
uint16_t lengths[2] = {};
} }
unsigned yaw_updates = 0;
float yaw = 0;
void Chassis485_ApplyData(const Chassis485Data &data) { yaw=data.yaw_offset_deg; ++yaw_updates; }
void sendReady() {
    HAL::UART::board=0;
    HAL::UART::handles[0].gState=HAL_UART_STATE_READY;
    Chassis485_Send();
    assert(HAL::UART::lengths[0]==32);
}
int main() {
    using namespace HAL::UART;
    assert(BoardLink::crc16(reinterpret_cast<const uint8_t*>("123456789"),9)==0x6f91);
    uint8_t dbus[18] = {};
    const unsigned channels[4] = {364, 1684, 700, 1400};
    // Independently pack the four 11-bit DBUS channels for the input fixture.
    for (unsigned channel=0; channel<4; ++channel)
        for (unsigned bit=0; bit<11; ++bit)
            dbus[(channel*11+bit)/8] |= ((channels[channel]>>bit)&1U)<<((channel*11+bit)%8);
    dbus[5] |= (3U<<6)|(1U<<4);
    BoardLink::put16(dbus+6,static_cast<uint16_t>(-123));
    BoardLink::put16(dbus+8,32767);
    BoardLink::put16(dbus+10,static_cast<uint16_t>(-45));
    dbus[12]=1; dbus[13]=0;
    BoardLink::put16(dbus+14,0xa55a);
    BoardLink::put16(dbus+16,1684);
    assert(BoardLink::dbusValid(dbus));

    Chassis485_UpdateRemote(dbus);
    Chassis485_UpdateReferee(0x1234,0x4567,0x1111,1);
    // Input DMA may overwrite its buffer after the callback; the saved frame must survive.
    uint8_t expected[18]; memcpy(expected,dbus,18); memset(dbus,0,18);
    sendReady();
    uint8_t frame[32]; memcpy(frame,active[0],32);
    assert(frame[0]==0x91 && frame[1]==0x78 && memcmp(frame+2,expected,18)==0);
    BoardLink::ChassisData decoded{};
    assert(BoardLink::decodeChassis(frame,decoded));
    assert(decoded.remote_online && decoded.remote_sequence==1 && decoded.cooling==0x1234);
    assert(decoded.heat_limit==0x4567 && decoded.heat_42mm==0x1111 && decoded.referee_online);

    board=1; Gimbal485_UpdateYawFromISR(34);
    const uint8_t noise[]={1,0x91,2,0x78,0};
    Gimbal485_Parse(noise,sizeof(noise));
    Gimbal485_Parse(frame,11); assert(remoteController.parsed==0);
    Gimbal485_Parse(frame+11,21);
    assert(remoteController.parsed==1 && remoteController.mouse_x==-123);
    assert(memcmp(remoteController.raw,expected,18)==0 && Gimbal485_RemoteOnline());
    Gimbal485RefereeData referee; Gimbal485_GetRefereeData(referee);
    assert(referee.received && referee.cooling_value==0x1234 && referee.age_ms==0);
    uint8_t reply[8]; assert(lengths[1]==8); memcpy(reply,active[1],8);
    board=0; Chassis485_Parse(reply,3); assert(yaw_updates==0);
    Chassis485_Parse(reply+3,5); assert(yaw_updates==1 && fabsf(yaw-10)<0.001f);

    // Same DBUS sequence: update referee data and replies, but consume no mouse delta twice.
    decoded.cooling=200;
    BoardLink::encodeChassis(frame,decoded);
    board=1;
    uint8_t pair[64]; memcpy(pair,frame,32); memcpy(pair+32,frame,32);
    Gimbal485_Parse(pair,64);
    assert(remoteController.parsed==1 && remoteController.mouse_x==-123);
    Gimbal485_GetRefereeData(referee); assert(referee.cooling_value==200);

    // Identical input bytes from a NEW radio frame must be parsed once more.
    now+=7; board=0; Chassis485_UpdateRemote(expected); sendReady();
    memcpy(frame,active[0],32); board=1; Gimbal485_Parse(frame,32);
    assert(remoteController.parsed==2 && remoteController.mouse_x==-246);
    // CRC failure must not publish input; the next valid frame must resynchronize.
    now+=7; board=0; Chassis485_UpdateRemote(expected); sendReady();
    memcpy(frame,active[0],32); frame[8]^=1; board=1; Gimbal485_Parse(frame,32);
    assert(remoteController.parsed==2);
    frame[8]^=1; Gimbal485_Parse(frame,32); assert(remoteController.parsed==3);

    // Busy DMA cannot have its active transmission buffer overwritten.
    board=0; uint8_t saved[32]; memcpy(saved,active[0],32);
    now+=7; Chassis485_UpdateRemote(expected);
    const unsigned attempts_before=attempts[0]; Chassis485_Send();
    assert(attempts[0]==attempts_before && memcmp(saved,active[0],32)==0);
    handles[0].gState=HAL_UART_STATE_READY; accept[0]=false; Chassis485_Send();
    accept[0]=true; Chassis485_Send();
    memcpy(frame,active[0],32); board=1; Gimbal485_Parse(frame,32);
    assert(remoteController.parsed==4);

    // A valid CRC cannot turn invalid DBUS switches into a valid radio sample.
    assert(BoardLink::decodeChassis(frame,decoded));
    decoded.dbus[5]&=0x0f; BoardLink::encodeChassis(frame,decoded);
    now+=5; Gimbal485_Parse(frame,32); assert(remoteController.parsed==4);
    Gimbal485_GetRefereeData(referee); assert(referee.age_ms==5);
    memcpy(decoded.dbus,expected,18);
    // Explicit radio offline and recovery, even with the same sequence.
    decoded.remote_online=0; BoardLink::encodeChassis(frame,decoded); Gimbal485_Parse(frame,32);
    assert(!Gimbal485_RemoteOnline());
    decoded.remote_online=1; BoardLink::encodeChassis(frame,decoded); Gimbal485_Parse(frame,32);
    assert(Gimbal485_RemoteOnline() && remoteController.parsed==5);
    // A link timeout followed by recovery may accept a reused sequence.
    now+=100; assert(!Gimbal485_RemoteOnline()); Gimbal485_Parse(frame,32);
    assert(remoteController.parsed==6);

    // Sequence wrap must still recognize a new radio frame.
    decoded.remote_sequence=65535; BoardLink::encodeChassis(frame,decoded); Gimbal485_Parse(frame,32);
    decoded.remote_sequence=0; BoardLink::encodeChassis(frame,decoded); Gimbal485_Parse(frame,32);
    assert(remoteController.parsed==8);
    // The chassis must mark its cached radio frame offline after 100 ms.
    sendReady(); memcpy(frame,active[0],32); assert(BoardLink::decodeChassis(frame,decoded));
    assert(!decoded.remote_online);
    board=1; Gimbal485_Parse(frame,32); assert(!Gimbal485_RemoteOnline());

    // Invalid yaw feedback stays invalid; it must not publish an angle on the chassis.
    handles[1].gState=HAL_UART_STATE_READY; Gimbal485_Send(); memcpy(reply,active[1],8);
    board=0; Chassis485_Parse(reply,8); assert(yaw_updates==1);
    // Fresh radio data and the unsigned time checks also work across tick wrap.
    now=0xfffffff0U; Chassis485_UpdateRemote(expected); sendReady();
    now=0x20U; sendReady(); memcpy(frame,active[0],32);
    assert(BoardLink::decodeChassis(frame,decoded) && decoded.remote_online);
    board=1; Gimbal485_Parse(frame,32); assert(Gimbal485_RemoteOnline());

    // Two accepted radio frames before the next send must both reach the gimbal.
    now=1000;
    uint8_t pressed[18], released[18];
    memcpy(pressed,expected,18); memcpy(released,expected,18);
    BoardLink::put16(pressed+6,11); BoardLink::put16(pressed+14,1U<<8);
    BoardLink::put16(released+6,static_cast<uint16_t>(-7)); BoardLink::put16(released+14,0);
    Chassis485_UpdateRemote(pressed); Chassis485_UpdateRemote(released);
    const unsigned burst_before=remoteController.parsed;
    const int32_t mouse_before=remoteController.mouse_x;
    sendReady(); memcpy(frame,active[0],32);
    assert(memcmp(frame+2,pressed,18)==0);
    board=1; Gimbal485_Parse(frame,32);
    now+=2; sendReady(); memcpy(frame,active[0],32);
    assert(memcmp(frame+2,released,18)==0);
    board=1; Gimbal485_Parse(frame,32);
    assert(remoteController.parsed==burst_before+2);
    assert(remoteController.mouse_x==mouse_before+4);

    // Only frames routed to camera Yaw contribute X, at most once per sequence.
    now+=2;
    pressed[5] = (pressed[5] & 0x0fU) | 0xf0U;
    pressed[13] = 1;
    Chassis485_UpdateRemote(pressed);
    sendReady(); memcpy(frame,active[0],32);
    assert(BoardLink::decodeChassis(frame,decoded) && decoded.yaw_mouse_active);
    assert((frame[20]&0x03U)==0x03U);
    board=1; Gimbal485_Parse(frame,32); Gimbal485_Parse(frame,32);
    auto camera=Gimbal485_ConsumeYawMouseInput();
    assert(camera.online && remoteController.get_s1()==3 && remoteController.get_s2()==3);
    assert(camera.mouse_active && camera.mouse_delta_x==11);
    assert(Gimbal485_ConsumeYawMouseInput().mouse_delta_x==0);

    // A release clears pending X from a preceding pressed sample.
    now+=2; Chassis485_UpdateRemote(pressed); sendReady(); memcpy(frame,active[0],32);
    board=1; Gimbal485_Parse(frame,32);
    now+=2; released[5]=pressed[5];
    Chassis485_UpdateRemote(released); sendReady(); memcpy(frame,active[0],32);
    board=1; Gimbal485_Parse(frame,32);
    camera=Gimbal485_ConsumeYawMouseInput();
    assert(camera.online && remoteController.get_s1()==3 && remoteController.get_s2()==3);
    assert(!camera.mouse_active && camera.mouse_delta_x==0);
    const unsigned parsed_before_timeout=remoteController.parsed;

    // Repeated heartbeats with a frozen radio sequence cannot keep input online forever.
    for (unsigned i=0;i<5;++i) { now+=20; Gimbal485_Parse(frame,32); }
    assert(!Gimbal485_RemoteOnline() && remoteController.parsed==parsed_before_timeout);
    camera=Gimbal485_ConsumeYawMouseInput();
    assert(!camera.online && !camera.mouse_active && camera.mouse_delta_x==0);

    // A fresh radio sample must not cause queued commands older than 100 ms to replay.
    now=1200; Chassis485_UpdateRemote(pressed);
    now=1301; Chassis485_UpdateRemote(released);
    sendReady(); memcpy(frame,active[0],32);
    assert(memcmp(frame+2,released,18)==0);
    board=1; Gimbal485_Parse(frame,32); assert(Gimbal485_RemoteOnline());

    // Bounded queue overflow explicitly keeps the newest eight frames, in order.
    now=1400;
    uint8_t burst[18]; memcpy(burst,expected,18);
    for (unsigned i=1;i<=10;++i) { BoardLink::put16(burst+6,i); Chassis485_UpdateRemote(burst); }
    for (unsigned i=3;i<=10;++i) {
        sendReady(); memcpy(frame,active[0],32);
        assert(BoardLink::get16(frame+8)==i);
        board=1; Gimbal485_Parse(frame,32); now+=2;
    }

    // The actual key dispatcher clears latched R/V and mouse state on the offline input.
    now=2000; input_dispatcher.Update(3,3,0,false,false);
    now+=31; input_dispatcher.Update(3,3,(1U<<8)|(1U<<14),true,false);
    now+=31; input_dispatcher.Update(3,3,(1U<<8)|(1U<<14),true,false);
    assert(input_dispatcher.IsFrictionOn() && input_dispatcher.IsHeatLimitBypassed());
    assert(input_dispatcher.IsLeftButtonPressed());
    input_dispatcher.Update(0,0,0,false,false);
    assert(!input_dispatcher.IsFrictionOn() && !input_dispatcher.IsHeatLimitBypassed());
    assert(!input_dispatcher.IsLeftButtonPressed());

    assert(fabsf(BoardLink::yawOffsetDeg(24))<0.001f);
    assert(BoardLink::yawOffsetDeg(204)==-180.0f);

    // All switch combinations reach the control task as raw decoded switches.
    assert(BoardLink::decodeChassis(frame,decoded));
    board=1;
    for (uint8_t s1=1; s1<=3; ++s1) {
        for (uint8_t s2=1; s2<=3; ++s2) {
            ++now;
            ++decoded.remote_sequence;
            decoded.dbus[5]=(decoded.dbus[5]&0x0fU)|(s1<<6)|(s2<<4);
            BoardLink::encodeChassis(frame,decoded);
            Gimbal485_Parse(frame,32);
            camera=Gimbal485_ConsumeYawMouseInput();
            assert(camera.online && remoteController.get_s1()==s1 && remoteController.get_s2()==s2);
        }
    }
}
'''

with tempfile.TemporaryDirectory(prefix="raw_dbus_485_", ignore_cleanup_errors=True) as directory:
    work = Path(directory)
    def put(name, text):
        p=work/name; p.parent.mkdir(parents=True,exist_ok=True); p.write_text(text,encoding="utf-8")
    shutil.copyfile(root/"rs485_protocol.hpp",work/"rs485_protocol.hpp")
    for board, source, fixture in (
        ("chassis","tank_chassis","communication"),
        ("gimbal","tank_gimbal","communication_between_boards"),
    ):
        for ext in ("cpp","hpp"):
            target=work/board/fixture/f"boards_communication_uart_485.{ext}"
            target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(root/source/fixture/target.name,target)
        put(f"{board}/user/core/HAL/UART/uart_hal.hpp",'#include "uart_stub.hpp"\n')
    put("gimbal/RtosTask/remote_control_task.hpp",'#include "remote_stub.hpp"\n')
    for ext in ("cpp", "hpp"):
        shutil.copyfile(root/"tank_gimbal/communication_between_boards"/f"input_dispatcher.{ext}",
                        work/"gimbal/communication_between_boards"/f"input_dispatcher.{ext}")
    put("gimbal/user/core/BSP/RemoteControl/DT7.hpp", """#pragma once
namespace BSP { namespace REMOTE_CONTROL {
struct RemoteController { enum { MIDDLE=3, KEY_R=1U<<8, KEY_V=1U<<14 }; };
} }
""")
    put("main.h", '#include "stm32h7xx_hal.h"\n')
    put("stm32h7xx_hal.h",hal)
    put("uart_stub.hpp",uart)
    put("remote_stub.hpp",remote)
    put("FreeRTOS.h","#pragma once\n")
    put("task.h","#pragma once\n#define taskENTER_CRITICAL() ((void)0)\n#define taskEXIT_CRITICAL() ((void)0)\n")
    put("test.cpp",checks)
    compiler=os.environ.get("CXX","g++")
    command=[compiler,"-std=c++11","-I.","test.cpp",
             "chassis/communication/boards_communication_uart_485.cpp",
             "gimbal/communication_between_boards/boards_communication_uart_485.cpp",
             "gimbal/communication_between_boards/input_dispatcher.cpp",
             "-o","test.exe"]
    prefix=os.environ.get("CXX_TOOL_PREFIX")
    if prefix:
        command.insert(1, "-B" + prefix + os.sep)
    if os.name == "nt":
        import ctypes
        ctypes.windll.kernel32.SetErrorMode(0x8003)
        command.insert(1, "-static")
    subprocess.run(command,cwd=work,check=True,timeout=45)
    subprocess.run([str(work/"test.exe")],check=True,timeout=10)
print("PASS: raw DBUS, duplicate/stale inputs, CRC/framing, busy/burst/overflow/expiry queue, offline key reset, sequence/time wrap, yaw reply")
