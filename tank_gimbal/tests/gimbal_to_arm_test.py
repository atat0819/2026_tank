"""Verify UART7 remote-control frame encoding and DMA buffer ownership."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "communication_between_boards/gimbal_to_arm.cpp").read_text(encoding="utf-8")
header = (root / "communication_between_boards/gimbal_to_arm.hpp").read_text(encoding="utf-8")
task = (root / "RtosTask/remote_control_task.cpp").read_text(encoding="utf-8")
assert "class GimbalToArm" in header, "UART7 sender must be encapsulated in a class"
callback = task[task.index("uart5.register_rx_callback"):task.index("// 启动接收")]
assert "arm_remote_received = true" in callback, "UART5 must only mark a received frame"
assert callback.index("data.size == 18") < callback.index("remoteController.parseData") < callback.index("arm_remote_received = true")
assert "gimbal_to_arm." not in callback, "UART5 callback must not send on UART7"
service = task[task.index("for (;;)"):]
assert service.index("taskENTER_CRITICAL()") < service.index("remoteController.get_keyboard()") < service.index("taskEXIT_CRITICAL()")
assert service.index("taskEXIT_CRITICAL()") < service.index("gimbal_to_arm.SendRemoteData(")
assert "ServiceOffline" not in service, "online and offline must use the same send function"
assert "remoteController.isConnected()" in service
assert "gimbal_to_arm.cpp" in (root / "MDK-ARM/tank_gimbal.uvprojx").read_text(encoding="utf-8")

hal = r'''
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
struct Bus { Device device; Device &get_uart7() { return device; } };
inline Bus &get_uart_bus_instance() { static Bus bus; return bus; }
}
'''
remote = r'''
#pragma once
#include <cstdint>
namespace BSP::REMOTE_CONTROL {
struct RemoteController {
    uint8_t s1=1, s2=3;
    uint16_t keyboard=0xa55a;
    float get_left_x() const {return -1.0f;} float get_left_y() const {return 0.25f;}
    float get_right_x() const {return -0.5f;} float get_right_y() const {return 0.75f;}
    float get_scroll_() const {return 1.0f;}
    int16_t get_mouseX() const {return -1234;} int16_t get_mouseY() const {return 2345;}
    int16_t get_mouseZ() const {return -7;} bool get_mouseLeft() const {return true;}
    bool get_mouseRight() const {return false;} uint16_t get_keyboard() const {return keyboard;}
    uint8_t get_s1() const {return s1;} uint8_t get_s2() const {return s2;}
};
}
'''
checks = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include "communication_between_boards/gimbal_to_arm.hpp"
#include "user/core/BSP/RemoteControl/DT7.hpp"
#include "user/core/HAL/UART/uart_hal.hpp"
uint32_t now=0;
namespace HAL::UART { UART_HandleTypeDef handle; unsigned attempts=0; bool accept=true;
uint8_t *active=nullptr; uint16_t active_size=0; }
uint16_t crc(const uint8_t *p, unsigned n) {
    uint16_t value=0xffff;
    for (unsigned i=0;i<n;++i) {
        uint8_t r=0; for (unsigned b=0;b<8;++b) r|=((p[i]>>b)&1)<<(7-b);
        value^=uint16_t(r)<<8;
        for (unsigned b=0;b<8;++b) value=(value&0x8000)?(value<<1)^0x1021:value<<1;
    }
    uint16_t r=0; for (unsigned b=0;b<16;++b) r|=((value>>b)&1)<<(15-b); return r;
}
uint16_t count() { auto *p=HAL::UART::active; return p[6] | (uint16_t(p[7])<<8); }
void check_crc() { auto *p=HAL::UART::active; assert((p[10] | (uint16_t(p[11])<<8))==crc(p,10)); }
int main() {
    BSP::REMOTE_CONTROL::RemoteController rc;
    auto send = [&]() { HAL::UART::handle.gState=HAL_UART_STATE_READY; assert(gimbal_to_arm.SendRemoteData(rc.s1,rc.s2,rc.keyboard,true)==HAL_OK); };
    auto offline = [&]() { return gimbal_to_arm.SendRemoteData(rc.s1,rc.s2,rc.keyboard,false); };
    auto advance = [&](uint32_t duration) {
        while (duration) { uint32_t step=duration>10 ? 10 : duration; now+=step; duration-=step; send(); }
    };
    assert(offline()==HAL_OK);
    assert(HAL::UART::active_size==12);
    auto *p=HAL::UART::active;
    assert(p[0]==0x25 && p[1]==0x44 && p[9]==0);
    for (unsigned i=2;i<10;++i) assert(p[i]==0);
    check_crc();
    send(); p=HAL::UART::active;
    assert(p[2]==1 && p[3]==3 && p[4]==0x5a && p[5]==0xa5);
    assert(count()==0 && p[8]==0 && p[9]==1); check_crc();
    uint8_t snapshot[12]; std::memcpy(snapshot,p,12);
    assert(gimbal_to_arm.SendRemoteData(rc.s1,rc.s2,rc.keyboard,true)==HAL_BUSY && std::memcmp(snapshot,p,12)==0);
    HAL::UART::handle.gState=HAL_UART_STATE_READY; HAL::UART::accept=false;
    assert(gimbal_to_arm.SendRemoteData(rc.s1,rc.s2,rc.keyboard,true)==HAL_ERROR); HAL::UART::accept=true;

    // Only double-middle accepts F/G. Both edges must remain stable for 30 ms.
    rc.s1=3; rc.keyboard=0; send();
    rc.keyboard=1<<10; send(); advance(29); assert(p[8]==0);
    advance(1); assert(p[8]==1); advance(40); assert(p[8]==1);
    rc.keyboard=0; send(); advance(30); rc.keyboard=1<<10; send(); advance(30); assert(p[8]==0);
    rc.keyboard=0; send(); advance(30);
    rc.keyboard=1<<9; send(); advance(29); assert(count()==0);
    advance(1); uint32_t start=now; advance(40); assert(count()==0);
    rc.keyboard=0; send(); advance(30); rc.keyboard=1<<9; send(); advance(30);
    rc.keyboard=0; send(); advance(30); rc.keyboard=1<<9; send(); advance(30);
    rc.keyboard=0; send(); advance(30);
    advance(start+3990-now); assert(count()==0);
    // The result survives a busy DMA and is sent exactly once.
    now+=10; HAL::UART::handle.gState=HAL_UART_STATE_BUSY_TX;
    assert(gimbal_to_arm.SendRemoteData(rc.s1,rc.s2,rc.keyboard,true)==HAL_BUSY);
    HAL::UART::handle.gState=HAL_UART_STATE_READY; HAL::UART::accept=false;
    assert(gimbal_to_arm.SendRemoteData(rc.s1,rc.s2,rc.keyboard,true)==HAL_ERROR);
    HAL::UART::accept=true; send(); assert(count()==3); check_crc();
    send(); assert(count()==0);

    // Exiting keyboard mode cancels a live count and resets G.
    rc.keyboard=1<<10; send(); advance(30); assert(p[8]==1);
    rc.keyboard=1<<9; send(); advance(30);
    rc.s1=1; send(); assert(p[8]==0 && count()==0);
    rc.s1=3; rc.keyboard=0; send(); advance(30); advance(4010); assert(count()==0);
    // Loss resets state, sends zeroed offline frames and reconnects without stale state.
    rc.keyboard=1<<10; send(); advance(30); assert(p[8]==1);
    rc.keyboard=1<<9; send(); advance(30);
    now+=100; std::memcpy(snapshot,p,12);
    HAL::UART::handle.gState=HAL_UART_STATE_BUSY_TX;
    assert(offline()==HAL_BUSY && std::memcmp(snapshot,p,12)==0);
    HAL::UART::handle.gState=HAL_UART_STATE_READY; assert(offline()==HAL_OK);
    assert(p[9]==0 && p[8]==0 && count()==0);
    for (unsigned i=2;i<10;++i) assert(p[i]==0);
    check_crc(); unsigned old=HAL::UART::attempts;
    now+=2; HAL::UART::handle.gState=HAL_UART_STATE_READY; assert(offline()==HAL_OK); assert(HAL::UART::attempts==old+1);
    now+=2; HAL::UART::handle.gState=HAL_UART_STATE_READY; assert(offline()==HAL_OK); assert(HAL::UART::attempts==old+2);
    rc.keyboard=0; send(); assert(p[9]==1 && p[8]==0 && count()==0);
    advance(4010); assert(count()==0);
    // A key already held while entering double-middle is not a new press.
    rc.s1=1; rc.keyboard=1<<10; send(); advance(40);
    rc.s1=3; send(); advance(40); assert(p[8]==0);
    rc.keyboard=0; send(); advance(30); rc.keyboard=1<<10; send(); advance(30); assert(p[8]==1);
    // A short bounce does not start or increment an F window.
    rc.s1=1; rc.keyboard=0; send(); rc.s1=3; send();
    rc.keyboard=1<<9; send(); advance(20); rc.keyboard=0; send(); advance(30);
    rc.keyboard=1<<9; send(); advance(30); uint32_t second_start=now;
    rc.keyboard=0; send(); advance(30); advance(second_start+3990-now); assert(count()==0);
    advance(10); assert(count()==1); send(); assert(count()==0);
    // A timeout across tick rollover is detected even before the periodic task runs.
    now=UINT32_MAX-80; offline(); send(); rc.keyboard=1<<10; send(); advance(30); assert(p[8]==1);
    now+=100; offline(); rc.keyboard=0; send(); assert(p[8]==0 && p[9]==1);
}
'''
with tempfile.TemporaryDirectory(prefix="gimbal_arm_") as directory:
    temp = Path(directory)
    uart = uart.replace('#include "stm32h7xx_hal.h"', "")
    header = header.replace('#include "stm32h7xx_hal.h"', "")
    for include in ('#include "gimbal_to_arm.hpp"',
                    '#include "../user/core/BSP/RemoteControl/DT7.hpp"',
                    '#include "../user/core/HAL/UART/uart_hal.hpp"'):
        source = source.replace(include, "")
    for include in ('#include "communication_between_boards/gimbal_to_arm.hpp"',
                    '#include "user/core/BSP/RemoteControl/DT7.hpp"',
                    '#include "user/core/HAL/UART/uart_hal.hpp"'):
        checks = checks.replace(include, "")
    cpp = temp / "test.cpp"
    cpp.write_text("\n".join((hal, uart, remote, header, source, checks)).replace("#pragma once", ""), encoding="utf-8")
    exe = temp / "test.exe"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    str(cpp), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
    # Execute the actual task-loop snapshot and dispatch code with an interrupt
    # simulated immediately after the snapshot leaves the critical section.
    loop = service[service.index("        bool remote_online;"):service.index("        if (!remote_online)")]
    task_test = r'''
#include <cassert>
#include <cstdint>
namespace BSP::REMOTE_CONTROL { struct RemoteController { enum { DOWN=2 }; }; }
uint32_t g_watchdog_rearm_required=0;
struct Remote {
    bool connected=true; uint8_t s1=1, s2=3; uint16_t keyboard=0x1234;
    bool isConnected() { return connected; }
    uint8_t get_s1() { return s1; } uint8_t get_s2() { return s2; }
    float get_left_y() { return 0; } float get_left_x() { return 0; }
    float get_right_y() { return 0; } float get_right_x() { return 0; }
    uint16_t get_keyboard() { return keyboard; }
} remoteController;
bool arm_remote_received=false, inject_new_frame=false, in_critical=false;
struct Arm {
    int online_calls=0, offline_calls=0; uint8_t s1=0,s2=0; uint16_t keyboard=0;
    void SendRemoteData(uint8_t a,uint8_t b,uint16_t k,bool online) {
        assert(!in_critical); if (online) ++online_calls; else ++offline_calls;
        s1=a; s2=b; keyboard=k;
    }
} gimbal_to_arm;
void enter() { assert(!in_critical); in_critical=true; }
void leave() {
    assert(in_critical); in_critical=false;
    if (inject_new_frame) { remoteController.s1=2; remoteController.s2=1;
        remoteController.keyboard=0x5678; inject_new_frame=false; }
}
#define taskENTER_CRITICAL() enter()
#define taskEXIT_CRITICAL() leave()
void run_once() {
TASK_LOOP
}
int main() {
    run_once(); assert(gimbal_to_arm.offline_calls==1 && gimbal_to_arm.online_calls==0);
    arm_remote_received=true; inject_new_frame=true; run_once();
    assert(gimbal_to_arm.online_calls==1 && gimbal_to_arm.s1==1 && gimbal_to_arm.s2==3 && gimbal_to_arm.keyboard==0x1234);
    run_once(); assert(gimbal_to_arm.online_calls==2 && gimbal_to_arm.s1==2 && gimbal_to_arm.s2==1 && gimbal_to_arm.keyboard==0x5678);
    remoteController.connected=false; run_once();
    assert(gimbal_to_arm.offline_calls==2 && gimbal_to_arm.online_calls==2);
}
'''.replace("TASK_LOOP", loop)
    cpp.write_text(task_test, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    str(cpp), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print("PASS: 12-byte payload, task-context TX, debounce, F/G, busy/error, mode, offline and recovery")
