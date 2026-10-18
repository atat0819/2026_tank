"""用真实两端编解码源码验证请求/回复、拆粘帧、CRC 和控制失联处理。"""
from pathlib import Path
import shutil
import subprocess
import tempfile

chassis = Path(__file__).resolve().parents[1]
gimbal = chassis.parent / "tank_gimbal"
task_source = (chassis / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
task_header = (chassis / "RtosTask/can_send_task.hpp").read_text(encoding="utf-8")
control_type = task_header[task_header.index("struct ControlInputSnapshot"):task_header.index("ControlInputSnapshot GetControlInputSnapshot")]
publication = task_source[task_source.index("void Chassis485_ApplyData"):task_source.index("StairCommand stair_command;")]
control = r'''
#include "chassis/fsm/control_input_selector.hpp"
#include "task.h"
struct GimbalState { float yaw_offset_deg, vx, vy, s1, s2, imu_yaw_deg, imu_gyro_z_deg_s; };
GimbalState gimbalChassis_communicate{};
uint16_t gimbal_keyboard=0;
uint32_t gimbal_yaw_last_tick=0, gimbal_speed_last_tick=0, gimbal_switch_last_tick=0, gimbal_keyboard_last_tick=0;
bool gimbal_yaw_received=false, gimbal_speed_received=false, gimbal_switch_received=false, gimbal_keyboard_received=false;
bool yaw_offset_updated=false;
uint8_t gimbalChassisSpeedUpdated=0;
bool local_remote_received=false;
uint32_t local_remote_last_tick=0;
uint16_t local_remote_keyboard=0;
int16_t local_remote_mouse_x=0;
struct { uint8_t s1=1,s2=1; float vx=0,vy=0,wz=0; } remoteData;
''' + control_type + publication
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
extern unsigned board, attempts[2];
extern bool accept[2];
extern UART_HandleTypeDef handles[2];
extern uint8_t *active[2];
extern uint16_t lengths[2];
struct Device {
    unsigned id;
    UART_HandleTypeDef *get_handle() { return &handles[id]; }
    bool transmit_dma(const Data &d) {
        ++attempts[id];
        if (!accept[id]) return false;
        active[id]=d.buffer; lengths[id]=d.size;
        handles[id].gState=HAL_UART_STATE_BUSY_TX;
        return true;
    }
};
struct Bus {
    Device devices[2]={{0},{1}};
    Device &get_uart3() { return devices[board]; }
};
inline Bus &get_uart_bus_instance() { static Bus bus; return bus; }
}
'''
motor = r'''
#pragma once
#include <cassert>
#include <cstdint>
namespace BSP::Motor::DM {
template<uint8_t N> struct J4340 {
    bool online=true; float angle=37.613f;
    bool isConnected(int slot,int ring) { assert(slot==1 && ring==1); return online; }
    float getAngleDeg(int slot) { assert(slot==1); return angle; }
};
}
'''
checks = r'''
#include <cassert>
#include <cmath>
#include <cstring>
#include "chassis/communication/boards_communication_uart_485.hpp"
#include "chassis/communication/gimbal_refree_uart_485.hpp"
#include "gimbal/communication_between_boards/boards_communication_uart_485.hpp"
#include "gimbal/communication_between_boards/refree_receive_uart_485.hpp"
#include "chassis/user/core/HAL/UART/uart_hal.hpp"
#include "gimbal/user/core/BSP/Motor/DM/DmMotor.hpp"
uint32_t now=100;
namespace HAL::UART {
unsigned board=0, attempts[2]={}; bool accept[2]={true,true};
UART_HandleTypeDef handles[2]; uint8_t *active[2]={}; uint16_t lengths[2]={};
}
BSP::Motor::DM::J4340<2> gimbal_motor;
unsigned applied=0;
Chassis485Data snapshot;
void Chassis485_ApplyData(const Chassis485Data &d) { snapshot=d; ++applied; }
void next_request() {
    now+=5; HAL::UART::board=0;
    HAL::UART::handles[0].gState=HAL_UART_STATE_READY;
}
int main() {
    using namespace HAL::UART;
    board=1;
    Referee485_Update(0x1234,0x4567,0x89ab,1);
    board=0;
    assert(RS485_SendBoardData(0.5f,-0.25f,3,3,-45,12,0x1234)==HAL_OK);
    assert(lengths[0]==28);
    uint8_t request[28]; memcpy(request,active[0],28);
    handles[0].gState=HAL_UART_STATE_READY;
    // 即使上一帧已经发完，未收到回复且未到 2ms 超时仍不能发送。
    assert(RS485_SendBoardData(0,0,0,0,0,0,0)==HAL_BUSY);
    assert(attempts[0]==1);
    board=1;
    uint8_t noise[]={1,2,0x78,0x22,0};
    Chassis485_Parse(noise,sizeof(noise));
    Chassis485_Parse(request,9); assert(applied==0 && attempts[1]==0);
    Chassis485_Parse(request+9,19);
    assert(applied==1 && attempts[1]==1 && lengths[1]==12);
    assert(std::fabs(snapshot.yaw_offset_deg-17.25f)<0.00001f);
    assert(snapshot.vx==0.5f && snapshot.vy==-0.25f && snapshot.keyboard==0x1234);
    assert(snapshot.imu_yaw_deg==315 && snapshot.imu_gyro_z_deg_s==12);
    auto input=GetControlInputSnapshot(now);
    assert(input.source==ControlInputSource::GIMBAL && input.keyboard==0x1234);
    assert(gimbal_switch_last_tick==now && gimbal_yaw_last_tick==now);
    assert(GetControlInputSnapshot(now+100).source==ControlInputSource::NONE);
    local_remote_received=true; local_remote_last_tick=now;
    assert(GetControlInputSnapshot(now).source==ControlInputSource::LOCAL);
    local_remote_received=false;
    uint8_t reply[12]; memcpy(reply,active[1],12);
    assert(reply[0]==0x91 && reply[1]==0x78 && reply[2]==0x12 && reply[3]==0x34);
    auto &ref=Communication::GimbalRefree485::instance();
    board=0;
    ref.parse485(reply,4); assert(!ref.isDataReceived());
    ref.parse485(reply+4,8);
    assert(ref.isDataReceived() && ref.getCoolingValue()==0x1234);
    assert(ref.getHeatLimit()==0x4567 && ref.getHeat42mm()==0x89ab && ref.isRefereeOnline());
    assert(ref.getDataAgeMs(now)==0);
    // 收到有效回复即可在下一次调用发送，不再等待固定 5ms。
    assert(RS485_SendBoardData(0,0,1,1,0,0,0)==HAL_OK);
    handles[0].gState=HAL_UART_STATE_READY;
    now+=1;
    assert(RS485_SendBoardData(0,0,1,1,0,0,0)==HAL_BUSY);
    now+=1;
    assert(RS485_SendBoardData(0,0,1,1,0,0,0)==HAL_OK);
    // 错误 CRC 不刷新输入、不回复；下一帧仍能重新同步。
    board=1; handles[1].gState=HAL_UART_STATE_READY;
    request[8]^=1; Chassis485_Parse(request,28);
    assert(applied==1 && attempts[1]==1);
    request[8]^=1;
    uint8_t pair[56]; memcpy(pair,request,28); memcpy(pair+28,request,28);
    Chassis485_Parse(pair,56); assert(applied==3);
    // 回复 DMA 忙时不得覆盖旧缓冲区。
    uint8_t saved[12]; memcpy(saved,active[1],12);
    Referee485_Update(1,2,3,0); Chassis485_Parse(request,28);
    assert(memcmp(saved,active[1],12)==0);
    // 损坏的回复不刷新裁判心跳，含噪声后的完整帧能恢复。
    board=0; const uint32_t age_before_bad=ref.getDataAgeMs(now);
    now+=10; reply[5]^=1; ref.parse485(reply,12);
    assert(ref.getDataAgeMs(now)==age_before_bad+10);
    reply[5]^=1; ref.parse485(noise,sizeof(noise)); ref.parse485(reply,12);
    assert(ref.getDataAgeMs(now)==0);
    // 无回复时按周期发送最新快照，不积压旧命令。
    next_request(); gimbal_motor.online=false;
    assert(RS485_SendBoardData(0,0,0,0,0,0,0)==HAL_OK);
    board=1; handles[1].gState=HAL_UART_STATE_READY;
    Chassis485_Parse(active[0],28);
    assert(snapshot.yaw_offset_deg==0 && snapshot.s1==0 && snapshot.s2==0);
    assert(snapshot.vx==0 && snapshot.vy==0 && snapshot.keyboard==0);
    assert(GetControlInputSnapshot(now).source==ControlInputSource::NONE);
    assert(!gimbal_switch_received && !gimbal_keyboard_received);
    board=0; ref.parse485(active[1],12);
    assert(!ref.isRefereeOnline() && ref.getCoolingValue()==1);
    // 非法开关/NaN 即使带合法 CRC 也不能进入控制数据。
    next_request(); assert(RS485_SendBoardData(NAN,0,3,3,0,0,0)==HAL_OK);
    board=1; unsigned old=applied; Chassis485_Parse(active[0],28); assert(applied==old);
    next_request(); assert(RS485_SendBoardData(0,0,4,3,0,0,0)==HAL_OK);
    board=1; Chassis485_Parse(active[0],28); assert(applied==old);
    // HAL 启动失败不能把链路锁在等待状态。
    next_request(); accept[0]=false;
    assert(RS485_SendBoardData(0,0,1,1,0,0,0)==HAL_ERROR);
    accept[0]=true; assert(RS485_SendBoardData(0,0,1,1,0,0,0)==HAL_OK);
    // tick 溢出后 2ms 回复超时仍按无符号差值判断。
    now=0xfffffffeU; handles[0].gState=HAL_UART_STATE_READY;
    assert(RS485_SendBoardData(0,0,1,1,0,0,0)==HAL_OK);
    now=0xffffffffU; handles[0].gState=HAL_UART_STATE_READY;
    assert(RS485_SendBoardData(0,0,1,1,0,0,0)==HAL_BUSY);
    now=0; assert(RS485_SendBoardData(0,0,1,1,0,0,0)==HAL_OK);
}
'''
with tempfile.TemporaryDirectory(prefix="rs485_link_") as directory:
    work = Path(directory)
    for name, root, files in (
        ("chassis", chassis, ["communication/boards_communication_uart_485.cpp", "communication/boards_communication_uart_485.hpp", "communication/gimbal_refree_uart_485.cpp", "communication/gimbal_refree_uart_485.hpp"]),
        ("gimbal", gimbal, ["communication_between_boards/boards_communication_uart_485.cpp", "communication_between_boards/boards_communication_uart_485.hpp", "communication_between_boards/refree_receive_uart_485.cpp", "communication_between_boards/refree_receive_uart_485.hpp"]),
    ):
        for file in files:
            target = work / name / file
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(root / file, target)
        for path, content in (("user/core/HAL/UART/uart_hal.hpp",uart), ("user/core/BSP/Motor/DM/DmMotor.hpp",motor)):
            target=work/name/path; target.parent.mkdir(parents=True,exist_ok=True); target.write_text(content,encoding="utf-8")
    (work/"chassis/fsm").mkdir()
    shutil.copyfile(chassis/"fsm/control_input_selector.hpp",work/"chassis/fsm/control_input_selector.hpp")
    (work/"stm32h7xx_hal.h").write_text(hal,encoding="utf-8")
    (work/"main.h").write_text('#include "stm32h7xx_hal.h"\n',encoding="utf-8")
    (work/"FreeRTOS.h").write_text('#pragma once\n',encoding="utf-8")
    (work/"task.h").write_text('#pragma once\n#define taskENTER_CRITICAL() ((void)0)\n#define taskEXIT_CRITICAL() ((void)0)\n',encoding="utf-8")
    test_source = checks.replace("unsigned applied=0;", control.replace("Chassis485_ApplyData(","ApplyActualData(") + "\nunsigned applied=0;")
    test_source = test_source.replace("snapshot=d; ++applied;", "ApplyActualData(d); snapshot=d; ++applied;")
    (work/"test.cpp").write_text(test_source,encoding="utf-8")
    subprocess.run(["g++","-std=c++17","-I",".","test.cpp",
        "chassis/communication/boards_communication_uart_485.cpp","chassis/communication/gimbal_refree_uart_485.cpp",
        "gimbal/communication_between_boards/boards_communication_uart_485.cpp",
        "gimbal/communication_between_boards/refree_receive_uart_485.cpp",
        "-o","test.exe"],cwd=work,check=True)
    subprocess.run([str(work/"test.exe")],check=True)
print("RS485 request/reply, split/merged frames, CRC, busy protection and offline safety passed")
