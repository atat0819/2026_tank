"""Run the actual referee/CRC/UI sources against 2027 wire frames and UART state."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
REFEREE = ROOT / "user/core/APP/Referee"

HAL = r'''
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <vector>
#include <stdexcept>
enum { HAL_UART_STATE_READY=0x20, HAL_UART_STATE_BUSY_TX=0x21,
       HAL_UART_STATE_BUSY_RX=0x22 };
struct UART_HandleTypeDef { uint32_t gState=HAL_UART_STATE_READY, RxState=HAL_UART_STATE_BUSY_RX; };
extern uint32_t tick;
extern int state_polls;
inline uint32_t HAL_GetTick() { return tick; }
inline uint32_t HAL_UART_GetState(UART_HandleTypeDef *h) {
    if (++state_polls > 100) throw std::runtime_error("RX busy blocked UI TX");
    return h->gState | h->RxState;
}
void osDelay(uint32_t ms);
namespace HAL::UART {
struct Data { uint8_t *buffer; uint16_t size; };
struct Device {
    UART_HandleTypeDef handle;
    std::vector<std::vector<uint8_t>> frames;
    uint8_t *pending=nullptr;
    uint16_t pending_size=0;
    bool fail_tx=false, stuck_tx=false;
    int rx_starts=0;
    UART_HandleTypeDef *get_handle() { return &handle; }
    bool transmit_dma(const Data &data) {
        if (fail_tx || handle.gState!=HAL_UART_STATE_READY) return false;
        handle.gState=HAL_UART_STATE_BUSY_TX; pending=data.buffer; pending_size=data.size;
        return true;
    }
    bool receive_dma_idle(const Data &) {
        if (handle.RxState!=HAL_UART_STATE_READY) return false;
        ++rx_starts; handle.RxState=HAL_UART_STATE_BUSY_RX; return true;
    }
    void complete() {
        if (stuck_tx) return;
        if (pending) frames.emplace_back(pending,pending+pending_size);
        pending=nullptr; pending_size=0; handle.gState=HAL_UART_STATE_READY;
    }
};
struct Bus { Device device; Device &get_uart1() { return device; } };
inline Bus &get_uart_bus_instance() { static Bus bus; return bus; }
}
'''

TEST = r'''
#include "RM_RefereeSystem.h"
#include "RMRefereeSystemUI.hpp"
#include "PowerControl.hpp"
#include <cstdio>
#include <string>
using namespace RM_RefereeSystem;
using namespace RM_RefereeSystemCRC;
namespace RM_RefereeSystem { void RM_daohangtx(map_robot_data_t); }
uint32_t tick=0;
int state_polls=0, failures=0;
uint8_t referee_buffer[512]={};
HAL::UART::Data uart1_rx_data{referee_buffer,sizeof(referee_buffer)};
auto &uart=HAL::UART::get_uart_bus_instance().device;
void osDelay(uint32_t ms) { tick+=ms; uart.complete(); }
#define CHECK(c,msg) do { if (!(c)) { std::printf("FAIL: %s\n",msg); ++failures; } } while(0)
std::vector<uint8_t> frame(uint16_t id, const void *payload, size_t size) {
    std::vector<uint8_t> f(size+9); f[0]=0xA5; f[1]=size; f[2]=size>>8;
    f[5]=id; f[6]=id>>8;
    if (size) std::memcpy(f.data()+7,payload,size);
    Append_CRC8_Check_Sum(f.data(),5); Append_CRC16_Check_Sum(f.data(),f.size());
    return f;
}
void feed(const std::vector<uint8_t> &f) { RM_RefereeSystemParse(f.data(),f.size()); }
template<class T> void feed(uint16_t id,const T &p) { feed(frame(id,&p,sizeof(p))); }
bool valid_frame(std::vector<uint8_t> f,uint16_t id,size_t len) {
    return f.size()==len+9 && f[1]+(f[2]<<8)==len &&
        f[5]+(f[6]<<8)==id && Verify_CRC8_Check_Sum(f.data(),5) &&
        Verify_CRC16_Check_Sum(f.data(),f.size());
}
int main(int argc,char **argv) {
    if (argc!=2) return 2;
    std::string test=argv[1];
    ext_game_robot_status_t s={}; s.robot_id=1; s.chassis_power_limit=100;
    s.bullet_speed_limit=15; s.power_management_chassis_output=1;
    ext_power_heat_data_t p={};p.chassis_power_buffer=40;p.shooter_id1_42mm_cooling_heat=123;
    if (test=="layout") {
        CHECK(sizeof(s)==17,"0201 size"); CHECK(sizeof(p)==14,"0202 size");
        CHECK(sizeof(graphic_data_struct_t)==15,"graphic size");
        CHECK(sizeof(map_command_t)==12,"0303 must be 12 bytes");
        CHECK(sizeof(map_robot_data_t)==48,"0305 must be 48 bytes");
        CHECK(offsetof(map_command_t,cmd_keyboard)==8 && offsetof(map_command_t,target_robot_id)==9 &&
              offsetof(map_command_t,cmd_source)==10,"0303 field offsets");
        CHECK(offsetof(ext_game_robot_status_t,chassis_power_limit)==10,"power offset");
        CHECK(offsetof(ext_power_heat_data_t,shooter_id1_42mm_cooling_heat)==12,"heat offset");
        uint8_t m[12]={};float x=1.5f,y=2.5f;
        std::memcpy(m,&x,4);std::memcpy(m+4,&y,4);m[8]='W';m[9]=101;m[10]=0x65;m[11]=1;
        feed(frame(0x0303,m,12));
        const auto *raw=reinterpret_cast<const uint8_t*>(&map_command_0x0303);
        CHECK(std::memcmp(raw,m,12)==0,"0303 full payload decodes at correct offsets");
        CHECK(map_command_0x0303.cmd_keyboard=='W' && map_command_0x0303.target_robot_id==101 &&
              map_command_0x0303.cmd_source==357,"0303 key, target and source values");
    } else if (test=="length") {
        tick=10;feed(0x0201,s);feed(0x0202,p);
        s.robot_id=3;p.shooter_id1_42mm_cooling_heat=999;
        feed(frame(0x0201,&s,12));feed(frame(0x0202,&p,12));
        CHECK(ext_power_heat_data_0x0201.robot_id==1,"short 0201 must not publish");
        CHECK(ext_power_heat_data_0x0202.shooter_id1_42mm_cooling_heat==123,"short 0202 must not zero heat");
        uint8_t long_s[18]={};std::memcpy(long_s,&s,sizeof s);feed(frame(0x0201,long_s,18));
        CHECK(ext_power_heat_data_0x0201.robot_id==1,"long 0201 must not publish");
        uint8_t long_p[15]={};std::memcpy(long_p,&p,sizeof p);feed(frame(0x0202,long_p,15));
        CHECK(ext_power_heat_data_0x0202.shooter_id1_42mm_cooling_heat==123,"long 0202 must not publish");
        auto old=map_command_0x0303;uint8_t m[15]={1};
        feed(frame(0x0303,m,11));feed(frame(0x0303,m,15));
        CHECK(std::memcmp(&old,&map_command_0x0303,sizeof old)==0,"wrong 0303 length rejected");
    } else if (test=="stream") {
        auto a=frame(0x0201,&s,sizeof s),b=frame(0x0202,&p,sizeof p);
        for (size_t n=0;n<5;++n) RM_RefereeSystemParse(&a[n]);
        RM_RefereeSystemParse(a.data()+5,a.size()-5);
        CHECK(ext_power_heat_data_0x0201.chassis_power_limit==100,"fragmented frame");
        a.insert(a.end(),b.begin(),b.end());feed(a);
        CHECK(ext_power_heat_data_0x0202.shooter_id1_42mm_cooling_heat==123,"concatenated frames");
        s.robot_id=3;a=frame(0x0201,&s,sizeof s);a.back()^=1;feed(a);
        CHECK(ext_power_heat_data_0x0201.robot_id==1,"CRC16 corrupt frame rejected");
        a=frame(0x0201,&s,sizeof s);a[4]^=1;feed(a);feed(0x0201,s);
        CHECK(ext_power_heat_data_0x0201.robot_id==3,"recover after bad header CRC");
    } else if (test=="freshness") {
        CHECK(RM_RefereeSystemDir(),"startup has no valid referee data");
        tick=20;feed(0x0201,s);CHECK(RM_RefereeSystemDir(),"0201 alone does not validate heat");
        feed(0x0202,p);CHECK(!RM_RefereeSystemDir(),"both fresh messages online");
        ALG::PowerControl::PowerControlStrategy strategy(1600);
        strategy.Update(false,!RM_RefereeSystemDir(),s.chassis_power_limit,p.chassis_power_buffer,0);
        CHECK(strategy.GetInputLimit()==100,"fresh data supplies actual power strategy");
        tick=1019;CHECK(!RM_RefereeSystemDir(),"fresh through 999ms");
        tick=1020;feed(0x0201,s);CHECK(RM_RefereeSystemDir(),"0201 cannot refresh old 0202");
        feed(0x0202,p);CHECK(!RM_RefereeSystemDir(),"valid data recovers online");
        tick=2020;uint8_t noise[]={1,2,3};RM_RefereeSystemParse(noise,3);
        auto corrupt=frame(0x0202,&p,sizeof p);corrupt.back()^=1;feed(corrupt);
        feed(frame(0x0201,&s,12));uint8_t map[12]={};feed(frame(0x0303,map,12));
        CHECK(RM_RefereeSystemDir(),"noise, corrupt, wrong length and map do not refresh control data");
        strategy.Update(false,!RM_RefereeSystemDir(),s.chassis_power_limit,p.chassis_power_buffer,0);
        CHECK(strategy.GetInputLimit()==100,"actual power strategy keeps last valid limit offline");
        CHECK(ext_power_heat_data_0x0201.chassis_power_limit==100,"offline preserves last power for fallback");
        CHECK(uart.rx_starts==0,"stale data must not restart already active RX");
        feed(0x0202,p);CHECK(RM_RefereeSystemDir(),"0202 cannot refresh old 0201");
        feed(0x0201,s);CHECK(!RM_RefereeSystemDir(),"both messages recover after timeout");
        tick+=1000;uart.handle.RxState=HAL_UART_STATE_READY;RM_RefereeSystemDir();
        CHECK(uart.rx_starts==1,"stopped RX restarts on timeout");
    } else if (test=="wrap") {
        tick=0xfffffff0U;feed(0x0201,s);feed(0x0202,p);
        tick=0x3d7;CHECK(!RM_RefereeSystemDir(),"999ms across tick rollover");
        tick=0x3d8;CHECK(RM_RefereeSystemDir(),"1000ms across tick rollover");
    } else if (test=="ui" || test=="ui_retry" || test=="tx_stuck") {
        ext_power_heat_data_0x0201=s;
        if (test=="tx_stuck") {
            uart.handle.gState=HAL_UART_STATE_BUSY_TX;uart.stuck_tx=true;
            std::memset(tx_buf,0x5a,sizeof(tx_buf));uart.pending=tx_buf;uart.pending_size=sizeof(tx_buf);
            RMRefereeSystemUI::UpdatePowerStatus(12,100,40);
            CHECK(tick>=20 && tick<=25,"bounded TX wait");
            for (auto b:tx_buf) CHECK(b==0x5a,"busy DMA data unchanged");
        } else {
            if (test=="ui_retry") { uart.fail_tx=true;RMRefereeSystemUI::UpdatePowerStatus(12,100,40);uart.fail_tx=false; }
            RMRefereeSystemUI::UpdatePowerStatus(12,100,40);uart.complete();
            CHECK(uart.frames.size()==5,"initial UI sends three labels and two graphic frames with RX busy");
            for (size_t i=0;i<uart.frames.size();++i) {
                CHECK(valid_frame(uart.frames[i],0x0301,i<3 ? 51 : i==3 ? 36 : 21),"UI length and CRC");
                CHECK(uart.frames[i][9]==1 && uart.frames[i][11]==1 && uart.frames[i][12]==1,"UI sender and client IDs");
            }
            uart.frames.clear();RMRefereeSystemUI::UpdatePowerStatus(-12,100,40);uart.complete();
            CHECK(uart.frames.size()==2,"subsequent UI modifies numeric graphics only");
            if (uart.frames.size()==2) {
                int32_t packed=0;std::memcpy(&packed,uart.frames[0].data()+24,4);
                CHECK(packed==-12,"signed integer wire packing");
                CHECK((uart.frames[0][16]&7)==2,"UI revise operation");
            }
        }
    } else if (test=="invalid_ui") {
        ext_power_heat_data_0x0201=s;
        graphic_data_struct_t g[8]={};
        RM_RefereeSystemSendDataN(g,0);RM_RefereeSystemSendDataN(g,3);RM_RefereeSystemSendDataN(g,8);
        ext_client_custom_character_t text={};text.grapic_data_struct.end_angle=31;RM_RefereeSystemSendStr(text);
        uart.complete();CHECK(uart.frames.empty(),"invalid UI sizes rejected before packing");
    } else if (test=="ui_max") {
        ext_power_heat_data_0x0201=s;graphic_data_struct_t g[7]={};
        for (int n:{1,2,5,7}) { CHECK(RM_RefereeSystemSendDataN(g,n),"allowed graphic count");uart.complete(); }
        for (size_t i=0;i<uart.frames.size();++i) {
            const size_t count[]={1,2,5,7};
            CHECK(valid_frame(uart.frames[i],0x0301,6+15*count[i]),"all UI graphic frame sizes and CRCs");
        }
        auto f=RMRefereeSystemUI::MakeFloat("FLT",1,-1.234f,1,2);
        int32_t packed=0;std::memcpy(&packed,reinterpret_cast<const uint8_t*>(&f)+11,4);
        CHECK(packed==-1234,"float uses signed integer scaled by 1000");
        char name[]="TXT",long_text[41];std::memset(long_text,'a',40);long_text[40]=0;
        auto t=RM_RefereeSystemSetStr(name,1,long_text,1,2);
        CHECK(t.grapic_data_struct.end_angle==30,"legacy string builder respects 30-byte payload");
    } else if (test=="map_tx") {
        map_robot_data_t m={};
        ext_power_heat_data_0x0201.robot_id=1;RM_daohangtx(m);uart.complete();
        CHECK(uart.frames.empty(),"chassis ID cannot send radar-only 0305");
        ext_power_heat_data_0x0201.robot_id=9;RM_daohangtx(m);uart.complete();
        CHECK(uart.frames.size()==1,"radar map frame sent");
        if (uart.frames.size()==1) CHECK(valid_frame(uart.frames[0],0x0305,48),"0305 complete 57-byte frame");
        m.ally_sentry_position_y=0x1234;m.opponent_heavyengineer_position_x=0x5678;
        ext_power_heat_data_0x0201.robot_id=109;RM_daohangtx(m);uart.complete();
        CHECK(uart.frames.size()==2,"blue radar map frame sent");
        if (uart.frames.size()==2) CHECK(uart.frames[1][7]==0x78 && uart.frames[1][8]==0x56 &&
            uart.frames[1][53]==0x34 && uart.frames[1][54]==0x12,"0305 first and last coordinate offsets");
    } else return 2;
    return failures ? 1 : 0;
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix="referee_2027_") as directory:
        work = Path(directory)
        for name in ("RM_RefereeSystem.h", "RM_RefereeSystem.cpp", "RM_RefereeSystemCRC.h",
                     "RM_RefereeSystemCRC.cpp", "RMRefereeSystemUI.hpp", "RMRefereeSystemUI.cpp"):
            source = (REFEREE / name).read_text(encoding="utf-8-sig")
            source = source.replace('"../../HAL/UART/uart_hal.hpp"', '"host_hal.hpp"')
            source = source.replace('"../../BSP/Common/StateWatch/state_watch.hpp"', '"state_watch.hpp"')
            source = source.replace('"main.h"', '"host_hal.hpp"').replace('"cmsis_os.h"', '"host_hal.hpp"')
            (work / name).write_text(source, encoding="utf-8")
        for name in ("state_watch.hpp", "state_watch.cpp"):
            source = (ROOT / "user/core/BSP/Common/StateWatch" / name).read_text(encoding="utf-8")
            (work / name).write_text(source.replace('"main.h"', '"host_hal.hpp"'), encoding="utf-8")
        for name in ("PowerControl.hpp", "PowerControlBase.hpp"):
            source = (ROOT / "user/core/Alg/PowerControl" / name).read_text(encoding="utf-8")
            source = source.replace('"../User/core/Alg/PowerControl/PowerControlBase.hpp"', '"PowerControlBase.hpp"')
            (work / name).write_text(source, encoding="utf-8")
        (work / "host_hal.hpp").write_text(HAL, encoding="utf-8")
        (work / "test.cpp").write_text(TEST, encoding="utf-8")
        subprocess.run(["g++", "-std=c++17", "-I.", "RM_RefereeSystem.cpp", "RM_RefereeSystemCRC.cpp",
                        "RMRefereeSystemUI.cpp", "state_watch.cpp", "test.cpp", "-o", "test.exe"], cwd=work, check=True)
        failed = []
        cases = ("layout", "length", "stream", "freshness", "wrap", "ui", "ui_retry", "tx_stuck", "invalid_ui", "ui_max", "map_tx")
        for case in cases:
            result = subprocess.run([str(work / "test.exe"), case], cwd=work, capture_output=True, text=True)
            print(f"{case}: {'PASS' if result.returncode == 0 else 'FAIL'}", flush=True)
            if result.returncode:
                failed.append(case)
                print(result.stdout + result.stderr, flush=True)
        assert not failed, f"Failed cases: {', '.join(failed)}"
        print(f"referee protocol: {len(cases)} cases passed")


if __name__ == "__main__":
    main()
