"""Compile the actual UI task and inspect referee UART frames on a host."""
from pathlib import Path
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
import referee_protocol_test as protocol

ROOT = Path(__file__).resolve().parents[1]

HAL = protocol.HAL.replace(
    "int rx_starts=0;",
    "int rx_starts=0, attempts=0, fail_on_attempt=0;\n"
    "    std::vector<uint32_t> send_ticks;",
).replace(
    "if (fail_tx || handle.gState!=HAL_UART_STATE_READY) return false;",
    "if (++attempts==fail_on_attempt || fail_tx || "
    "handle.gState!=HAL_UART_STATE_READY) return false;\n"
    "        send_ticks.push_back(tick);",
)

TEST = protocol.TEST.replace(
    "void osDelay(uint32_t ms) { tick+=ms; uart.complete(); }",
    "void OnUiDelay(uint32_t ms);\n"
    "void osDelay(uint32_t ms) { tick+=ms; uart.complete(); OnUiDelay(ms); }",
).replace("int main(int argc,char **argv)", "int protocol_main(int argc,char **argv)") + r'''
#include "UI_task.hpp"
#include "host_remote.hpp"
PowerData_t PowerData={24.0f,0.5f,12.0f};
struct TaskFinished {};
std::string hud_case;
std::vector<std::vector<std::vector<uint8_t>>> cycles;
int max_cycles=2;
void OnUiDelay(uint32_t ms) {
    if (ms!=200) return;
    cycles.push_back(uart.frames);
    uart.frames.clear();
    const auto n=cycles.size();
    if (hud_case=="health") {
        if (n==1) ext_power_heat_data_0x0201.current_HP=0;
        if (n==2) ext_power_heat_data_0x0201.current_HP=125;
        if (n==3) ext_power_heat_data_0x0201.current_HP=700;
        if (n==4) {
            ext_power_heat_data_0x0201.current_HP=0;
            ext_power_heat_data_0x0201.maximum_HP=0;
        }
    }
    if (hud_case=="offline") RM_RefereeSystemDirFlag=(n==2);
    if (n>=static_cast<size_t>(max_cycles)) throw TaskFinished{};
}
struct Graphic {
    graphic_data_struct_t data;
    std::string text;
};
std::vector<Graphic> graphics(size_t cycle) {
    std::vector<Graphic> result;
    for (const auto &f:cycles.at(cycle)) {
        CHECK(f.size()>=30 && valid_frame(f,0x0301,f.size()-9),"valid HUD frame length and CRC");
        if (f.size()<30) continue;
        const auto command=f[7]+(f[8]<<8);
        const int count=command==0x0101 ? 1 : command==0x0102 ? 2 :
                        command==0x0103 ? 5 : command==0x0104 ? 7 : command==0x0110 ? 1 : 0;
        CHECK(count>0,"HUD uses a supported UI command");
        for (int i=0;i<count;++i) {
            Graphic g={};std::memcpy(&g.data,f.data()+13+15*i,15);
            if (command==0x0110) g.text.assign(reinterpret_cast<const char*>(f.data()+28),g.data.end_angle);
            result.push_back(g);
        }
    }
    return result;
}
bool named(const Graphic &g,const char *name) { return std::memcmp(g.data.graphic_name,name,3)==0; }
const Graphic *find(const std::vector<Graphic> &items,const char *name) {
    for (const auto &g:items) if (named(g,name)) return &g;
    return nullptr;
}
int32_t number(const Graphic &g) {
    int32_t value=0;std::memcpy(&value,reinterpret_cast<const uint8_t*>(&g.data)+11,4);return value;
}
void check_initial(size_t cycle,int cx,int cy,int first_mark_y) {
    const auto items=graphics(cycle);
    for (const char *name:{"PW_","MX_","BF_","PWR","MAX","BUF","HP_","HPN","HPB","HPF",
                          "CRH","CRV","DT0","DT1","DT2","DL0","DL1","DL2"}) {
        const auto *g=find(items,name);
        CHECK(g!=nullptr,"initial HUD contains every power, health and aiming graphic");
        if (g) CHECK(g->data.operate_tpye==1,"initial HUD adds graphics");
    }
    for (size_t i=0;i<items.size();++i) for (size_t j=i+1;j<items.size();++j)
        CHECK(std::memcmp(items[i].data.graphic_name,items[j].data.graphic_name,3)!=0,"HUD graphic names are unique");
    const auto *hp=find(items,"HPN"),*power=find(items,"PWR"),*limit=find(items,"MAX"),*buffer=find(items,"BUF");
    if (hp) CHECK(number(*hp)==250,"health number comes from referee status");
    if (power) CHECK(number(*power)==12,"existing measured power is retained");
    if (limit) CHECK(number(*limit)==100,"existing power limit is retained");
    if (buffer) CHECK(number(*buffer)==40,"existing buffer energy is retained");
    const auto *horizontal=find(items,"CRH"),*vertical=find(items,"CRV"),*mark=find(items,"DT0"),*label=find(items,"DL0");
    if (horizontal) CHECK(horizontal->data.start_x==cx-24 && horizontal->data.end_x==cx+24 &&
                          horizontal->data.start_y==cy && horizontal->data.end_y==cy,"adjustable horizontal crosshair coordinates");
    if (vertical) CHECK(vertical->data.start_x==cx && vertical->data.end_x==cx &&
                        vertical->data.start_y==cy-24 && vertical->data.end_y==cy+24,"adjustable vertical crosshair coordinates");
    if (mark) CHECK(mark->data.start_x==cx-16 && mark->data.end_x==cx+16 &&
                    mark->data.start_y==first_mark_y && mark->data.end_y==first_mark_y,"adjustable distance mark coordinates");
    if (label) CHECK(label->text=="3m" && label->data.start_x==cx+28 && label->data.start_y==first_mark_y+10,
                    "distance label follows mark position");
}
int main(int argc,char **argv) {
    if (argc!=2) return 2;
    hud_case=argv[1];
    ext_power_heat_data_0x0201.robot_id=1;
    ext_power_heat_data_0x0201.current_HP=250;
    ext_power_heat_data_0x0201.maximum_HP=500;
    ext_power_heat_data_0x0201.chassis_power_limit=100;
    ext_power_heat_data_0x0202.chassis_power_buffer=40;
    RM_RefereeSystemDirFlag=false;
    if (hud_case=="health") max_cycles=5;
    if (hud_case=="retry") { uart.fail_on_attempt=10;max_cycles=3; }
    if (hud_case=="offline") { RM_RefereeSystemDirFlag=true;max_cycles=4; }
    if (hud_case=="timing") tick=0xfffffe00U;
    try { ui_task(nullptr); } catch (TaskFinished &) {}
    CHECK(cycles.size()==static_cast<size_t>(max_cycles),"actual UI task completes requested cycles");
    if (hud_case=="hud" || hud_case=="timing" || hud_case=="aim_config") {
        const int cx=hud_case=="aim_config" ? 1000 : 960,cy=hud_case=="aim_config" ? 600 : 540;
        check_initial(0,cx,cy,hud_case=="aim_config" ? 535 : 500);
        const auto next=graphics(1);
        CHECK(next.size()==5,"subsequent cycle updates three power and two health graphics only");
        CHECK(!find(next,"CRH") && !find(next,"DT0") && !find(next,"HPB"),"fixed guides and health border are not sent every cycle");
        for (const char *name:{"PWR","MAX","BUF","HPN","HPF"}) {
            const auto *g=find(next,name);CHECK(g!=nullptr,"dynamic graphics remain present");
            if (g) CHECK(g->data.operate_tpye==2,"subsequent dynamic graphics use revise");
        }
    } else if (hud_case=="health") {
        const int values[]={250,0,125,700,0},lengths[]={146,0,73,292,0},operations[]={1,3,1,2,3};
        for (size_t i=0;i<cycles.size();++i) {
            const auto items=graphics(i);const auto *hp=find(items,"HPN"),*fill=find(items,"HPF");
            CHECK(hp && fill,"each health update sends the number and bar operation");
            if (hp) CHECK(number(*hp)==values[i],"health number tracks current HP");
            if (fill) {
                CHECK(fill->data.operate_tpye==static_cast<uint32_t>(operations[i]),"empty bar is deleted and nonzero HP re-adds it");
                if (lengths[i]) CHECK(static_cast<int>(fill->data.end_x)-static_cast<int>(fill->data.start_x)==lengths[i],
                                     "bar is proportional and saturates at maximum HP");
            }
        }
    } else if (hud_case=="retry") {
        const auto first=graphics(0),retry=graphics(1),last=graphics(2);
        CHECK(find(first,"CRH") && !find(first,"DL0"),"failure interrupts aim label initialization");
        for (const char *name:{"CRH","CRV","DT0","DT1","DT2","DL0","DL1","DL2"})
            CHECK(find(retry,name)!=nullptr,"failed static initialization is fully retried");
        CHECK(!find(last,"CRH"),"successful retry finishes static initialization");
    } else if (hud_case=="offline") {
        CHECK(cycles[0].empty() && cycles[2].empty(),"offline task sends no UI packets");
        check_initial(1,960,540,500);check_initial(3,960,540,500);
    } else return 2;
    for (size_t i=1;i<uart.send_ticks.size();++i)
        CHECK(static_cast<uint32_t>(uart.send_ticks[i]-uart.send_ticks[i-1])>=40,"all HUD packets share minimum 40 ms spacing across tick wrap");
    return failures ? 1 : 0;
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix="chassis_ui_") as directory:
        work = Path(directory)
        for name in ("RM_RefereeSystem.h", "RM_RefereeSystem.cpp", "RM_RefereeSystemCRC.h",
                     "RM_RefereeSystemCRC.cpp", "RMRefereeSystemUI.hpp", "RMRefereeSystemUI.cpp"):
            source = (protocol.REFEREE / name).read_text(encoding="utf-8-sig")
            source = source.replace('"../../HAL/UART/uart_hal.hpp"', '"host_hal.hpp"')
            source = source.replace('"main.h"', '"host_hal.hpp"')
            source = source.replace('"cmsis_os.h"', '"host_hal.hpp"')
            (work / name).write_text(source, encoding="utf-8")
        for name in ("state_watch.hpp", "state_watch.cpp"):
            source = (ROOT / "user/core/BSP/Common/StateWatch" / name).read_text(encoding="utf-8")
            (work / name).write_text(source.replace('"main.h"', '"host_hal.hpp"'), encoding="utf-8")
        for name in ("PowerControl.hpp", "PowerControlBase.hpp"):
            source = (ROOT / "user/core/Alg/PowerControl" / name).read_text(encoding="utf-8")
            source = source.replace('"../User/core/Alg/PowerControl/PowerControlBase.hpp"', '"PowerControlBase.hpp"')
            (work / name).write_text(source, encoding="utf-8")
        for name in ("UI_task.hpp", "UI_task.cpp"):
            source = (ROOT / "RtosTask" / name).read_text(encoding="utf-8-sig")
            source = source.replace('"remote_task.hpp"', '"host_remote.hpp"')
            source = source.replace('"cmsis_os.h"', '"host_hal.hpp"')
            source = source.replace('"../user/core/APP/Referee/', '"')
            (work / name).write_text(source, encoding="utf-8")
        (work / "host_remote.hpp").write_text(
            '#pragma once\nstruct PowerData_t { float voltage,current,power; };\nextern PowerData_t PowerData;\n',
            encoding="utf-8",
        )
        (work / "host_hal.hpp").write_text(HAL, encoding="utf-8")
        (work / "test.cpp").write_text(TEST, encoding="utf-8")
        command = ["g++", "-std=c++17", "-I.", "RM_RefereeSystem.cpp", "RM_RefereeSystemCRC.cpp",
                   "RMRefereeSystemUI.cpp", "UI_task.cpp", "state_watch.cpp", "test.cpp", "-o", "test.exe"]
        subprocess.run(command, cwd=work, check=True)
        failed = []
        cases = ("hud", "health", "retry", "offline", "timing")
        for case in cases:
            result = subprocess.run([str(work / "test.exe"), case], cwd=work, capture_output=True, text=True)
            print(f"{case}: {'PASS' if result.returncode == 0 else 'FAIL'}", flush=True)
            if result.returncode:
                failed.append(case)
                print(result.stdout + result.stderr, flush=True)
        assert not failed, f"Failed HUD cases: {', '.join(failed)}"
        header = work / "RMRefereeSystemUI.hpp"
        source = header.read_text(encoding="utf-8")
        for before, after in (("CENTER_X = 960", "CENTER_X = 1000"),
                              ("CENTER_Y = 540", "CENTER_Y = 600"),
                              ('{"3m", 0, -40, 16}', '{"3m", 0, -65, 16}')):
            assert before in source, f"Missing documented calibration parameter: {before}"
            source = source.replace(before, after)
        header.write_text(source, encoding="utf-8")
        subprocess.run(command, cwd=work, check=True)
        subprocess.run([str(work / "test.exe"), "aim_config"], cwd=work, check=True)
        print(f"chassis UI: {len(cases) + 1} cases passed (including changed calibration parameters)")


if __name__ == "__main__":
    main()
