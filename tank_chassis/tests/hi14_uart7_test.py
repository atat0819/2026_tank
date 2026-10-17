"""Exercise the real HI14 parser and UART7 startup/RX/error fragments on the host."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "RtosTask/remote_task.cpp").read_text(encoding="utf-8")

def branch(function):
    start = source.index(function)
    start = source.index("else if (huart->Instance == UART7)", start)
    end = source.index("else if (huart->Instance == USART1)", start)
    return source[start:end].replace("else if", "if", 1)

startup = source[source.index("    // ---- 5. UART7"):source.index("    // ---- 6. USART1")]
assert "uart10.register_rx_callback" not in startup, "HI14 still owns UART10 RX"

HOST = r'''
#include "HI14.hpp"
#include <cassert>
#include <cmath>
#include <cstring>
#include <functional>
struct DMA_HandleTypeDef { bool busy=false; } dma;
struct UART_HandleTypeDef {
    int Instance=7, RxState=0, Lock=0, gState=77;
    DMA_HandleTypeDef *hdmarx=&dma;
} huart7;
DMA_HandleTypeDef &hdma_uart7_rx=dma;
enum { UART7=7, HAL_OK=0, HAL_UART_STATE_READY=0, HAL_UNLOCKED=0,
       UART_IT_RXNE=1, DMA_IT_HT=1, HAL_UART_RXEVENT_HT=1 };
int event_type=0, starts=0, aborts=0, ht_disabled=0, parse_calls=0;
int publications=0, invalidations=0;
uint32_t HAL_GetTick() { return 123U; }
void PublishHi14ImuFromISR(const IMU_Float_t &data, uint32_t tick, bool valid) {
    assert(tick==123U);
    if (valid) { ++publications; assert(data.pitch==ImuFloat.pitch); }
    else { ++invalidations; }
}
int HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *) { return event_type; }
int HAL_UART_AbortReceive(UART_HandleTypeDef *) { ++aborts; dma.busy=false; return HAL_OK; }
#define __HAL_UART_DISABLE_IT(h,f) ((void)0)
#define __HAL_UART_CLEAR_OREFLAG(h) ((void)0)
#define __HAL_UART_CLEAR_NEFLAG(h) ((void)0)
#define __HAL_UART_CLEAR_FEFLAG(h) ((void)0)
#define __HAL_UART_CLEAR_PEFLAG(h) ((void)0)
#define __HAL_DMA_DISABLE_IT(h,f) (++ht_disabled)
uint8_t imu_rx_buffer[64], power_rx_buffer[12];
struct { uint8_t bytes[12]; } PowerData;
namespace HAL::UART {
struct Data { uint8_t *buffer; uint16_t size; };
struct Device {
    std::function<void(const Data &)> callback;
    UART_HandleTypeDef *get_handle() { return &huart7; }
    void register_rx_callback(std::function<void(const Data &)> fn) { callback=fn; }
    bool receive_dma_idle(const Data &data) {
        assert(data.buffer==imu_rx_buffer && data.size==64);
        if (dma.busy) return false;
        ++starts; dma.busy=true; return true;
    }
    void trigger_rx_callbacks(const Data &data) {
        // Parsing must finish before DMA can overwrite the same buffer.
        assert(!dma.busy); ++parse_calls; callback(data);
    }
};
struct Bus { Device device; Device &get_uart7() { return device; } } bus;
Bus &get_uart_bus_instance() { return bus; }
}
HAL::UART::Data uart7_rx_data;
'''

MAIN = r'''
void feed(const uint8_t *data, uint16_t size) {
    assert(size<=64); std::memcpy(imu_rx_buffer,data,size);
    dma.busy=false; rx(&huart7,size);
}
int main() {
    auto &uart7=HAL::UART::get_uart_bus_instance().get_uart7();
    startup(uart7);
    assert(starts==1 && ht_disabled==1 && !ImuDataReady);
    uint8_t frame[54]={0x5A,0xA5,48,0};
    auto put16=[&](int pos,int16_t value) { std::memcpy(frame+6+pos,&value,2); };
    auto put32=[&](int pos,int32_t value) { std::memcpy(frame+6+pos,&value,4); };
    auto seal=[&]() {
        uint16_t crc=0;
        for(int j=0;j<54;++j) {
            if(j==4 || j==5) continue;
            crc^=static_cast<uint16_t>(frame[j])<<8;
            for(int k=0;k<8;++k) crc=(crc&0x8000) ? (crc<<1)^0x1021 : crc<<1;
        }
        frame[4]=crc&0xff; frame[5]=crc>>8;
    };
    frame[6]=0x92;
    put32(28,-180000); put32(32,90000); put32(36,-179999);
    put16(10,-1000); put16(12,2000); put16(14,-3000);
    put16(40,10000); put16(42,-1000); put16(44,2000); put16(46,-3000);
    seal();
    // Half-transfer must neither parse nor restart normal DMA.
    event_type=HAL_UART_RXEVENT_HT; rx(&huart7,32);
    assert(parse_calls==0 && starts==1 && !ImuDataReady);
    event_type=0;
    feed(frame,17); assert(!ImuDataReady);
    feed(frame+17,37); assert(ImuDataReady);
    assert(publications==1);
    assert(ImuData.Roll==-180000 && ImuData.Pitch==90000 && ImuData.Yaw==-179999);
    assert(std::fabs(hi14.GetPitchAngleDeg()-90.0f)<0.001f);
    assert(std::fabs(hi14.GetRollAngleDeg()+180.0f)<0.001f);
    assert(std::fabs(hi14.GetYawAngleDeg()+179.999f)<0.001f);
    assert(std::fabs(hi14.GetPitchRateDps()+57.29578f)<0.001f);
    assert(std::fabs(hi14.GetRollRateDps()-114.59156f)<0.001f);
    assert(std::fabs(hi14.GetYawRateDps()+171.88734f)<0.001f);
    assert(ImuFloat.qw==1.0f && std::fabs(ImuFloat.qx+0.1f)<0.001f);
    // Two contiguous frames crossing DMA chunk boundaries.
    uint8_t joined[108]; std::memcpy(joined,frame,54); std::memcpy(joined+54,frame,54);
    feed(joined,64); feed(joined+64,44);
    assert(publications==3);
    auto before=ImuData.Pitch;
    frame[6]=0; put32(32,-45000); seal(); feed(frame,54);
    assert(ImuData.Pitch==before && hImu.ErrCount==1);
    frame[6]=0x92; seal(); frame[4]^=1; feed(frame,54);
    assert(ImuData.Pitch==before && hImu.ErrCount==2);
    assert(publications==3);
    seal(); feed(frame,54); assert(ImuData.Pitch==-45000);
    assert(std::fabs(hi14.GetPitchAngleDeg()+45.0f)<0.001f);
    // A transport error interrupts a partial frame and releases RX DMA.
    feed(frame,20); error(&huart7);
    assert(aborts==1 && dma.busy && huart7.gState==77);
    assert(!ImuDataReady && hImu.Step==0);
    assert(invalidations==1);
    feed(frame,54); assert(ImuDataReady && ImuData.Pitch==-45000);
    // The raw integer 4096 is 4.096 degrees, not one full revolution.
    put32(28,0); put32(32,4096); put32(36,180000); seal(); feed(frame,54);
    assert(hi14.GetRollAngleDeg()==0.0f);
    assert(std::fabs(hi14.GetPitchAngleDeg()-4.096f)<0.001f);
    assert(std::fabs(hi14.GetYawAngleDeg()-180.0f)<0.001f);
    // A malformed length cannot overrun the fixed HI92 buffer or prevent recovery.
    const uint8_t bad_length[]={0x5A,0xA5,0xff,0xff};
    const auto errors=hImu.ErrCount;
    feed(bad_length,4); assert(hImu.ErrCount==errors+1);
    feed(frame,54); assert(ImuData.Pitch==4096);
}
'''

with tempfile.TemporaryDirectory(prefix="hi14-uart7-") as directory:
    work = Path(directory)
    (work / "HI14.hpp").write_text((ROOT / "user/core/BSP/Hi14/HI14.hpp").read_text(encoding="utf-8"), encoding="utf-8")
    parser = (ROOT / "user/core/BSP/Hi14/HI14.cpp").read_text(encoding="utf-8")
    (work / "FreeRTOS.h").write_text("#pragma once\nusing QueueHandle_t=void*;\n", encoding="utf-8")
    (work / "queue.h").write_text('#include "FreeRTOS.h"\n', encoding="utf-8")
    (work / "HI14.cpp").write_text(parser, encoding="utf-8")
    test = HOST + "\nvoid rx(UART_HandleTypeDef *huart,uint16_t Size) {\n" + branch('extern "C" void HAL_UARTEx_RxEventCallback') + "\n}\n"
    test += "void error(UART_HandleTypeDef *huart) {\n" + branch('extern "C" void HAL_UART_ErrorCallback') + "\n}\n"
    test += "void startup(HAL::UART::Device &uart7) {\n" + startup + "\n}\n" + MAIN
    (work / "test.cpp").write_text(test, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-I", ".", "HI14.cpp", "test.cpp", "-o", "test.exe"], cwd=work, check=True)
    subprocess.run([str(work / "test.exe")], check=True)
print("PASS: official HI92 framing/CRC, degree/dps getters, split/continuous frames and UART7 recovery")
