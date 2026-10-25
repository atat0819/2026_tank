"""用 HAL 替身编译实际 UART 封装和接收回调，检查两路 RS485 接入。"""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "RtosTask/remote_control_task.cpp").read_text(encoding="utf-8")
callbacks = source[source.index('extern "C" void HAL_UART_RxCpltCallback'):source.index('extern "C" void HAL_UART_ErrorCallback')]
stub = r'''
#pragma once
#include <cstdint>
#include <cstddef>
enum HAL_StatusTypeDef { HAL_OK, HAL_BUSY };
struct DMA_HandleTypeDef {};
struct UART_HandleTypeDef {
    uint8_t *pRxBuffPtr = nullptr;
    uint16_t RxXferSize = 0;
    DMA_HandleTypeDef *hdmarx = nullptr;
};
extern UART_HandleTypeDef huart1, huart2, huart3, huart5, huart7, huart10;
extern UART_HandleTypeDef *last_tx, *last_rx;
extern unsigned starts, ht_disables;
extern HAL_StatusTypeDef tx_result;
inline HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *, uint16_t) { last_tx=h; return tx_result; }
inline HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *, uint16_t) { last_tx=h; return tx_result; }
inline HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *, uint16_t, unsigned) { last_tx=h; return tx_result; }
inline HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t n) { last_rx=h; h->pRxBuffPtr=p; h->RxXferSize=n; return HAL_OK; }
inline HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *h, uint8_t *p, uint16_t n) { return HAL_UART_Receive_IT(h,p,n); }
inline HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *p, uint16_t n) { ++starts; return HAL_UART_Receive_IT(h,p,n); }
inline HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *h, uint8_t *p, uint16_t n, unsigned) { return HAL_UART_Receive_IT(h,p,n); }
enum { HAL_UART_RXEVENT_HT, HAL_UART_RXEVENT_TC, HAL_UART_RXEVENT_IDLE };
extern int rx_event;
inline int HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *) { return rx_event; }
#define RESET 0
#define UART_FLAG_ORE 1
#define DMA_IT_HT 1
#define __HAL_UART_GET_FLAG(h,f) 0
#define __HAL_UART_CLEAR_OREFLAG(h) ((void)0)
#define __HAL_DMA_DISABLE_IT(h,f) (++ht_disables)
'''
checks = r'''
#include <cassert>
#include "UART/uart_hal.hpp"
UART_HandleTypeDef huart1, huart2, huart3, huart5, huart7, huart10;
UART_HandleTypeDef *last_tx=nullptr, *last_rx=nullptr;
unsigned starts=0, ht_disables=0;
HAL_StatusTypeDef tx_result=HAL_OK;
int rx_event=HAL_UART_RXEVENT_IDLE;
DMA_HandleTypeDef hdma_uart5_rx, dma2, dma3;
uint8_t receivedata[18];
'''
main = r'''
int main() {
    using namespace HAL::UART;
    auto &bus=get_uart_bus_instance();
    auto &u2=bus.get_uart2(); auto &u3=bus.get_uart3();
    assert(bus.has_device(UartDeviceId::HAL_Uart2));
    assert(bus.has_device(UartDeviceId::HAL_Uart3));
    assert(u2.get_handle()==&huart2 && u3.get_handle()==&huart3);
    assert(bus.get_uart5().get_handle()==&huart5);
    assert(bus.get_uart7().get_handle()==&huart7);
    uint8_t b2[32]={}, b3[48]={};
    Data d2{b2,32}, d3{b3,48};
    assert(u2.transmit(d2) && last_tx==&huart2);
    assert(u3.transmit_dma(d3) && last_tx==&huart3);
    tx_result=HAL_BUSY;
    assert(!u2.transmit_dma(d2)); tx_result=HAL_OK;
    huart2.hdmarx=&dma2; huart3.hdmarx=&dma3;
    unsigned calls2=0, calls3=0; uint16_t size2=0, size3=0;
    u2.register_rx_callback([&](const Data &d){++calls2; size2=d.size; assert(d.buffer==b2);});
    u3.register_rx_callback([&](const Data &d){++calls3; size3=d.size; assert(d.buffer==b3);});
    assert(u2.receive_dma_idle(d2)); assert(u3.receive_dma_idle(d3));
    unsigned before=starts;
    rx_event=HAL_UART_RXEVENT_HT;
    HAL_UARTEx_RxEventCallback(&huart2,16);
    assert(calls2==0 && calls3==0 && starts==before);
    rx_event=HAL_UART_RXEVENT_IDLE;
    HAL_UARTEx_RxEventCallback(&huart2,7);
    assert(calls2==1 && size2==7 && calls3==0);
    assert(starts==before+1 && last_rx==&huart2 && huart2.RxXferSize==32);
    rx_event=HAL_UART_RXEVENT_TC;
    HAL_UARTEx_RxEventCallback(&huart3,48);
    assert(calls3==1 && size3==48 && starts==before+2 && huart3.RxXferSize==48);
    HAL_UARTEx_RxEventCallback(&huart1,3);
    assert(calls2==1 && calls3==1 && starts==before+2);
    assert(ht_disables==2);
    HAL_UARTEx_RxEventCallback(&huart7,82);
    assert(calls2==1 && calls3==1 && starts==before+2 && ht_disables==2);
}
'''
with tempfile.TemporaryDirectory(prefix="rs485_uart_") as directory:
    work = Path(directory)
    shutil.copytree(root / "user/core/HAL/UART", work / "UART")
    (work / "main.h").write_text(stub, encoding="utf-8")
    (work / "usart.h").write_text('#pragma once\n#include "main.h"\n', encoding="utf-8")
    (work / "test.cpp").write_text(checks + callbacks + main, encoding="utf-8")
    exe = work / "test.exe"
    subprocess.run(["g++", "-std=c++17", "-I", ".", "test.cpp",
                    "UART/interface/uart_bus.cpp", "UART/impl/uart_bus_impl.cpp",
                    "UART/impl/uart_device_impl.cpp", "-o", "test.exe"], cwd=work, check=True)
    subprocess.run([str(exe)], check=True)
print("RS485 UART device routing and RX callbacks passed")
