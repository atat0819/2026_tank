"""提取两端实际 UART3 错误分支，验证 TX DMA 错误后释放 RX DMA 再重启。"""
from pathlib import Path
import subprocess
import tempfile

chassis = Path(__file__).resolve().parents[1]
gimbal = chassis.parent / "tank_gimbal"
stub = r'''
#include <cassert>
#include <cstdint>
struct DMA_HandleTypeDef { bool busy=true; } dma;
struct UART_HandleTypeDef {
    uint8_t *pRxBuffPtr;
    uint16_t RxXferSize;
    DMA_HandleTypeDef *hdmarx;
    int gState=77;
};
uint8_t uart3_rx_buffer[64];
UART_HandleTypeDef huart3{uart3_rx_buffer,64,&dma}, huart2{};
unsigned aborts=0,starts=0,ht_disabled=0,resets=0;
enum { HAL_OK, HAL_BUSY };
int HAL_UART_AbortReceive(UART_HandleTypeDef *h) { assert(h==&huart3); ++aborts; h->hdmarx->busy=false; return HAL_OK; }
#define __HAL_UART_CLEAR_OREFLAG(h) ((void)0)
#define __HAL_UART_CLEAR_NEFLAG(h) ((void)0)
#define __HAL_UART_CLEAR_FEFLAG(h) ((void)0)
#define __HAL_UART_CLEAR_PEFLAG(h) ((void)0)
#define __HAL_DMA_DISABLE_IT(h,f) (++ht_disabled)
#define DMA_IT_HT 1
void Chassis485_ResetRx() { ++resets; }
namespace HAL::UART {
struct Data { uint8_t *buffer; uint16_t size; };
struct Device {
    bool receive_dma_idle(Data &d) {
        if (dma.busy) return false;
        assert(d.buffer==uart3_rx_buffer && d.size==64);
        ++starts; dma.busy=true; return true;
    }
};
struct Bus { Device device; Device &get_uart3() { return device; } Device &get_uart2() { return device; } } bus;
Bus &get_uart_bus_instance() { return bus; }
}
HAL::UART::Data uart3_rx_data{uart3_rx_buffer,64};
'''
for repo, file, end in (
    (chassis, "RtosTask/remote_task.cpp", "    if (huart->Instance == UART5)"),
    (gimbal, "RtosTask/remote_control_task.cpp", "    auto &uart5 ="),
):
    source = (repo / file).read_text(encoding="utf-8")
    start = source.index('extern "C" void HAL_UART_ErrorCallback')
    # Only exercise the RS485 branch; other UARTs have their own host tests.
    start = source.index("    if (huart == &huart3)" if repo == chassis else
                         "    if (huart == &huart2 || huart == &huart3)", start)
    callback = ('extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {\n'
                + source[start:source.index(end, start)] + "\n}\n")
    checks = r'''
int main() {
    HAL_UART_ErrorCallback(&huart3);
    assert(aborts==1 && starts==1 && ht_disabled==1);
    assert(huart3.gState==77 && dma.busy);
}
'''
    with tempfile.TemporaryDirectory(prefix="rs485_error_") as directory:
        work=Path(directory)
        (work/"test.cpp").write_text(stub+callback+checks,encoding="utf-8")
        subprocess.run(["g++","-std=c++17","test.cpp","-o","test.exe"],cwd=work,check=True)
        subprocess.run([str(work/"test.exe")],check=True)
print("Both RS485 error callbacks release RX DMA before restart and preserve TX state")
