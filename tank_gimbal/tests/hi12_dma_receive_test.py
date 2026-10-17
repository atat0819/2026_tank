from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'RtosTask/remote_control_task.cpp').read_text(encoding='utf-8-sig')
start = source.index('// IMU circular DMA receive')
end = source.index('// End IMU circular DMA receive', start)
helpers = source[start:end]
uart7_start = source.index('// UART7 — IMU DMA')
body_start = source.index('{', source.index('if (huart == uart7.get_handle())', uart7_start))
depth, body_end = 1, body_start + 1
while depth:
    depth += (source[body_end] == '{') - (source[body_end] == '}')
    body_end += 1
callback = '\nvoid SampleImuRxEvent(uint16_t Size) {\n' + \
    'auto &uart7 = HAL::UART::get_uart_bus_instance().get_uart7();\n' + \
    source[body_start + 1:body_end - 1] + '\n}\n'
prefix = r'''
#include <cassert>
#include <cstdint>
#include <vector>
struct UART_HandleTypeDef {};
enum { HAL_OK, HAL_ERROR };
bool abort_ok=true;
unsigned abort_calls=0;
int HAL_UART_AbortReceive(UART_HandleTypeDef *) { ++abort_calls; return abort_ok ? HAL_OK : HAL_ERROR; }
#define __HAL_UART_CLEAR_OREFLAG(handle) ((void)(handle))
struct Imu {
    unsigned resets=0;
    std::vector<uint8_t> received;
    void ResetReceiveBuffer() { ++resets; }
    void DataUpdate(const uint8_t *p, uint16_t n) { received.insert(received.end(),p,p+n); }
} imu;
namespace HAL::UART {
struct Data { uint8_t *buffer; uint16_t size; };
struct Uart {
    UART_HandleTypeDef handle;
    bool start_ok=true;
    unsigned starts=0;
    Data last{};
    UART_HandleTypeDef *get_handle() { return &handle; }
    bool receive_dma_idle(Data &data) { ++starts; last=data; return start_ok; }
    void trigger_rx_callbacks(const Data &data) { imu.DataUpdate(data.buffer,data.size); }
};
struct Bus { Uart uart; Uart& get_uart7() { return uart; } } bus;
Bus& get_uart_bus_instance() { return bus; }
}
'''
suffix = r'''
int main() {
    auto &uart=HAL::UART::bus.uart;
    StartImuReceive();
    assert(imu_rx_active && uart.starts==1 && uart.last.buffer==imu_dma_buffer);
    assert(uart.last.size==164 && imu_dma_read_pos==0);
    for (unsigned i=0;i<IMU_DMA_BUF_SIZE;++i) imu_dma_buffer[i]=i;
    HandleImuRxEvent(20); // IDLE with a fragment.
    HandleImuRxEvent(20); // Duplicate IDLE: no duplicate bytes.
    HandleImuRxEvent(82); // DMA half transfer.
    HandleImuRxEvent(82); // IDLE at the same boundary.
    HandleImuRxEvent(120);
    HandleImuRxEvent(164); // DMA complete.
    HandleImuRxEvent(164); // IDLE after DMA complete.
    assert(uart.starts==1 && imu.received.size()==164);
    for (unsigned i=0;i<164;++i) assert(imu.received[i]==i);
    for (unsigned i=0;i<82;++i) imu_dma_buffer[i]=static_cast<uint8_t>(164+i);
    HandleImuRxEvent(82); // Circular wrap.
    assert(imu.received.size()==246);
    for (unsigned i=164;i<246;++i) assert(imu.received[i]==static_cast<uint8_t>(i));
    HandleImuRxEvent(0); HandleImuRxEvent(165);
    assert(imu.received.size()==246);
    // A callback can cross the ring end even when the intermediate TC was not handled.
    imu_dma_read_pos=150;
    for (unsigned i=150;i<164;++i) imu_dma_buffer[i]=i;
    for (unsigned i=0;i<10;++i) imu_dma_buffer[i]=i;
    HandleImuRxEvent(10);
    assert(imu.received.size()==270);
    for (unsigned i=0;i<14;++i) assert(imu.received[246+i]==150+i);
    for (unsigned i=0;i<10;++i) assert(imu.received[260+i]==i);
    imu_rx_active=false; HandleImuRxEvent(20);
    assert(imu.received.size()==270);
    uart.start_ok=false; StartImuReceive();
    assert(!imu_rx_active && imu_dma_read_pos==0);
    uart.start_ok=true; StartImuReceive();
    assert(imu_rx_active && imu_dma_read_pos==0 && imu.resets==3);
    abort_ok=false; StartImuReceive(); assert(!imu_rx_active && uart.starts==3);
    abort_ok=true; StartImuReceive(); assert(imu_rx_active && uart.starts==4);
}
'''
assert 'imu.DataUpdate(data.buffer, data.size);' in source
uart7_branch = source[source.index('// UART7 — IMU DMA'):source.index('extern "C" void HAL_UART_ErrorCallback')]
assert 'HandleImuRxEvent' not in source and 'receive_dma_idle(next_rx)' not in uart7_branch
suffix = suffix.replace('HandleImuRxEvent(', 'SampleImuRxEvent(')
assert '__HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT)' not in source
assert 'taskENTER_CRITICAL();' in source and 'taskEXIT_CRITICAL();' in source
usart = (root / 'Core/Src/usart.c').read_text(encoding='utf-8-sig')
assert 'huart7.Init.BaudRate = 115200;' in usart
assert 'hdma_uart7_rx.Init.Mode = DMA_CIRCULAR;' in usart
assert 'Dma.UART7_RX.1.Mode=DMA_CIRCULAR' in (root / 'tank_gimbal.ioc').read_text(encoding='utf-8-sig')
with tempfile.TemporaryDirectory(prefix='hi12_dma_') as tmp:
    path = Path(tmp)
    (path / 'test.cpp').write_text(prefix + helpers + callback + suffix, encoding='utf-8')
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    str(path / 'test.cpp'), '-o', str(path / 'test.exe')], check=True)
    subprocess.run([str(path / 'test.exe')], check=True)
print('PASS: IMU circular DMA fragments, HT/TC/IDLE duplicates, wrap, uninterrupted reception and restart failures')
