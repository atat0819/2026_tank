"""Check the real six-channel VOFA packet and DMA buffer ownership."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
code = re.sub(r"//[^\n]*|/\*.*?\*/", "", source, flags=re.S)
assert "vofa_send(motor_target_speed[0]" not in code, \
    "wheel telemetry must not mix with the IMU comparison stream"
feedback = code.index("current_speed_rads[0] =")
# Function + task loop only: STOP and input faults must not skip wheel feedback.
assert code[:feedback].count("{") - code[:feedback].count("}") == 2, \
    "wheel feedback must remain outside control-only branches"
assert feedback < code.index("if (input.source == ControlInputSource::NONE || source_changed)")
match = re.search(r"void vofa_send\([^;{}]*\)\s*\{", source)
depth, end = 1, match.end()
while depth:
    depth += (source[end] == "{") - (source[end] == "}")
    end += 1
function = source[match.start():end]
host = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
constexpr int HAL_UART_STATE_READY = 0, HAL_UART_STATE_BUSY_TX = 1;
namespace HAL::UART {
struct Data { uint8_t *buffer; uint16_t size; };
struct Handle { int gState = HAL_UART_STATE_READY; };
struct Device {
    Handle handle;
    Data active{};
    unsigned sends = 0;
    Handle *get_handle() { return &handle; }
    bool transmit_dma(const Data &data) {
        if (handle.gState != HAL_UART_STATE_READY) return false;
        active = data;
        ++sends;
        handle.gState = HAL_UART_STATE_BUSY_TX;
        return true;
    }
};
struct Bus { Device uart; Device &get_uart10() { return uart; } } bus;
Bus &get_uart_bus_instance() { return bus; }
}
'''
host += function + r'''
int main() {
    auto &uart = HAL::UART::bus.uart;
    vofa_send(1, 2, 3, 4, 5, 6);
    assert(uart.sends == 1 && uart.active.size == 28);
    const float first[6] = {1, 2, 3, 4, 5, 6};
    const uint8_t tail[4] = {0, 0, 0x80, 0x7f};
    assert(std::memcmp(uart.active.buffer, first, sizeof(first)) == 0);
    assert(std::memcmp(uart.active.buffer + sizeof(first), tail, sizeof(tail)) == 0);
    uint8_t in_flight[28];
    std::memcpy(in_flight, uart.active.buffer, sizeof(in_flight));
    vofa_send(7, 8, 9, 10, 11, 12);
    assert(uart.sends == 1);
    assert(std::memcmp(uart.active.buffer, in_flight, sizeof(in_flight)) == 0);
    uart.handle.gState = HAL_UART_STATE_READY;
    vofa_send(-1.25f, 0, 3.5f, -4, 5, 6);
    const float next[6] = {-1.25f, 0, 3.5f, -4, 5, 6};
    assert(uart.sends == 2);
    assert(std::memcmp(uart.active.buffer, next, sizeof(next)) == 0);
    assert(std::memcmp(uart.active.buffer + sizeof(next), tail, sizeof(tail)) == 0);
}
'''
with tempfile.TemporaryDirectory() as directory:
    cpp = Path(directory) / "vofa.cpp"
    exe = Path(directory) / "vofa.exe"
    cpp.write_text(host, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-O2", cpp.as_posix(), "-o", exe.as_posix()], check=True)
    subprocess.run([str(exe)], check=True)
print("PASS: six float channels, JustFloat tail, persistent DMA buffer and busy-send skip")
