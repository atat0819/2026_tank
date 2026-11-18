"""Host-check the production USB JustFloat sender with up to 30 channels."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
buffer = re.search(r"uint8_t send_str2\[[^;]+\];", source).group()
start = source.index("void vofa_sendN(const float *data, uint8_t count)\n{")
sender = source[start:source.index("\n}", start) + 2]

code = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
struct USBD_CDC_HandleTypeDef { uint32_t TxState = 0; } cdc;
struct { void *pClassData = &cdc; } hUsbDeviceHS;
unsigned calls = 0;
uint16_t sent_length = 0;
uint8_t captured[124] = {};
uint8_t CDC_Transmit_HS(uint8_t *, uint16_t);
BUFFER
SENDER
uint8_t CDC_Transmit_HS(uint8_t *data, uint16_t length) {
    assert(length <= sizeof(captured) && length <= sizeof(send_str2));
    std::memcpy(captured, data, length);
    sent_length = length;
    ++calls;
    cdc.TxState = 1;
    return 0;
}
int main() {
    float data[31];
    for (unsigned i = 0; i < 31; ++i)
        data[i] = (i % 2 ? -1.0f : 1.0f) * (i + 0.25f);

    vofa_sendN(data, 30);
    assert(calls == 1);
    assert(sent_length == 124 && "30 channels must produce a 124-byte frame");
    assert(std::memcmp(captured, data, 120) == 0);
    const uint8_t tail[] = {0x00, 0x00, 0x80, 0x7F};
    assert(std::memcmp(captured + 120, tail, sizeof(tail)) == 0);

    // A pending USB transfer must keep its backing buffer unchanged.
    uint8_t pending[sizeof(send_str2)];
    std::memcpy(pending, send_str2, sizeof(pending));
    data[0] = 99.0f;
    vofa_sendN(data, 30);
    assert(calls == 1 && std::memcmp(pending, send_str2, sizeof(pending)) == 0);
    cdc.TxState = 0;
    vofa_sendN(nullptr, 30);
    vofa_sendN(data, 0);
    hUsbDeviceHS.pClassData = nullptr;
    vofa_sendN(data, 30);
    assert(calls == 1);
    hUsbDeviceHS.pClassData = &cdc;

    const uint8_t counts[] = {1, 6, 10, 30, 31, 255};
    for (uint8_t count : counts) {
        cdc.TxState = 0;
        const unsigned before = calls;
        vofa_sendN(data, count);
        const unsigned channels = count > 30 ? 30 : count;
        assert(calls == before + 1 && sent_length == (channels + 1) * 4);
        assert(std::memcmp(captured, data, channels * 4) == 0);
        assert(std::memcmp(captured + channels * 4, tail, sizeof(tail)) == 0);
    }
}
'''.replace("BUFFER", buffer).replace("SENDER", sender)

with tempfile.TemporaryDirectory(prefix="usb_vofa_") as directory:
    cpp, exe = Path(directory) / "test.cpp", Path(directory) / "test.exe"
    cpp.write_text(code, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    str(cpp), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print("PASS: USB JustFloat 30 channels, payload/tail, bounds and busy guards")
