"""Exercise the production send method with a simulated HAL and critical section."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'user/core/HAL/FDCAN/impl/fdcan_device_impl.cpp').read_text(encoding='utf-8')
start = source.index('bool FdcanDevice::send(')
end = source.index('\nbool FdcanDevice::receive(', start)
method = source[start:end]
harness = r'''
#include <cassert>
#include <cstdint>
int depth = 0, calls = 0, result = 0;
#define taskENTER_CRITICAL() (++depth)
#define taskEXIT_CRITICAL() (--depth)
enum { FDCAN_EXTENDED_ID, FDCAN_STANDARD_ID, FDCAN_REMOTE_FRAME, FDCAN_DATA_FRAME,
       FDCAN_ESI_ACTIVE, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN, FDCAN_NO_TX_EVENTS, HAL_OK = 0 };
struct Frame { uint8_t data[8]{}; uint32_t id=0, mailbox=0; uint8_t dlc=8; bool is_extended_id=false, is_remote_frame=false; };
struct Header { uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator,
    BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker; };
using FDCAN_TxHeaderTypeDef = Header;
struct IFdcanDevice { static int to_dlc_code(int n) { return n; } };
int HAL_FDCAN_AddMessageToTxFifoQ(int *handle, Header *header, const uint8_t *data) {
    assert(depth == 1 && handle && header->Identifier == 0x1ff && data[0] == 42);
    ++calls; return result;
}
struct FdcanDevice { int *handle_; bool send(const Frame &frame); };
METHOD
int main() {
    Frame frame; frame.id=0x1ff; frame.data[0]=42;
    FdcanDevice missing{nullptr}; assert(!missing.send(frame)); assert(depth==0 && calls==0);
    int handle=0; FdcanDevice device{&handle};
    assert(device.send(frame)); assert(depth==0 && calls==1);
    result=1; assert(!device.send(frame)); assert(depth==0 && calls==2);
}
'''.replace('METHOD', method)
with tempfile.TemporaryDirectory() as directory:
    work = Path(directory)
    (work / 'test.cpp').write_text(harness, encoding='utf-8')
    exe = work / 'test.exe'
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    str(work / 'test.cpp'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print('PASS: FDCAN send critical section, success, failure and null handle')
