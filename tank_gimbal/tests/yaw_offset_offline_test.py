"""提取实际偏移计算及发送函数，用主机 CAN/电机替身验证发送内容。"""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "communication_between_boards/boards_communication.cpp").read_text(encoding="utf-8")
offset_code = source[source.index("float YawOffset_GetDeg(void)"):source.index("HAL_StatusTypeDef CAN2_SendChassisSpeed")]
# 截取点位于下一函数的注释之后，移除末尾注释即可独立编译。
offset_code = re.sub(r"/\*\*[^/]*\*/\s*$", "", offset_code)
stub = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR };
namespace HAL { namespace FDCAN {
enum class FdcanDeviceId { HAL_Fdcan2 };
struct Frame { uint32_t id; uint8_t dlc; bool is_extended_id, is_remote_frame; uint8_t data[8]; };
Frame sent;
struct Device { bool send(const Frame &f) { sent = f; return true; } } device;
struct Bus { Device &get_device(FdcanDeviceId) { return device; } } bus;
Bus &get_fdcan_bus_instance() { return bus; }
}}
'''
checks = r'''
void expect(float expected) {
    assert(YawOffset_SendToCan2() == HAL_OK);
    const auto &f = HAL::FDCAN::sent;
    assert(f.id == 0x301 && f.dlc == 8 && !f.is_extended_id && !f.is_remote_frame);
    float actual;
    memcpy(&actual, f.data, sizeof(actual));
    assert(std::fabs(actual - expected) < 0.001f);
    for (int i = 4; i < 8; ++i) assert(f.data[i] == 0);
}
int main() {
    expect(0.0f); // 固定云台相对底盘偏角为零，不依赖电机。
    expect(0.0f);
}
'''
with tempfile.TemporaryDirectory(prefix="yaw_offset_test_") as directory:
    cpp = Path(directory) / "test.cpp"
    exe = Path(directory) / "test.exe"
    cpp.write_text(stub + offset_code + checks, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", str(cpp), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print("PASS: fixed zero relative Yaw and legacy CAN payload")
