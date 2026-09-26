"""提取实际偏移计算及发送函数，用主机 CAN/电机替身验证发送内容。"""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "communication_between_boards/boards_communication.cpp").read_text(encoding="utf-8")
offset_code = source[source.index("#define YAW_FRONT_OFFSET_DEG"):source.index("HAL_StatusTypeDef CAN2_SendChassisSpeed")]
# 截取点位于下一函数的注释之后，移除末尾注释即可独立编译。
offset_code = re.sub(r"/\*\*[^/]*\*/\s*$", "", offset_code)
stub = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR };
struct { float angle_deg; } dm4340_state[2] = {};
struct Motor {
    bool online = false;
    float angle = 0.0f;
    bool isConnected(int id, int can_id) { assert(id == 1 && can_id == 1); return online; }
    float getAngleDeg(int id) { assert(id == 1); return angle; }
} gimbal_motor;
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
    dm4340_state[0].angle_deg = 90.0f; // 缓存故意保留旧角度。
    gimbal_motor.angle = 90.0f;
    expect(0.0f); // 无反馈时不得发送旧角度或标定常数。
    gimbal_motor.online = true;
    expect(90.0f + YAW_FRONT_OFFSET_DEG);
    gimbal_motor.online = false;
    expect(0.0f);
    assert(yaw_offset_deg == 0.0f);
    gimbal_motor.angle = -170.0f;
    gimbal_motor.online = true;
    expect(-170.0f + YAW_FRONT_OFFSET_DEG + 360.0f); // 恢复后使用最新反馈。
}
'''
with tempfile.TemporaryDirectory(prefix="yaw_offset_test_") as directory:
    cpp = Path(directory) / "test.cpp"
    exe = Path(directory) / "test.exe"
    cpp.write_text(stub + offset_code + checks, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", str(cpp), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print("PASS: Yaw offline zero, online offset, recovery, and CAN payload")
