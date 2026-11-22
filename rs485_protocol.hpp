#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>

namespace BoardLink
{
constexpr unsigned kDbusSize = 18;
constexpr unsigned kChassisSize = 32;
constexpr unsigned kYawSize = 8;
constexpr float kYawForwardDeg = 24.0f;

inline float yawOffsetDeg(float encoder_deg)
{
    if (!isfinite(encoder_deg)) return encoder_deg;
    float offset = fmodf(encoder_deg - kYawForwardDeg, 360.0f);
    if (offset >= 180.0f) offset -= 360.0f;
    if (offset < -180.0f) offset += 360.0f;
    return offset;
}

struct ChassisData
{
    uint8_t dbus[kDbusSize];       // 接收机发来的完整 18 字节，不重新组装。
    uint16_t remote_sequence;      // 每收到一帧有效 DBUS 就递增，防止云台重复解析。
    uint8_t remote_online;
    uint16_t cooling;
    uint16_t heat_limit;
    uint16_t heat_42mm;
    uint8_t referee_online;
    bool yaw_mouse_active;
};

inline uint16_t crc16(const uint8_t *data, unsigned size)
{
    uint16_t crc = 0xffff;
    for (unsigned i = 0; i < size; ++i)
    {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc & 1U) ? (crc >> 1) ^ 0x8408U : crc >> 1;
    }
    return crc;
}

inline void put16(uint8_t *data, uint16_t value)
{
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8);
}

inline uint16_t get16(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

// 只检查 DBUS 通道与拨杆范围，实际解析仍由 RemoteController 完成。
inline bool dbusValid(const uint8_t *data)
{
    const unsigned ch0 = (data[0] | (data[1] << 8)) & 0x07ffU;
    const unsigned ch1 = ((data[1] >> 3) | (data[2] << 5)) & 0x07ffU;
    const unsigned ch2 = ((data[2] >> 6) | (data[3] << 2) | (data[4] << 10)) & 0x07ffU;
    const unsigned ch3 = ((data[4] >> 1) | (data[5] << 7)) & 0x07ffU;
    const unsigned s1 = (data[5] >> 6) & 3U;
    const unsigned s2 = (data[5] >> 4) & 3U;
    const unsigned wheel = get16(data + 16);
    return ch0 >= 364U && ch0 <= 1684U && ch1 >= 364U && ch1 <= 1684U &&
           ch2 >= 364U && ch2 <= 1684U && ch3 >= 364U && ch3 <= 1684U &&
           s1 >= 1U && s1 <= 3U && s2 >= 1U && s2 <= 3U &&
           wheel >= 364U && wheel <= 1684U;
}

inline bool dbusKeymouse(const uint8_t *data)
{
    return (data[5] & 0xf0U) == 0xf0U;
}

inline bool dbusYawMouseActive(const uint8_t *data)
{
    return dbusKeymouse(data) && data[13] != 0U;
}

// 底盘 -> 云台：91 78 + DBUS[18] + 标志[2] + 裁判数据[6] + 序号[2] + CRC[2]。
// 字节 20：bit0 遥控在线，bit1 本帧鼠标 X 分配给相机 Yaw。
// 多字节整数使用小端顺序；CRC 覆盖前 30 字节，低字节在前。
inline void encodeChassis(uint8_t *frame, const ChassisData &data)
{
    frame[0] = 0x91;
    frame[1] = 0x78;
    memcpy(frame + 2, data.dbus, kDbusSize);
    frame[20] = data.remote_online ? (data.yaw_mouse_active ? 3U : 1U) : 0U;
    frame[21] = data.referee_online;
    put16(frame + 22, data.cooling);
    put16(frame + 24, data.heat_limit);
    put16(frame + 26, data.heat_42mm);
    put16(frame + 28, data.remote_sequence);
    put16(frame + 30, crc16(frame, 30));
}

inline bool decodeChassis(const uint8_t *frame, ChassisData &data)
{
    if (frame[0] != 0x91 || frame[1] != 0x78 ||
        get16(frame + 30) != crc16(frame, 30) ||
        frame[20] > 3U || frame[20] == 2U || frame[21] > 1U)
        return false;

    memcpy(data.dbus, frame + 2, kDbusSize);
    data.remote_online = frame[20] & 1U;
    data.yaw_mouse_active = (frame[20] & 2U) != 0U;
    data.referee_online = frame[21];
    data.cooling = get16(frame + 22);
    data.heat_limit = get16(frame + 24);
    data.heat_42mm = get16(frame + 26);
    data.remote_sequence = get16(frame + 28);
    return true;
}

// 云台 -> 底盘：78 91 + Yaw 偏移角 float[4] + CRC[2]。
inline void encodeYaw(uint8_t *frame, float angle_deg)
{
    static_assert(sizeof(float) == 4, "RS485 requires a 4-byte float");
    frame[0] = 0x78;
    frame[1] = 0x91;
    memcpy(frame + 2, &angle_deg, 4);
    put16(frame + 6, crc16(frame, 6));
}

inline bool decodeYaw(const uint8_t *frame, float &angle_deg)
{
    if (frame[0] != 0x78 || frame[1] != 0x91 || get16(frame + 6) != crc16(frame, 6))
        return false;
    memcpy(&angle_deg, frame + 2, 4);
    return true;
}
}
