#include "refree_receive_uart_485.hpp"
#include "boards_communication_uart_485.hpp"
#include "main.h"
#include <string.h>

namespace Communication
{

GimbalRefree485 &GimbalRefree485::instance()
{
    static GimbalRefree485 inst;
    return inst;
}

uint32_t GimbalRefree485::getDataAgeMs(uint32_t now_tick_ms) const
{
    if (!data_received_)
    {
        return UINT32_MAX;
    }
    return static_cast<uint32_t>(now_tick_ms - last_receive_tick_ms_);
}

void GimbalRefree485::parse485(const uint8_t *data, uint16_t size)
{
    static uint8_t frame[12];
    static uint8_t received = 0;
    for (uint16_t i = 0; i < size; ++i)
    {
        frame[received++] = data[i];
        if (received < sizeof(frame))
        {
            continue;
        }
        uint16_t crc = 0xFFFF;
        for (uint8_t j = 0; j < 10; ++j)
        {
            crc ^= frame[j];
            for (uint8_t bit = 0; bit < 8; ++bit)
            {
                crc = (crc & 1U) ? (crc >> 1) ^ 0x8408U : crc >> 1;
            }
        }
        const uint16_t received_crc = frame[10] | (static_cast<uint16_t>(frame[11]) << 8);
        if (frame[0] != 0x91 || frame[1] != 0x78 || crc != received_crc || frame[8] > 1U)
        {
            memmove(frame, frame + 1, sizeof(frame) - 1);
            received = sizeof(frame) - 1;
            continue;
        }
        received = 0;
        // 与原裁判 CAN 数据区的大端顺序保持一致，更新独立的 RS485 数据。
        cooling_value_ = (static_cast<uint16_t>(frame[2]) << 8) | frame[3];
        heat_limit_ = (static_cast<uint16_t>(frame[4]) << 8) | frame[5];
        heat_42mm_ = (static_cast<uint16_t>(frame[6]) << 8) | frame[7];
        referee_online_ = frame[8] != 0;
        last_receive_tick_ms_ = HAL_GetTick();
        data_received_ = true;
        RS485_ReplyReceived();
    }
}

} // namespace Communication
