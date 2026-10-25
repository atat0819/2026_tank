#include "boards_communication_uart_485.hpp"
#include "gimbal_refree_uart_485.hpp"
#include <math.h>
#include <string.h>

static_assert(sizeof(float) == 4, "RS485 protocol requires 4-byte float");
static uint8_t rx_frame[28];
static uint8_t rx_size = 0;

// CRC16/MCRF4XX，覆盖帧头和数据的前 26 字节，低字节在前。
static uint16_t Crc16(const uint8_t *data, uint16_t size)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < size; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 1U) ? (crc >> 1) ^ 0x8408U : crc >> 1;
        }
    }
    return crc;
}

void Chassis485_ResetRx()
{
    rx_size = 0;
}

void Chassis485_Parse(const uint8_t *data, uint16_t size)
{
    for (uint16_t i = 0; i < size; ++i)
    {
        rx_frame[rx_size++] = data[i];
        if (rx_size < sizeof(rx_frame))
        {
            continue;
        }
        const uint16_t crc = rx_frame[26] | (static_cast<uint16_t>(rx_frame[27]) << 8);
        if (rx_frame[0] != 0x78 || rx_frame[1] != 0x91 || Crc16(rx_frame, 26) != crc)
        {
            // 每次前移一字节寻找下一帧，错误帧内部的有效帧头也能恢复。
            memmove(rx_frame, rx_frame + 1, sizeof(rx_frame) - 1);
            rx_size = sizeof(rx_frame) - 1;
            continue;
        }

        Chassis485Data decoded{};
        memcpy(&decoded.right_stick_x, &rx_frame[2], sizeof(int16_t));
        memcpy(&decoded.mouse_x, &rx_frame[4], sizeof(int16_t));
        memcpy(&decoded.vx, &rx_frame[6], sizeof(float));
        memcpy(&decoded.vy, &rx_frame[10], sizeof(float));
        decoded.s1 = rx_frame[14];
        decoded.s2 = rx_frame[15];
        memcpy(&decoded.imu_yaw_deg, &rx_frame[16], sizeof(float));
        memcpy(&decoded.imu_gyro_z_deg_s, &rx_frame[20], sizeof(float));
        decoded.keyboard = rx_frame[24] | (static_cast<uint16_t>(rx_frame[25]) << 8);
        rx_size = 0;

        const bool switches_valid = decoded.s1 >= 1U && decoded.s1 <= 3U &&
                                    decoded.s2 >= 1U && decoded.s2 <= 3U;
        // 云台明确失联帧：档位及全部操作量归零，立即撤销云台输入有效性。
        const bool offline = decoded.s1 == 0U && decoded.s2 == 0U &&
                             decoded.right_stick_x == 0 && decoded.mouse_x == 0 &&
                             decoded.vx == 0.0f && decoded.vy == 0.0f && decoded.keyboard == 0U;
        if ((!switches_valid && !offline) ||
            decoded.right_stick_x < -660 || decoded.right_stick_x > 660 ||
            !isfinite(decoded.vx) || !isfinite(decoded.vy) ||
            !isfinite(decoded.imu_yaw_deg) || !isfinite(decoded.imu_gyro_z_deg_s))
        {
            continue;
        }
        Chassis485_ApplyData(decoded);
        // 云台发起、底盘收到有效请求后回复；不得独立周期抢占线路。
        Referee485_SendReply();
    }
}
