#include "boards_communication_uart_485.hpp"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include <math.h>
#include <string.h>

static_assert(sizeof(float) == 4, "RS485 protocol requires 4-byte float");

static uint32_t last_send_tick = 0U;
static volatile bool waiting_reply = false;

void RS485_ReplyReceived(void)
{
    waiting_reply = false;
}

// CRC16/MCRF4XX：初值 0xFFFF，反射多项式 0x8408，无最终异或。
// 校验范围为字节 0..25，CRC 低字节在前；与项目裁判系统 CRC16 参数一致。
static uint16_t Boards485_Crc16(const uint8_t *data, uint16_t size)
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

HAL_StatusTypeDef RS485_SendBoardData(int16_t right_stick_x, int16_t mouse_x,
                                    float chassis_vx, float chassis_vy,
                                    uint8_t s1, uint8_t s2,
                                    float imu_yaw_deg, float gyro_z_deg_s,
                                    uint16_t keyboard)
{
    auto &uart3_485 = HAL::UART::get_uart_bus_instance().get_uart3();
    // RX DMA 可以同时挂起；只检查发送状态，避免覆盖上一帧仍在读取的缓冲区。
    if (uart3_485.get_handle()->gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    const uint32_t now = HAL_GetTick();
    const uint32_t age = now - last_send_tick;
    if (waiting_reply)
    {
        if (age < 2U)
        {
            return HAL_BUSY;
        }
        waiting_reply = false;
    }

    static uint8_t tx_buffer[28];
    // 沿用原 CAN IMU 帧的度数含义，确保多圈或负角均落在 [0, 360)。
    float imu_yaw_0_360 = fmodf(imu_yaw_deg, 360.0f);
    if (imu_yaw_0_360 < 0.0f)
    {
        imu_yaw_0_360 += 360.0f;
    }

    // STM32H723 为小端，float 使用 IEEE754 单精度；逐字段拷贝避免结构体填充。
    tx_buffer[0] = 0x78;
    tx_buffer[1] = 0x91;
    memcpy(&tx_buffer[2], &right_stick_x, sizeof(right_stick_x));
    memcpy(&tx_buffer[4], &mouse_x, sizeof(mouse_x));
    memcpy(&tx_buffer[6], &chassis_vx, sizeof(float));
    memcpy(&tx_buffer[10], &chassis_vy, sizeof(float));
    tx_buffer[14] = s1;
    tx_buffer[15] = s2;
    memcpy(&tx_buffer[16], &imu_yaw_0_360, sizeof(float));
    memcpy(&tx_buffer[20], &gyro_z_deg_s, sizeof(float));
    tx_buffer[24] = static_cast<uint8_t>(keyboard);
    tx_buffer[25] = static_cast<uint8_t>(keyboard >> 8);
    const uint16_t crc = Boards485_Crc16(tx_buffer, 26);
    tx_buffer[26] = static_cast<uint8_t>(crc);
    tx_buffer[27] = static_cast<uint8_t>(crc >> 8);

    HAL::UART::Data tx{tx_buffer, sizeof(tx_buffer)};
    waiting_reply = true;
    if (!uart3_485.transmit_dma(tx))
    {
        waiting_reply = false;
        return HAL_ERROR;
    }
    last_send_tick = now;
    return HAL_OK;
}
