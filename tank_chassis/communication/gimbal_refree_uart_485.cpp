#include "gimbal_refree_uart_485.hpp"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static uint8_t referee_data[8] = {};

void Referee485_Update(uint16_t cooling_value, uint16_t heat_limit,
                       uint16_t heat_42mm, uint8_t referee_online)
{
    const uint8_t snapshot[8] = {
        static_cast<uint8_t>(cooling_value >> 8), static_cast<uint8_t>(cooling_value),
        static_cast<uint8_t>(heat_limit >> 8), static_cast<uint8_t>(heat_limit),
        static_cast<uint8_t>(heat_42mm >> 8), static_cast<uint8_t>(heat_42mm),
        static_cast<uint8_t>(referee_online ? 1U : 0U), 0U
    };
    // 防止 UART ISR 在任务更新到一半时读取到混合快照。
    taskENTER_CRITICAL();
    memcpy(referee_data, snapshot, sizeof(referee_data));
    taskEXIT_CRITICAL();
}

HAL_StatusTypeDef Referee485_SendReply()
{
    auto &uart3_485 = HAL::UART::get_uart_bus_instance().get_uart3();
    if (uart3_485.get_handle()->gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }
    static uint8_t tx_frame[12];
    tx_frame[0] = 0x91;
    tx_frame[1] = 0x78;
    memcpy(tx_frame + 2, referee_data, sizeof(referee_data));
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < 10; ++i)
    {
        crc ^= tx_frame[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 1U) ? (crc >> 1) ^ 0x8408U : crc >> 1;
        }
    }
    tx_frame[10] = static_cast<uint8_t>(crc);
    tx_frame[11] = static_cast<uint8_t>(crc >> 8);
    HAL::UART::Data tx{tx_frame, sizeof(tx_frame)};
    return uart3_485.transmit_dma(tx) ? HAL_OK : HAL_ERROR;
}
