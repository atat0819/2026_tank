#pragma once
#include "stm32h7xx_hal.h"

// 在 can_send_task 中更新最新裁判快照，不在此函数中发送。
void Referee485_Update(uint16_t cooling_value, uint16_t heat_limit,
                       uint16_t heat_42mm, uint8_t referee_online);
// 仅 UART RX ISR 收到有效请求后调用；91 78 + 原大端数据区 8 字节 + CRC16 = 12 字节。
// 冷却值/热量沿用原 CAN 含义，第 6 字节为裁判在线标志，第 7 字节保留。
HAL_StatusTypeDef Referee485_SendReply();
