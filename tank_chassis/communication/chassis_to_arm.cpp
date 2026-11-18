#include "chassis_to_arm.hpp"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include <string.h>

ChassisToArm chassis_to_arm;

bool ChassisToArm::DebouncedKey::Pressed(bool value, uint32_t now)
{
    if (value != raw)
    {
        raw = value;
        changed_ms = now;
    }
    if (raw != confirmed && now - changed_ms >= KEY_DEBOUNCE_MS)
    {
        confirmed = raw;
        return confirmed;
    }
    return false;
}

void ChassisToArm::ResetKeys()
{
    f_key = {};
    g_key = {};
    f_count = 0;
    f_pending = 0;
    f_active = false;
    g_state = false;
    key_mode_active = false;
}

void ChassisToArm::UpdateKeys(uint8_t s1, uint8_t s2, uint16_t keys, uint32_t now)
{
    if (s1 != 3U || s2 != 3U)
    {
        ResetKeys();
        return;
    }

    if (!key_mode_active)
    {
        // 进入双中时同步现有电平；已经按住的键必须先松开，再作为新按下计数。
        f_key.raw = f_key.confirmed = (keys & KEY_F) != 0U;
        g_key.raw = g_key.confirmed = (keys & KEY_G) != 0U;
        f_key.changed_ms = g_key.changed_ms = now;
        key_mode_active = true;
    }

    // 到期的结果等待 DMA 成功启动；下一轮可在结果等待期间开始。
    if (f_active && now - f_start_ms >= F_WINDOW_MS && f_pending == 0U)
    {
        f_pending = f_count;
        f_count = 0;
        f_active = false;
    }

    const bool f_pressed = f_key.Pressed((keys & KEY_F) != 0U, now);
    const bool g_pressed = g_key.Pressed((keys & KEY_G) != 0U, now);
    if (f_pressed)
    {
        if (!f_active)
        {
            f_active = true;
            f_start_ms = now;
            f_count = 1U;
        }
        else if (now - f_start_ms < F_WINDOW_MS && f_count < UINT16_MAX)
        {
            ++f_count;
        }
    }
    if (g_pressed)
    {
        g_state = !g_state;
    }
}

// 唯一发送缓冲区：仅在 gState READY 时写入，DMA 完成前不能覆盖。
HAL_StatusTypeDef ChassisToArm::SendFrame(bool input_online, uint8_t s1, uint8_t s2, uint16_t keyboard)
{
    auto &uart10 = HAL::UART::get_uart_bus_instance().get_uart10();
    if (uart10.get_handle()->gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    memset(tx, 0, sizeof(tx));
    tx[0] = 0x25;
    tx[1] = 0x44;
    if (input_online)
    {
        tx[2] = s1;
        tx[3] = s2;
        tx[4] = static_cast<uint8_t>(keyboard);
        tx[5] = static_cast<uint8_t>(keyboard >> 8);
        tx[6] = static_cast<uint8_t>(f_pending);
        tx[7] = static_cast<uint8_t>(f_pending >> 8);
        tx[8] = g_state ? 1U : 0U;
        tx[9] = 1U;
    }

    uint16_t crc = 0xFFFFU;
    for (unsigned i = 0; i < 10; ++i)
    {
        crc ^= tx[i];
        for (unsigned bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 1U) ? (crc >> 1) ^ 0x8408U : crc >> 1;
        }
    }
    tx[10] = static_cast<uint8_t>(crc);
    tx[11] = static_cast<uint8_t>(crc >> 8);
    HAL::UART::Data data{tx, sizeof(tx)};
    if (!uart10.transmit_dma(data))
    {
        return HAL_ERROR;
    }
    if (input_online)
    {
        f_pending = 0U; // 只在成功启动的帧中发布一次
    }
    return HAL_OK;
}
HAL_StatusTypeDef ChassisToArm::SendRemoteData(uint8_t s1, uint8_t s2, uint16_t keyboard, bool is_online)
{
    if (is_online)
    {
        UpdateKeys(s1, s2, keyboard, HAL_GetTick()); // DMA 忙时仍更新按键状态。
    }
    else
    {
        ResetKeys();
    }
    return SendFrame(is_online, s1, s2, keyboard);
}
