#pragma once

#include "stm32h7xx_hal.h"

// UART7: 921600 8N1，由遥控器任务周期调用，离线时发送安全帧。
// 固定 12 字节，小端：0..1 帧头 25 44；2..3 S1/S2；
// 4..5 键盘掩码；6..7 F 次数（uint16）；8 G 状态（0/1）；
// 9 遥控器状态（0=离线，1=在线）；10..11 CRC16/MCRF4XX。
// CRC 初值 FFFF、反射多项式 8408、无最终异或，覆盖字节 0..9。
// F 结果仅随一帧成功启动发送；离线帧除帧头和 CRC 外均为 0。
// 接收方仍须独立判断板间链路超时。
class GimbalToArm
{
public:
    GimbalToArm() = default;
    GimbalToArm(const GimbalToArm &) = delete;
    GimbalToArm &operator=(const GimbalToArm &) = delete;

    // 任务周期调用；离线时清除按键状态并发送零值帧。
    HAL_StatusTypeDef SendRemoteData(uint8_t s1, uint8_t s2, uint16_t keyboard, bool is_online);

private:
    static constexpr uint32_t KEY_DEBOUNCE_MS = 30U;
    static constexpr uint32_t F_WINDOW_MS = 4000U;
    static constexpr uint16_t KEY_F = 1U << 9;
    static constexpr uint16_t KEY_G = 1U << 10;

    struct DebouncedKey
    {
        bool raw = false;
        bool confirmed = false;
        uint32_t changed_ms = 0;
        bool Pressed(bool value, uint32_t now);
    };

    void ResetKeys();
    void UpdateKeys(uint8_t s1, uint8_t s2, uint16_t keyboard, uint32_t now);
    HAL_StatusTypeDef SendFrame(bool input_online, uint8_t s1, uint8_t s2, uint16_t keyboard);

    DebouncedKey f_key;
    DebouncedKey g_key;
    uint32_t f_start_ms = 0;
    uint16_t f_count = 0;
    uint16_t f_pending = 0;
    bool f_active = false;
    bool g_state = false;
    bool key_mode_active = false;
    uint8_t tx[12] = {};
};

extern GimbalToArm gimbal_to_arm;
