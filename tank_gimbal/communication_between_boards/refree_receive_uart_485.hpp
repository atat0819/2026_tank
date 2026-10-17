#pragma once

#include <stdint.h>

namespace Communication
{

// 云台独立 RS485 裁判接收类，与原 CAN 接收类分别存储数据。
// 回复格式：91 78 + 原裁判数据区 8 字节（大端）+ CRC16 两字节（小端）。
class GimbalRefree485
{
  public:
    static GimbalRefree485 &instance();

    // USART3 DMA+Idle lambda 调用；支持拆帧/粘帧，仅有效帧更新数据与心跳。
    void parse485(const uint8_t *data, uint16_t size);

    uint16_t getCoolingValue() const { return cooling_value_; }
    uint16_t getHeatLimit() const { return heat_limit_; }
    uint16_t getHeat42mm() const { return heat_42mm_; }
    bool isRefereeOnline() const { return referee_online_; }
    bool isDataReceived() const { return data_received_; }
    // 最近有效回复的年龄，单位 ms；未收到数据时返回 UINT32_MAX。
    uint32_t getDataAgeMs(uint32_t now_tick_ms) const;

  private:
    GimbalRefree485() = default;

    uint16_t cooling_value_ = 0;
    uint16_t heat_limit_ = 0;
    uint16_t heat_42mm_ = 0;
    bool referee_online_ = false;
    bool data_received_ = false;
    volatile uint32_t last_receive_tick_ms_ = 0U;
};

} // namespace Communication
