#ifndef CONTROL_INPUT_SELECTOR_HPP
#define CONTROL_INPUT_SELECTOR_HPP

#include <cstdint>

enum class ControlInputSource : uint8_t { NONE, LOCAL };

// 底盘 UART5 在 100 ms 内收到有效 DBUS 帧才提供控制输入。
inline ControlInputSource SelectControlInputSource(uint32_t now_tick,
                                                    bool local_received,
                                                    uint32_t local_last_tick) {
  if (local_received && now_tick - local_last_tick < 100U) {
    return ControlInputSource::LOCAL;
  }
  return ControlInputSource::NONE;
}

#endif  // CONTROL_INPUT_SELECTOR_HPP
