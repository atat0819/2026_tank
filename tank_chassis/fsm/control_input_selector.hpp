#ifndef CONTROL_INPUT_SELECTOR_HPP
#define CONTROL_INPUT_SELECTOR_HPP

#include <cstdint>

enum class ControlInputSource : uint8_t { NONE, LOCAL, GIMBAL };

// 100 ms 内收到完整控制帧才允许该输入源接管；本地接收机优先。
inline ControlInputSource SelectControlInputSource(uint32_t now_tick,
                                                    bool local_received,
                                                    uint32_t local_last_tick,
                                                    bool gimbal_received,
                                                    uint32_t gimbal_last_tick) {
  if (local_received && now_tick - local_last_tick < 100U) {
    return ControlInputSource::LOCAL;
  }
  if (gimbal_received && now_tick - gimbal_last_tick < 100U) {
    return ControlInputSource::GIMBAL;
  }
  return ControlInputSource::NONE;
}

#endif  // CONTROL_INPUT_SELECTOR_HPP
