#ifndef STAIR_MODE_POLICY_HPP
#define STAIR_MODE_POLICY_HPP

#include <cstdint>

static constexpr uint8_t UP = 1;
static constexpr uint8_t DOWN = 2;
static constexpr uint8_t MIDDLE = 3;

struct StairModePolicy {
  bool zero_all_torque;
  bool front_hold_enabled;
  bool rear_attitude_enabled;
  bool front_stair_command_enabled;
};

inline StairModePolicy EvaluateStairModePolicy(uint8_t s1, uint8_t s2,
                                               bool control_link_online,
                                               bool keyboard_online) {
  const bool valid_switches = s1 >= UP && s1 <= MIDDLE && s2 >= UP &&
                              s2 <= MIDDLE;
  const bool unsafe = !control_link_online || !valid_switches ||
                      (s1 == DOWN && s2 == DOWN);
  if (unsafe) {
    return {true, false, false, false};
  }

  return {false, true, true,
          s1 == MIDDLE && s2 == MIDDLE && keyboard_online};
}

#endif  // STAIR_MODE_POLICY_HPP
