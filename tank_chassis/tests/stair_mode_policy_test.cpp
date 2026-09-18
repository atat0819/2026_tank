#include <cassert>
#include <cstdint>

#include "fsm/stair_mode_policy.hpp"

namespace {

void expect_safe_policy(uint8_t s1, uint8_t s2, bool keyboard_online,
                        bool expected_front_command) {
  const StairModePolicy policy =
      EvaluateStairModePolicy(s1, s2, true, keyboard_online);
  assert(!policy.zero_all_torque);
  assert(policy.front_hold_enabled);
  assert(policy.rear_attitude_enabled);
  assert(policy.front_stair_command_enabled == expected_front_command);
}

void test_all_valid_switch_pairs() {
  for (uint8_t s1 = 1; s1 <= 3; ++s1) {
    for (uint8_t s2 = 1; s2 <= 3; ++s2) {
      const bool is_double_down = s1 == 2 && s2 == 2;
      const bool is_double_middle = s1 == 3 && s2 == 3;
      const StairModePolicy policy =
          EvaluateStairModePolicy(s1, s2, true, true);
      assert(policy.zero_all_torque == is_double_down);
      assert(policy.front_hold_enabled != is_double_down);
      assert(policy.rear_attitude_enabled != is_double_down);
      assert(policy.front_stair_command_enabled == is_double_middle);
    }
  }
}

void test_offline_link_forces_zero_torque() {
  const StairModePolicy policy = EvaluateStairModePolicy(3, 3, false, true);
  assert(policy.zero_all_torque);
  assert(!policy.front_hold_enabled);
  assert(!policy.rear_attitude_enabled);
  assert(!policy.front_stair_command_enabled);
}

void test_keyboard_offline_disables_double_middle_command() {
  expect_safe_policy(3, 3, false, false);
}

void test_invalid_switch_forces_zero_torque() {
  const StairModePolicy policy = EvaluateStairModePolicy(0, 1, true, true);
  assert(policy.zero_all_torque);
  assert(!policy.front_hold_enabled);
  assert(!policy.rear_attitude_enabled);
  assert(!policy.front_stair_command_enabled);
}

void test_invalid_switch_values_force_zero_torque() {
  const uint8_t invalid_values[] = {0, 4};
  for (uint8_t invalid : invalid_values) {
    const StairModePolicy invalid_s1 =
        EvaluateStairModePolicy(invalid, UP, true, true);
    assert(invalid_s1.zero_all_torque);
    assert(!invalid_s1.front_hold_enabled);
    assert(!invalid_s1.rear_attitude_enabled);
    assert(!invalid_s1.front_stair_command_enabled);

    const StairModePolicy invalid_s2 =
        EvaluateStairModePolicy(UP, invalid, true, true);
    assert(invalid_s2.zero_all_torque);
    assert(!invalid_s2.front_hold_enabled);
    assert(!invalid_s2.rear_attitude_enabled);
    assert(!invalid_s2.front_stair_command_enabled);
  }
}

}  // namespace

int main() {
  test_all_valid_switch_pairs();
  test_offline_link_forces_zero_torque();
  test_keyboard_offline_disables_double_middle_command();
  test_invalid_switch_forces_zero_torque();
  test_invalid_switch_values_force_zero_torque();
  return 0;
}
