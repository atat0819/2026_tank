#ifndef STAIR_MODE_POLICY_HPP
#define STAIR_MODE_POLICY_HPP

#include <cstdint>

static constexpr uint8_t UP = 1;
static constexpr uint8_t DOWN = 2;
static constexpr uint8_t MIDDLE = 3;

struct StairModePolicy {
  bool zero_all_torque;              // 双下、失联或非法档位：四电机零力矩
  bool front_hold_enabled;           // 是否允许前 4310 保持初始/目标位置
  bool rear_attitude_enabled;        // 是否允许后 6248 运行姿态控制
  bool front_stair_command_enabled;  // 仅双中且键盘在线时允许 B 动作
};

inline StairModePolicy EvaluateStairModePolicy(uint8_t s1, uint8_t s2,
                                               bool control_link_online,
                                               bool keyboard_online) {
  // 将遥控档位和通信状态转换为上台阶任务使用的统一策略。
  // 返回值不直接发送电机命令，只决定哪些控制模块可以运行。
  // 先统一判断档位和链路，再由任务执行安全分支，避免各处重复判断。
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
