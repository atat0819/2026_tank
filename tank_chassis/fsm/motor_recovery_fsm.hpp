#ifndef MOTOR_RECOVERY_FSM_HPP
#define MOTOR_RECOVERY_FSM_HPP

#include <stdint.h>

// 电机没有反馈时，按时间间隔产生受限的 MIT 使能请求。
// 一旦收到在线反馈，就结束重试周期，避免健康电机被重复发送 On。
class MotorRecoveryFSM
{
public:
    // 电机无反馈时，两次自动 On 请求之间的最小间隔。
    static constexpr uint32_t ENABLE_RETRY_MS = 100U;

    // 设置恢复状态机的时间基准，并清除历史请求状态。
    void Init(uint32_t now_tick)
    {
        last_enable_tick_ = now_tick;
        enable_requested_ = false;
        was_online_ = false;
    }

    // 根据反馈在线状态决定当前周期是否需要发送一次 MIT On。
    // now_tick 使用无符号减法，天然支持系统 tick 回绕。
    bool Should_Enable(bool feedback_online, uint32_t now_tick)
    {
        if (feedback_online)
        {
            const bool recovered_online = !was_online_;
            was_online_ = true;
            enable_requested_ = false;
            if (recovered_online)
            {
                last_enable_tick_ = now_tick;
                return true;
            }
            return false;
        }

        was_online_ = false;
        if (!enable_requested_ ||
            (now_tick - last_enable_tick_ >= ENABLE_RETRY_MS))
        {
            last_enable_tick_ = now_tick;
            enable_requested_ = true;
            return true;
        }

        return false;
    }

private:
    uint32_t last_enable_tick_ = 0U; // 最近一次发送 On 请求的时间
    bool enable_requested_ = false;  // 当前离线周期是否已经请求过 On
    bool was_online_ = false;        // 上一周期是否检测到电机在线
};

#endif // MOTOR_RECOVERY_FSM_HPP
