#ifndef MOTOR_RECOVERY_FSM_HPP
#define MOTOR_RECOVERY_FSM_HPP

#include <stdint.h>

// 电机没有反馈时，按时间间隔产生受限的 MIT 使能请求。
// 一旦收到在线反馈，就结束重试周期，避免健康电机被重复发送 On。
class MotorRecoveryFSM
{
public:
    static constexpr uint32_t ENABLE_RETRY_MS = 100U;

    void Init(uint32_t now_tick)
    {
        last_enable_tick_ = now_tick;
        enable_requested_ = false;
        was_online_ = false;
    }

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
    uint32_t last_enable_tick_ = 0U;
    bool enable_requested_ = false;
    bool was_online_ = false;
};

#endif // MOTOR_RECOVERY_FSM_HPP
