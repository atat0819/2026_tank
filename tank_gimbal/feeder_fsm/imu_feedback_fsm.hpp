#ifndef IMU_FEEDBACK_FSM_HPP
#define IMU_FEEDBACK_FSM_HPP

#include <cmath>
#include <stdint.h>

// 只判断 IMU 数据健康和反馈来源，不计算 PID、不发送电机指令。
class ImuFeedbackFSM
{
public:
    static constexpr float MAX_JUMP_RAD = 45.0f * 3.14159265358979323846f / 180.0f;
    static constexpr uint32_t RECOVERY_STABLE_MS = 100U;

    struct Sample
    {
        float yaw_rad;
        float pitch_rad;
        float yaw_speed_rad_s;
        float pitch_speed_rad_s;
    };

    // 返回 true 表示本周期切换来源，任务需重新锚定目标并复位控制器。
    bool Update(bool online, const Sample &sample, uint32_t now_tick)
    {
        const bool was_using_encoder = using_encoder_;
        bool healthy = online && std::isfinite(sample.yaw_rad) &&
            std::isfinite(sample.pitch_rad) && std::isfinite(sample.yaw_speed_rad_s) &&
            std::isfinite(sample.pitch_speed_rad_s);
        if (healthy)
        {
            if (has_previous_)
            {
                // Yaw 越过 +/-pi 时取最短角差；Pitch 直接比较姿态角差。
                const float yaw_delta = std::remainder(sample.yaw_rad - previous_yaw_rad_,
                                                       2.0f * 3.14159265358979323846f);
                const float pitch_delta = sample.pitch_rad - previous_pitch_rad_;
                healthy = std::isfinite(yaw_delta) && std::isfinite(pitch_delta) &&
                    std::fabs(yaw_delta) <= MAX_JUMP_RAD &&
                    std::fabs(pitch_delta) <= MAX_JUMP_RAD;
            }
            // 异常跳变后从新姿态重新观察，持续健康 100 ms 才允许恢复。
            previous_yaw_rad_ = sample.yaw_rad;
            previous_pitch_rad_ = sample.pitch_rad;
            has_previous_ = true;
        }
        else
        {
            has_previous_ = false;
        }

        if (!healthy)
        {
            using_encoder_ = true;
            recovery_pending_ = false;
        }
        else if (using_encoder_)
        {
            if (!recovery_pending_)
            {
                recovery_pending_ = true;
                healthy_since_tick_ = now_tick;
            }
            if (now_tick - healthy_since_tick_ >= RECOVERY_STABLE_MS)
            {
                using_encoder_ = false;
                recovery_pending_ = false;
            }
        }
        return using_encoder_ != was_using_encoder;
    }

    bool UsingEncoder() const { return using_encoder_; }

private:
    bool using_encoder_ = true; // 启动时也先验证 IMU 连续健康，不使用未验证数据。
    bool has_previous_ = false;
    bool recovery_pending_ = false;
    uint32_t healthy_since_tick_ = 0U;
    float previous_yaw_rad_ = 0.0f;
    float previous_pitch_rad_ = 0.0f;
};

#endif
