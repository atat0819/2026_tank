#ifndef SECOND_ORDER_SETPOINT_PLANNER_HPP
#define SECOND_ORDER_SETPOINT_PLANNER_HPP

#include <algorithm>
#include <cmath>

namespace Alg::Utility
{
    // 基于 HERO（哈工大威海，影丨）二阶设定值跟随模型的速度域适配。
    // 来源 https://bbs.robomaster.com/wiki/36827278/1647282?source=4 ，CC BY-NC-SA 4.0。
    // 原模型的位置/速度/加速度在此对应车速/加速度/jerk；解析计算三段轨迹，
    // 修正零分母、反向和到达目标边界。加速度/jerk/周期参数均须为正。
    class SecondOrderSetpointPlanner
    {
    public:
        SecondOrderSetpointPlanner(float acceleration_limit, float jerk_limit, float period)
            : acceleration_limit_(acceleration_limit), jerk_limit_(jerk_limit), period_(period) {}

        void Update(float target)
        {
            const float previous_velocity = velocity_;
            const float distance = target - velocity_;
            // 先将当前加速度收至 0 所必需的速度增量，决定先加 jerk 的方向。
            const float stopping_distance = acceleration_ * std::fabs(acceleration_) / (2.0f * jerk_limit_);
            const float direction = distance >= stopping_distance ? 1.0f : -1.0f;
            float rate = direction * acceleration_;
            const float delta = direction * distance;
            const float peak = std::fmin(acceleration_limit_,
                std::sqrt(std::fmax(0.0f, jerk_limit_ * delta + 0.5f * rate * rate)));
            const float rise_time = std::fmax(0.0f, (peak - rate) / jerk_limit_);
            const float hold_time = peak > 0.0f ? std::fmax(0.0f,
                (delta - (2.0f * peak * peak - rate * rate) / (2.0f * jerk_limit_)) / peak) : 0.0f;
            const float fall_time = peak / jerk_limit_;
            if (rise_time + hold_time + fall_time <= period_)
            {
                velocity_ = target;
                acceleration_ = 0.0f;
            }
            else
            {
                float remaining = period_;
                const float times[3] = {rise_time, hold_time, fall_time};
                const float jerks[3] = {jerk_limit_, 0.0f, -jerk_limit_};
                for (unsigned phase = 0; phase < 3; ++phase)
                {
                    const float duration = std::fmin(remaining, times[phase]);
                    velocity_ += direction * (rate * duration + 0.5f * jerks[phase] * duration * duration);
                    rate += jerks[phase] * duration;
                    remaining -= duration;
                }
                acceleration_ = direction * rate;
            }
            // 前馈使用本周期平均加速度，使施加的惯性冲量对应实际规划速度增量。
            average_acceleration_ = (velocity_ - previous_velocity) / period_;
        }

        float GetOut() const { return velocity_; }
        float GetAcceleration() const { return average_acceleration_; }
        void Reset() { velocity_ = acceleration_ = average_acceleration_ = 0.0f; }

    private:
        float acceleration_limit_, jerk_limit_, period_;
        float velocity_ = 0.0f;
        float acceleration_ = 0.0f;
        float average_acceleration_ = 0.0f;
    };
}

#endif
