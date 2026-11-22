#include "yaw_fsm.hpp"
#include <algorithm>
#include <cmath>

void Yaw_FSM::Reset()
{
    mode_ = Mode::Stop;
    target_angle_ = 0.0f;
}

void Yaw_FSM::Update(const Struct_Yaw_Input &input, float current_angle, uint32_t now_tick)
{
    // 限制单次回中推进使用的时间，避免调度间隔过长造成目标突跳。
    const uint32_t elapsed_ms = std::min(now_tick - last_tick_, uint32_t{20U});
    last_tick_ = now_tick;
    if (!input.enabled || !Is_Feedback_In_Travel(current_angle))
    {
        Reset();
        return;
    }

    // 鼠标接管时锚定当前角，然后累加本周期位移。
    if (input.is_keymouse && input.mouse_active)
    {
        if (mode_ != Mode::Mouse || input.reanchor)
            target_angle_ = current_angle;
        mode_ = Mode::Mouse;
        target_angle_ += static_cast<float>(input.mouse_delta_x) * kMouseRadPerCount;
    }
    // 正常输入下从停止或鼠标控制进入回中时，以当前角为起点。
    else if (mode_ != Mode::Center)
    {
        mode_ = Mode::Center;
        target_angle_ = current_angle;
    }
    else
    {
        const float step = CENTER_SPEED_RAD_S * static_cast<float>(elapsed_ms) / 1000.0f;
        target_angle_ += std::clamp(kCenterAngleRad - target_angle_, -step, step);
    }
    target_angle_ = std::clamp(target_angle_, kMinAngleRad, kMaxAngleRad);
}

bool Yaw_FSM::Is_Feedback_In_Travel(float current_angle)
{
    return current_angle >= (kLeftEndDeg - kFeedbackToleranceDeg) * kDegToRad &&
           current_angle <= (kRightEndDeg + kFeedbackToleranceDeg) * kDegToRad;
}

bool Yaw_FSM::Is_Soft_Limit_Active(float current_angle)
{
    return current_angle <= kMinAngleRad || current_angle >= kMaxAngleRad;
}

float Yaw_FSM::Limit_Torque(float torque_nm, float current_angle) const
{
    // 无目标、反馈越界或力矩无效时输出零；软限位处阻止继续向外的力矩。
    if (!Is_Target_Valid() || !Is_Feedback_In_Travel(current_angle) || !std::isfinite(torque_nm) ||
        (current_angle <= kMinAngleRad && torque_nm < 0.0f) ||
        (current_angle >= kMaxAngleRad && torque_nm > 0.0f))
        return 0.0f;
    return std::clamp(torque_nm, -kMaxTorqueNm, kMaxTorqueNm);
}
