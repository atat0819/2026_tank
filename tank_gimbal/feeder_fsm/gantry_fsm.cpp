#include "gantry_fsm.hpp"
#include <algorithm>
#include <cmath>

bool Class_Gantry_FSM::ConfigValid()
{
    using namespace GantryTuning;
    if (!CALIBRATED || !std::isfinite(TRAVEL_MARGIN) ||
        TRAVEL_MARGIN <= 0 || TRAVEL_MARGIN >= 1 ||
        !std::isfinite(LIFT_MISMATCH) || LIFT_MISMATCH <= 0 || LIFT_MISMATCH >= 1)
        return false;
    const double first[2] = {LIFT_ID5_FIRST_DEG - LIFT_ID5_HOME_DEG,
                             LIFT_ID6_FIRST_DEG - LIFT_ID6_HOME_DEG};
    const double second[2] = {LIFT_ID5_SECOND_DEG - LIFT_ID5_FIRST_DEG,
                              LIFT_ID6_SECOND_DEG - LIFT_ID6_FIRST_DEG};
    for (unsigned i = 0; i < 2; ++i)
    {
        if (!std::isfinite(first[i]) || !std::isfinite(second[i]) ||
            std::abs(first[i]) < 1e-3 || std::abs(second[i]) < 1e-3 ||
            ((first[i] > 0) != (second[i] > 0)))
            return false;
    }
    const double travel = DM_EXTEND_DEG - DM_RETRACT_DEG;
    if (!std::isfinite(travel) || std::abs(travel) < 1e-3)
        return false;
    const float speed_rad_s[3] = {LIFT_ID5_SPEED_RAD_S, LIFT_ID6_SPEED_RAD_S, DM_SPEED_RAD_S};
    for (float speed : speed_rad_s)
        if (!std::isfinite(speed) || speed <= 0)
            return false;
    const uint32_t timeout_ms[5] = {LIFT_FIRST_TIMEOUT_MS, EXTEND_TIMEOUT_MS,
                                   LIFT_SECOND_TIMEOUT_MS, RETRACT_TIMEOUT_MS, LOWER_TIMEOUT_MS};
    for (uint32_t timeout : timeout_ms)
        if (timeout == 0 || timeout > 0x7FFFFFFFU)
            return false;
    return true;
}

bool Class_Gantry_FSM::Running() const
{
    return state_ >= GantryState::LiftFirst && state_ <= GantryState::Lower;
}

void Class_Gantry_FSM::Fail(GantryFault fault)
{
    fault_ = fault;
    state_ = GantryState::Fault;
    holding_ = active_ = false;
    start_armed_ = false;
    progress_ = 0;
    for (unsigned i = 0; i < 3; ++i)
    {
        moving_[i] = false;
        target_speed_rad_s_[i] = 0;
    }
}

void Class_Gantry_FSM::Enter(GantryState state, uint32_t now_ms,
                             const double (&angle_deg)[3], const bool (&limit_pressed)[5])
{
    state_ = state;
    entered_ms_ = now_ms;
    scale_ = 1;
    for (unsigned i = 0; i < 3; ++i)
    {
        start_deg_[i] = angle_deg[i];
        hold_deg_[i] = angle_deg[i];
    }
    // 新动作的终点开关应先处于松开状态，防止粘连或接线错误直接跳步。
    if (Running() && limit_pressed[static_cast<unsigned>(state_) -
                              static_cast<unsigned>(GantryState::LiftFirst)])
        Fail(GantryFault::SwitchSequence);
}

void Class_Gantry_FSM::Update(bool control_enabled, bool feedback_ok,
                              bool right_raw, bool right_pressed,
                              bool lift_first_limit, bool extend_limit, bool lift_second_limit,
                              bool retract_limit, bool lower_limit,
                              double lift_id5_angle_deg, double lift_id6_angle_deg, double dm_angle_deg,
                              uint32_t now_ms)
{
    using namespace GantryTuning;
    const double angle_deg[3] = {lift_id5_angle_deg, lift_id6_angle_deg, dm_angle_deg};
    const bool limit_pressed[5] = {lift_first_limit, extend_limit, lift_second_limit,
                                   retract_limit, lower_limit};
    active_ = false;
    progress_ = 0;
    for (unsigned i = 0; i < 3; ++i)
    {
        moving_[i] = false;
        target_speed_rad_s_[i] = 0;
    }
    if (!control_enabled || !ConfigValid())
    {
        state_ = GantryState::Disabled;
        fault_ = GantryFault::None;
        holding_ = start_armed_ = false;
        return;
    }
    if (state_ == GantryState::Fault)
        return; // 故障恢复：先退出键鼠模式，再回原点、进入模式并重新点击
    bool valid = feedback_ok;
    for (double angle : angle_deg)
        valid = valid && std::isfinite(angle);
    if (!valid)
    {
        if (Running() || holding_)
            Fail(GantryFault::Feedback);
        else
        {
            state_ = GantryState::Disabled;
            start_armed_ = false;
        }
        return;
    }
    if (state_ == GantryState::Disabled)
        Enter(GantryState::Idle, now_ms, angle_deg, limit_pressed);

    if (state_ == GantryState::Idle)
    {
        if (!right_raw && !right_pressed)
            start_armed_ = true;
        if (start_armed_ && right_raw && right_pressed)
        {
            start_armed_ = false;
            if (!limit_pressed[3] || !limit_pressed[4])
                Fail(GantryFault::NotHome);
            else
            {
                holding_ = true;
                Enter(GantryState::LiftFirst, now_ms, angle_deg, limit_pressed);
            }
        }
    }
    else if (Running())
    {
        start_armed_ = false; // 忙时的点击不排队，完成后必须重新松开、按下
        const unsigned step = static_cast<unsigned>(state_) -
                              static_cast<unsigned>(GantryState::LiftFirst);
        const uint32_t timeout_ms[5] = {LIFT_FIRST_TIMEOUT_MS, EXTEND_TIMEOUT_MS,
                                       LIFT_SECOND_TIMEOUT_MS, RETRACT_TIMEOUT_MS, LOWER_TIMEOUT_MS};
        if (limit_pressed[step])
        {
            // 开关决定结束；累计角度只决定减速与故障保护。
            Enter(state_ == GantryState::Lower ? GantryState::Idle :
                  static_cast<GantryState>(static_cast<unsigned>(state_) + 1),
                  now_ms, angle_deg, limit_pressed);
        }
        else if (static_cast<uint32_t>(now_ms - entered_ms_) >= timeout_ms[step])
            Fail(GantryFault::Timeout);
    }
    if (state_ == GantryState::Fault)
        return;

    active_ = holding_;
    if (!Running())
        return;

    const bool lift = state_ == GantryState::LiftFirst ||
                      state_ == GantryState::LiftSecond || state_ == GantryState::Lower;
    double distance[3] = {};
    const double home_deg[2] = {LIFT_ID5_HOME_DEG, LIFT_ID6_HOME_DEG};
    const double first_deg[2] = {LIFT_ID5_FIRST_DEG, LIFT_ID6_FIRST_DEG};
    const double second_deg[2] = {LIFT_ID5_SECOND_DEG, LIFT_ID6_SECOND_DEG};
    for (unsigned i = 0; i < 2; ++i)
    {
        if (state_ == GantryState::LiftFirst)
            distance[i] = first_deg[i] - home_deg[i];
        else if (state_ == GantryState::LiftSecond)
            distance[i] = second_deg[i] - first_deg[i];
        else if (state_ == GantryState::Lower)
            distance[i] = home_deg[i] - second_deg[i];
    }
    if (!lift)
        distance[2] = state_ == GantryState::Extend ? DM_EXTEND_DEG - DM_RETRACT_DEG :
                                                       DM_RETRACT_DEG - DM_EXTEND_DEG;
    float progress[3] = {};
    for (unsigned i = 0; i < 3; ++i)
    {
        moving_[i] = lift ? i < 2 : i == 2;
        if (!moving_[i])
            continue;
        progress[i] = static_cast<float>((angle_deg[i] - start_deg_[i]) / distance[i]);
        if (progress[i] < -TRAVEL_MARGIN || progress[i] > 1 + TRAVEL_MARGIN)
        {
            Fail(GantryFault::Travel);
            return;
        }
    }
    if (lift && std::abs(progress[0] - progress[1]) > LIFT_MISMATCH)
    {
        Fail(GantryFault::LiftMismatch);
        return;
    }
    progress_ = lift ? std::max(progress[0], progress[1]) : progress[2];
    // 两台升降电机一起切档；每段只降速不升速，避免边界抖动。
    if (state_ != GantryState::LiftSecond)
    {
        const float requested = progress_ >= .85f ? .1f : progress_ >= .5f ? .5f : 1;
        scale_ = std::min(scale_, requested);
    }
    const float speed_rad_s[3] = {LIFT_ID5_SPEED_RAD_S, LIFT_ID6_SPEED_RAD_S, DM_SPEED_RAD_S};
    for (unsigned i = 0; i < 3; ++i)
        if (moving_[i])
            target_speed_rad_s_[i] = (distance[i] > 0 ? 1 : -1) * speed_rad_s[i] * scale_;
}
