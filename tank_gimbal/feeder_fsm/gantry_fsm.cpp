#include "gantry_fsm.hpp"
#include <algorithm>
#include <cmath>

bool Class_Gantry_FSM::ConfigValid()
{
    using namespace GantryTuning;
    if (!CALIBRATED || !std::isfinite(TRAVEL_MARGIN) ||
        TRAVEL_MARGIN <= 0 || TRAVEL_MARGIN >= 1 ||
        !std::isfinite(LIFT_MISMATCH) || LIFT_MISMATCH <= 0 || LIFT_MISMATCH >= 1 ||
        !std::isfinite(LOWER_CURRENT_LIMIT_A) || LOWER_CURRENT_LIMIT_A <= 0 || LOWER_CURRENT_LIMIT_A > 20 ||
        !std::isfinite(LOWER_STALL_CURRENT_A) || LOWER_STALL_CURRENT_A <= 0 ||
        LOWER_STALL_CURRENT_A >= LOWER_CURRENT_LIMIT_A ||
        !std::isfinite(LOWER_STALL_SPEED_RAD_S) || LOWER_STALL_SPEED_RAD_S <= 0 ||
        LOWER_STALL_SPEED_RAD_S >= std::min(LIFT_ID5_SPEED_RAD_S, LIFT_ID6_SPEED_RAD_S) ||
        LOWER_STALL_CONFIRM_MS == 0 || LOWER_STALL_CONFIRM_MS >= LOWER_TIMEOUT_MS ||
        !std::isfinite(LIFT_HOME_TOLERANCE_DEG) || LIFT_HOME_TOLERANCE_DEG <= 0)
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
    stall_pending_ = false;
    start_armed_ = false;
    progress_ = 0;
    for (unsigned i = 0; i < 3; ++i)
    {
        moving_[i] = false;
        target_speed_rad_s_[i] = 0;
    }
}

void Class_Gantry_FSM::Enter(GantryState state, uint32_t now_ms,
                             const double (&angle_deg)[3], const bool (&limit_pressed)[4])
{
    state_ = state;
    entered_ms_ = now_ms;
    scale_ = 1;
    progress_ = 0;
    stall_pending_ = false;
    for (unsigned i = 0; i < 3; ++i)
    {
        start_deg_[i] = angle_deg[i];
        hold_deg_[i] = angle_deg[i];
        moving_[i] = false;
        target_speed_rad_s_[i] = 0;
    }
    // 新动作的终点开关应先处于松开状态，防止粘连或接线错误直接跳步。
    if (Running() && state_ != GantryState::Lower && limit_pressed[static_cast<unsigned>(state_) -
                              static_cast<unsigned>(GantryState::LiftFirst)])
        Fail(GantryFault::SwitchSequence);
}

void Class_Gantry_FSM::Update(bool control_enabled, bool feedback_ok, bool lift_feedback_ok,
                              bool right_raw, bool right_pressed,
                              bool lift_first_limit, bool extend_limit, bool lift_second_limit,
                              bool retract_limit,
                              double lift_id5_angle_deg, double lift_id6_angle_deg, double dm_angle_deg,
                              float lift_id5_current_a, float lift_id6_current_a,
                              float lift_id5_speed_rad_s, float lift_id6_speed_rad_s,
                              uint32_t now_ms)
{
    using namespace GantryTuning;
    const double angle_deg[3] = {lift_id5_angle_deg, lift_id6_angle_deg, dm_angle_deg};
    const bool limit_pressed[4] = {lift_first_limit, extend_limit, lift_second_limit, retract_limit};
    const float current_a[2] = {lift_id5_current_a, lift_id6_current_a};
    const float speed_rad_s[2] = {lift_id5_speed_rad_s, lift_id6_speed_rad_s};
    active_ = false;
    progress_ = 0;
    for (unsigned i = 0; i < 3; ++i)
    {
        moving_[i] = false;
        target_speed_rad_s_[i] = 0;
    }
    bool lift_valid = lift_feedback_ok;
    for (unsigned i = 0; i < 2; ++i)
        lift_valid = lift_valid && std::isfinite(angle_deg[i]) &&
                     std::isfinite(current_a[i]) && std::isfinite(speed_rad_s[i]);
    const bool valid = feedback_ok && lift_valid && std::isfinite(angle_deg[2]);
    const bool stopped = std::abs(speed_rad_s[0]) <= LOWER_STALL_SPEED_RAD_S &&
                         std::abs(speed_rad_s[1]) <= LOWER_STALL_SPEED_RAD_S;
    // 机构启动时已靠重力落底；首次有效低速快照是本次上电的原点参考。
    // 安全门关闭时也记录，后续切模式或故障恢复不重新定义原点。
    if (!home_recorded_ && lift_valid && stopped)
    {
        home_deg_[0] = angle_deg[0];
        home_deg_[1] = angle_deg[1];
        home_recorded_ = true;
    }
    const bool at_home = home_recorded_ && stopped &&
        std::abs(angle_deg[0] - home_deg_[0]) <= LIFT_HOME_TOLERANCE_DEG &&
        std::abs(angle_deg[1] - home_deg_[1]) <= LIFT_HOME_TOLERANCE_DEG;
    if (!control_enabled || !ConfigValid())
    {
        state_ = GantryState::Disabled;
        fault_ = GantryFault::None;
        holding_ = start_armed_ = false;
        stall_pending_ = false;
        return;
    }
    if (state_ == GantryState::Fault)
        return; // 故障恢复：先退出键鼠模式，再回原点、进入模式并重新点击
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
            if (!limit_pressed[3] || !at_home)
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
        if (state_ != GantryState::Lower && limit_pressed[step])
        {
            // 前四段由开关决定结束，最后下降由电流、低速和原点位置共同确认。
            Enter(static_cast<GantryState>(static_cast<unsigned>(state_) + 1),
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
    if (state_ == GantryState::Lower)
    {
        const bool stalled = stopped &&
            std::max(std::abs(current_a[0]), std::abs(current_a[1])) >= LOWER_STALL_CURRENT_A;
        if (!stalled)
            stall_pending_ = false;
        else if (!stall_pending_)
        {
            stall_since_ms_ = now_ms;
            stall_pending_ = true;
        }
        else if (static_cast<uint32_t>(now_ms - stall_since_ms_) >= LOWER_STALL_CONFIRM_MS)
        {
            if (at_home)
                Enter(GantryState::Idle, now_ms, angle_deg, limit_pressed);
            else
                Fail(GantryFault::Stall); // 中途堵住不能被当作回到原点。
            return;
        }
    }
    // 第一次抬升和 DM 伸缩按行程减速；第二次抬升和下降全程使用配置速度。
    if (state_ != GantryState::LiftSecond && state_ != GantryState::Lower)
    {
        const float requested = progress_ >= .85f ? .1f : progress_ >= .5f ? .5f : 1;
        scale_ = std::min(scale_, requested);
    }
    const float target_speed_rad_s[3] = {LIFT_ID5_SPEED_RAD_S, LIFT_ID6_SPEED_RAD_S, DM_SPEED_RAD_S};
    for (unsigned i = 0; i < 3; ++i)
        if (moving_[i])
            target_speed_rad_s_[i] = (distance[i] > 0 ? 1 : -1) * target_speed_rad_s[i] * scale_;
}
