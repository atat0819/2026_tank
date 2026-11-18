#include "gantry_fsm.hpp"
#include <algorithm>
#include <cmath>

bool Class_Gantry_FSM::ConfigValid()
{
    using namespace GantryTuning;
    if (!CALIBRATED || !std::isfinite(TRAVEL_MARGIN) ||
        TRAVEL_MARGIN <= 0 || TRAVEL_MARGIN >= 1 ||
        !std::isfinite(LIFT_MISMATCH) || LIFT_MISMATCH <= 0 || LIFT_MISMATCH >= 1 ||
        !std::isfinite(LIFT_SYNC_KP) || LIFT_SYNC_KP < 0 ||
        !std::isfinite(LIFT_SYNC_MAX_SPEED_RATIO) ||
        LIFT_SYNC_MAX_SPEED_RATIO < 0 || LIFT_SYNC_MAX_SPEED_RATIO > 1 ||
        !std::isfinite(HOME_STOP_SPEED_RAD_S) || HOME_STOP_SPEED_RAD_S <= 0 ||
        !std::isfinite(LIFT_HOME_TOLERANCE_DEG) || LIFT_HOME_TOLERANCE_DEG <= 0)
        return false;
    const float first[2] = {LIFT_ID5_FIRST_DEG - LIFT_ID5_HOME_DEG,
                             LIFT_ID6_FIRST_DEG - LIFT_ID6_HOME_DEG};
    const float second[2] = {LIFT_ID5_SECOND_DEG - LIFT_ID5_FIRST_DEG,
                              LIFT_ID6_SECOND_DEG - LIFT_ID6_FIRST_DEG};
    for (unsigned i = 0; i < 2; ++i)
    {
        if (!std::isfinite(first[i]) || !std::isfinite(second[i]) ||
            std::abs(first[i]) < 1e-3f || std::abs(second[i]) < 1e-3f ||
            ((first[i] > 0) != (second[i] > 0)))
            return false;
    }
    const float travel = DM_EXTEND_DEG - DM_RETRACT_DEG;
    if (!std::isfinite(travel) || std::abs(travel) < 1e-3f)
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
    gravity_unloading_ = false;
    gravity_load_factor_ = 0;
    start_armed_ = false;
    start_hold_pending_ = false;
    progress_ = 0;
    for (unsigned i = 0; i < 3; ++i)
    {
        moving_[i] = false;
        target_speed_rad_s_[i] = 0;
    }
}

void Class_Gantry_FSM::Enter(GantryState state, uint32_t now_ms,
                             const float (&angle_deg)[3], const bool (&limit_pressed)[5])
{
    state_ = state;
    entered_ms_ = now_ms;
    scale_ = 1;
    progress_ = 0;
    gravity_unloading_ = false;
    gravity_load_factor_ = (state == GantryState::LiftSecond || state == GantryState::Retract ||
                            state == GantryState::Lower) ? 1.0f : 0.0f;
    for (unsigned i = 0; i < 3; ++i)
    {
        start_deg_[i] = angle_deg[i];
        hold_deg_[i] = angle_deg[i];
        moving_[i] = false;
        target_speed_rad_s_[i] = 0;
    }
    // 新动作的终点开关应先处于松开状态，防止粘连或接线错误直接跳步。
    if (Running() && limit_pressed[static_cast<unsigned>(state_) -
                              static_cast<unsigned>(GantryState::LiftFirst)])
        Fail(GantryFault::SwitchSequence);
}

void Class_Gantry_FSM::Update(bool control_enabled, bool feedback_ok, bool lift_feedback_ok,
                              bool start_key_raw, bool start_key_pressed,
                              bool lift_first_limit, bool extend_limit, bool lift_second_limit,
                              bool retract_limit, bool lower_limit,
                              float lift_id5_angle_deg, float lift_id6_angle_deg, float dm_angle_deg,
                              float lift_id5_current_a, float lift_id6_current_a,
                              float lift_id5_speed_rad_s, float lift_id6_speed_rad_s,
                              uint32_t now_ms)
{
    using namespace GantryTuning;
    const float angle_deg[3] = {lift_id5_angle_deg, lift_id6_angle_deg, dm_angle_deg};
    const bool limit_pressed[5] = {lift_first_limit, extend_limit, lift_second_limit, retract_limit, lower_limit};
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
    const bool stopped = std::abs(speed_rad_s[0]) <= HOME_STOP_SPEED_RAD_S &&
                         std::abs(speed_rad_s[1]) <= HOME_STOP_SPEED_RAD_S;
    // 整车通电前已手动压到底并齐平；首次有效低速快照是本次上电的原点参考，不依赖每次编码器读数相同。
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
        holding_ = start_armed_ = start_hold_pending_ = false;
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
            start_hold_pending_ = false;
        }
        return;
    }
    if (state_ == GantryState::Disabled)
        Enter(GantryState::Idle, now_ms, angle_deg, limit_pressed);

    if (state_ == GantryState::Idle)
    {
        if (!start_key_raw && !start_key_pressed)
        {
            start_armed_ = true;
            start_hold_pending_ = false;
        }
        else if (!start_key_raw)
        {
            // 原始启动键松开立即取消计时，确认松开后才能重新计时。
            start_armed_ = false;
            start_hold_pending_ = false;
        }
        if (start_armed_ && start_key_raw && start_key_pressed)
        {
            if (!start_hold_pending_)
            {
                start_hold_pending_ = true;
                start_hold_since_ms_ = now_ms;
            }
            else if (static_cast<uint32_t>(now_ms - start_hold_since_ms_) >= START_HOLD_MS)
            {
                start_armed_ = start_hold_pending_ = false;
                if (!at_home)
                    Fail(GantryFault::NotHome);
                else
                {
                    holding_ = true;
                    Enter(GantryState::LiftFirst, now_ms, angle_deg, limit_pressed);
                }
            }
        }
    }
    else if (Running())
    {
        start_armed_ = false; // 忙时的点击不排队，完成后必须重新松开、按下
        start_hold_pending_ = false;
        const unsigned step = static_cast<unsigned>(state_) -
                              static_cast<unsigned>(GantryState::LiftFirst);
        const uint32_t timeout_ms[5] = {LIFT_FIRST_TIMEOUT_MS, EXTEND_TIMEOUT_MS,
                                       LIFT_SECOND_TIMEOUT_MS, RETRACT_TIMEOUT_MS, LOWER_TIMEOUT_MS};
        if (limit_pressed[step])
        {
            // 五段动作均由对应限位结束，下降到底后回到 Idle。
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
    // 保留本次上电的共同对齐基准；Enter() 的分段起点不参与同步误差。
    const float up_direction[2] = {
        LIFT_ID5_FIRST_DEG > LIFT_ID5_HOME_DEG ? 1.0f : -1.0f,
        LIFT_ID6_FIRST_DEG > LIFT_ID6_HOME_DEG ? 1.0f : -1.0f
    };
    lift_sync_error_deg_ =
        up_direction[0] * (angle_deg[0] - home_deg_[0]) -
        up_direction[1] * (angle_deg[1] - home_deg_[1]);
    if (!Running())
        return;

    const bool lift = state_ == GantryState::LiftFirst ||
                      state_ == GantryState::LiftSecond || state_ == GantryState::Lower;
    float distance[3] = {};
    const float home_deg[2] = {LIFT_ID5_HOME_DEG, LIFT_ID6_HOME_DEG};
    const float first_deg[2] = {LIFT_ID5_FIRST_DEG, LIFT_ID6_FIRST_DEG};
    const float second_deg[2] = {LIFT_ID5_SECOND_DEG, LIFT_ID6_SECOND_DEG};
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
        progress[i] = (angle_deg[i] - start_deg_[i]) / distance[i];
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
        if (!gravity_unloading_ && std::min(progress[0], progress[1]) >= LOWER_UNLOAD_PROGRESS)
        {
            gravity_unloading_ = true;
            gravity_unload_since_ms_ = now_ms;
        }
        if (gravity_unloading_)
            gravity_load_factor_ = 1.0f - std::min(1.0f,
                static_cast<float>(static_cast<uint32_t>(now_ms - gravity_unload_since_ms_)) /
                LOWER_UNLOAD_BLEND_MS);
    }
    // 第一次抬升和 DM 伸缩按行程减速；第二次抬升和下降不分档，但升降仍受同步纠偏影响。
    if (state_ != GantryState::LiftSecond && state_ != GantryState::Lower)
    {
        // 可调减速点：.5/.85 是本段已走行程的 50%/85%；.5/.1 是对应 50%/10% 速度比例。
        // 提前减速（降低进度阈值）或降低速度比例可减轻触碰冲击，但会延长动作时间，需同时核对超时。
        const float requested = progress_ >= .85f ? .1f : progress_ >= .5f ? .5f : 1;
        scale_ = std::min(scale_, requested);
    }
    if (lift)
    {
        // 公共升降速度取两侧配置上限的较小值；先在向上为正的坐标中纠偏。
        const float speed_limit = std::min(LIFT_ID5_SPEED_RAD_S, LIFT_ID6_SPEED_RAD_S) * scale_;
        const float correction_limit = speed_limit * LIFT_SYNC_MAX_SPEED_RATIO;
        const float correction = std::clamp(
            LIFT_SYNC_KP * lift_sync_error_deg_ * (3.14159265358979323846f / 180.0f),
            -correction_limit, correction_limit);
        const float base_speed = state_ == GantryState::Lower ? -speed_limit : speed_limit;
        // 即使落后侧已饱和，领先侧仍会减速；修正最多使本侧停住，不会反向。
        target_speed_rad_s_[0] = up_direction[0] *
            std::clamp(base_speed - correction, -speed_limit, speed_limit);
        target_speed_rad_s_[1] = up_direction[1] *
            std::clamp(base_speed + correction, -speed_limit, speed_limit);
    }
    else
    {
        target_speed_rad_s_[2] = (distance[2] > 0 ? 1 : -1) * DM_SPEED_RAD_S * scale_;
    }
}
