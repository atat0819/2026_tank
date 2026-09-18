#include "up_stair_behind_motor_fsm.hpp"

#include <math.h>

namespace
{
// 用于角度标定和调试显示的圆周常量。
const float PI_RAD = 3.14159265358979323846f;
}

Class_Up_Stair_Behind_Motor_FSM::Config::Config()
    : pitch_zero_deg(0.0f),
      roll_zero_deg(0.0f),
      angle_start_rad{0.0f, 0.0f},
      angle_end_rad{0.0f, 0.0f},
      motor_direction{1, 1}
{
}

Class_Up_Stair_Behind_Motor_FSM::Class_Up_Stair_Behind_Motor_FSM()
    : config_(),
      config_valid_(false),
      motor_controllable_{false, false},
      motor_angle_rad_{0.0f, 0.0f},
      feedback_pitch_deg_(0.0f),
      feedback_roll_deg_(0.0f),
      pitch_rate_dps_(0.0f),
      roll_rate_dps_(0.0f),
      output_scale_(0.0f),
      recovery_start_tick_(0U),
      recovery_last_tick_(0U),
      recovery_planner_(0.0f, 0.0f),
      feedback_degraded_(false),
      feedback_valid_previous_{false, false}
{
    Class_FSM::Init(UP_STAIR_BEHIND_MOTOR_COUNT,
                    UP_STAIR_BEHIND_MOTOR_DISABLED);
    config_valid_ = Validate_Config();
}

Class_Up_Stair_Behind_Motor_FSM::Class_Up_Stair_Behind_Motor_FSM(
    const Config &config)
    : config_(config),
      config_valid_(false),
      motor_controllable_{false, false},
      motor_angle_rad_{0.0f, 0.0f},
      feedback_pitch_deg_(0.0f),
      feedback_roll_deg_(0.0f),
      pitch_rate_dps_(0.0f),
      roll_rate_dps_(0.0f),
      output_scale_(0.0f),
      recovery_start_tick_(0U),
      recovery_last_tick_(0U),
      recovery_planner_(0.0f, 0.0f),
      feedback_degraded_(false),
      feedback_valid_previous_{false, false}
{
    Class_FSM::Init(UP_STAIR_BEHIND_MOTOR_COUNT,
                    UP_STAIR_BEHIND_MOTOR_DISABLED);
    config_valid_ = Validate_Config();
}

void Class_Up_Stair_Behind_Motor_FSM::Reset()
{
    // 完整复位后部状态、反馈有效标志和恢复斜坡规划器。
    Class_FSM::Init(UP_STAIR_BEHIND_MOTOR_COUNT,
                    UP_STAIR_BEHIND_MOTOR_DISABLED);
    output_scale_ = 0.0f;
    recovery_start_tick_ = 0U;
    recovery_last_tick_ = 0U;
    recovery_planner_.SetNowReal(0.0f);
    recovery_planner_.SetTarget(0.0f);
    recovery_planner_.TIM_Calculate_PeriodElapsedCallback(0.0f, 0.0f);
    motor_controllable_[0] = false;
    motor_controllable_[1] = false;
    feedback_degraded_ = false;
    feedback_valid_previous_[0] = false;
    feedback_valid_previous_[1] = false;
}

uint8_t Class_Up_Stair_Behind_Motor_FSM::To_Index(uint8_t id) const
{
    return (id >= 1U && id <= 2U) ? static_cast<uint8_t>(id - 1U) : 2U;
}

bool Class_Up_Stair_Behind_Motor_FSM::Validate_Config() const
{
    // 检查零位、机械安全区间和电机正负方向；无效配置必须禁止出力。
    for (uint8_t index = 0U; index < 2U; ++index)
    {
        const float start = config_.angle_start_rad[index];
        const float end = config_.angle_end_rad[index];
        if (!std::isfinite(start) || !std::isfinite(end) ||
            start < 0.0f || start > TWO_PI_RAD || end < 0.0f ||
            end > TWO_PI_RAD || start == end ||
            (config_.motor_direction[index] != 1 &&
             config_.motor_direction[index] != -1))
        {
            return false;
        }
        if (start == TWO_PI_RAD && end == 0.0f)
        {
            return false;
        }
    }
    return std::isfinite(config_.pitch_zero_deg) &&
           std::isfinite(config_.roll_zero_deg);
}

float Class_Up_Stair_Behind_Motor_FSM::To_Unwrapped_Angle(
    uint8_t index, float raw_angle_rad) const
{
    if (config_.angle_start_rad[index] > config_.angle_end_rad[index] &&
        raw_angle_rad < config_.angle_start_rad[index])
    {
        return raw_angle_rad + TWO_PI_RAD;
    }
    return raw_angle_rad;
}

bool Class_Up_Stair_Behind_Motor_FSM::Is_Angle_Valid(
    uint8_t id, float raw_angle_rad) const
{
    // 编码器原始角度必须在 [0, 2π] 内，并落在允许的普通区间或跨零区间内。
    const uint8_t index = To_Index(id);
    if (index > 1U || !config_valid_ || !std::isfinite(raw_angle_rad) ||
        raw_angle_rad < 0.0f || raw_angle_rad > TWO_PI_RAD)
    {
        return false;
    }

    if (config_.angle_start_rad[index] < config_.angle_end_rad[index])
    {
        return raw_angle_rad >= config_.angle_start_rad[index] &&
               raw_angle_rad <= config_.angle_end_rad[index];
    }

    return raw_angle_rad >= config_.angle_start_rad[index] ||
           raw_angle_rad <= config_.angle_end_rad[index];
}

void Class_Up_Stair_Behind_Motor_FSM::Disable()
{
    // 任何控制链路、IMU 或配置故障都回到禁用态，并清零恢复规划器。
    Set_Status(UP_STAIR_BEHIND_MOTOR_DISABLED);
    output_scale_ = 0.0f;
    recovery_start_tick_ = 0U;
    recovery_last_tick_ = 0U;
    recovery_planner_.SetNowReal(0.0f);
    recovery_planner_.SetTarget(0.0f);
    recovery_planner_.TIM_Calculate_PeriodElapsedCallback(0.0f, 0.0f);
    motor_controllable_[0] = false;
    motor_controllable_[1] = false;
    feedback_degraded_ = false;
    feedback_valid_previous_[0] = false;
    feedback_valid_previous_[1] = false;
}

void Class_Up_Stair_Behind_Motor_FSM::Start_Recovery(uint32_t now_tick)
{
    // 后部重新上线时从零比例开始，避免姿态力矩突然恢复。
    Set_Status(UP_STAIR_BEHIND_MOTOR_RECOVERING);
    recovery_start_tick_ = now_tick;
    recovery_last_tick_ = now_tick;
    output_scale_ = 0.0f;
    recovery_planner_.SetNowReal(0.0f);
    recovery_planner_.SetTarget(0.0f);
    recovery_planner_.TIM_Calculate_PeriodElapsedCallback(0.0f, 0.0f);
}

void Class_Up_Stair_Behind_Motor_FSM::Update_Recovery_Scale(uint32_t now_tick)
{
    const uint32_t elapsed = now_tick - recovery_start_tick_;
    const uint32_t delta_tick = now_tick - recovery_last_tick_;
    recovery_last_tick_ = now_tick;

    // 根据真实经过时间设置本周期增量，SlopePlanning 输出约 300 ms 内从 0 到 1。
    float step = static_cast<float>(delta_tick) /
                 static_cast<float>(RECOVERY_TIME_MS);
    if (step > 1.0f)
    {
        step = 1.0f;
    }
    recovery_planner_.SetIncreaseValue(step);
    recovery_planner_.SetDecreaseValue(step);
    recovery_planner_.TIM_Calculate_PeriodElapsedCallback(1.0f, 0.0f);
    output_scale_ = recovery_planner_.GetOut();

    if (elapsed >= RECOVERY_TIME_MS || output_scale_ >= 1.0f)
    {
        Set_Status(UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
        output_scale_ = 1.0f;
    }
}

void Class_Up_Stair_Behind_Motor_FSM::Update(
    bool control_enabled,
    bool imu_valid,
    bool left_feedback_valid,
    bool right_feedback_valid,
    float pitch_deg,
    float roll_deg,
    float pitch_rate_dps,
    float roll_rate_dps,
    float left_angle_rad,
    float right_angle_rad,
    uint32_t now_tick)
{
    if (!std::isfinite(pitch_deg) || !std::isfinite(roll_deg) ||
        !std::isfinite(pitch_rate_dps) || !std::isfinite(roll_rate_dps))
    {
        // IMU 任一通道出现非有限值，整帧姿态数据作废并立即禁用后部输出。
        feedback_pitch_deg_ = 0.0f;
        feedback_roll_deg_ = 0.0f;
        pitch_rate_dps_ = 0.0f;
        roll_rate_dps_ = 0.0f;
        motor_angle_rad_[0] = 0.0f;
        motor_angle_rad_[1] = 0.0f;
        Disable();
        return;
    }

    const float calibrated_pitch_deg = pitch_deg - config_.pitch_zero_deg;
    const float calibrated_roll_deg = roll_deg - config_.roll_zero_deg;
    if (!std::isfinite(calibrated_pitch_deg) ||
        !std::isfinite(calibrated_roll_deg))
    {
        // 零偏相减发生浮点溢出时同样不能继续使用该姿态数据。
        feedback_pitch_deg_ = 0.0f;
        feedback_roll_deg_ = 0.0f;
        pitch_rate_dps_ = 0.0f;
        roll_rate_dps_ = 0.0f;
        motor_angle_rad_[0] = 0.0f;
        motor_angle_rad_[1] = 0.0f;
        Disable();
        return;
    }

    feedback_pitch_deg_ = calibrated_pitch_deg;
    feedback_roll_deg_ = calibrated_roll_deg;
    pitch_rate_dps_ = pitch_rate_dps;
    roll_rate_dps_ = roll_rate_dps;
    motor_angle_rad_[0] = left_angle_rad;
    motor_angle_rad_[1] = right_angle_rad;
    motor_controllable_[0] = config_valid_ && left_feedback_valid &&
                              Is_Angle_Valid(1U, left_angle_rad);
    motor_controllable_[1] = config_valid_ && right_feedback_valid &&
                              Is_Angle_Valid(2U, right_angle_rad);
    // 每个电机独立判断反馈和机械角度；无效侧由 Limit_Torque 强制为零。
    const bool feedback_recovered =
        (!feedback_valid_previous_[0] && motor_controllable_[0]) ||
        (!feedback_valid_previous_[1] && motor_controllable_[1]);
    feedback_valid_previous_[0] = motor_controllable_[0];
    feedback_valid_previous_[1] = motor_controllable_[1];

    if (!control_enabled || !imu_valid || !config_valid_)
    {
        // 档位、IMU 或标定配置任意一项不满足，后部不允许姿态控制。
        Disable();
        return;
    }

    const bool both_feedback_valid = motor_controllable_[0] &&
                                     motor_controllable_[1];
    if (Get_State() != UP_STAIR_BEHIND_MOTOR_DISABLED &&
        !both_feedback_valid)
    {
        feedback_degraded_ = true;
    }

    if (Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED)
    {
        Start_Recovery(now_tick);
        return;
    }

    if (feedback_recovered)
    {
        // 任意一侧从无效恢复，都重新启动全局软启动，防止单侧突加力矩。
        Start_Recovery(now_tick);
        feedback_degraded_ = !both_feedback_valid;
        return;
    }

    if (Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING)
    {
        Update_Recovery_Scale(now_tick);
    }
    else
    {
        output_scale_ = 1.0f;
    }
}

uint8_t Class_Up_Stair_Behind_Motor_FSM::Get_State() const
{
    return Now_Status_Serial;
}

float Class_Up_Stair_Behind_Motor_FSM::Get_Output_Scale() const
{
    return output_scale_;
}

float Class_Up_Stair_Behind_Motor_FSM::Get_Target_Pitch_Deg() const
{
    return 0.0f;
}

float Class_Up_Stair_Behind_Motor_FSM::Get_Target_Roll_Deg() const
{
    return 0.0f;
}

float Class_Up_Stair_Behind_Motor_FSM::Get_Feedback_Pitch_Deg() const
{
    return feedback_pitch_deg_;
}

float Class_Up_Stair_Behind_Motor_FSM::Get_Feedback_Roll_Deg() const
{
    return feedback_roll_deg_;
}

float Class_Up_Stair_Behind_Motor_FSM::Get_Pitch_Rate_Dps() const
{
    return pitch_rate_dps_;
}

float Class_Up_Stair_Behind_Motor_FSM::Get_Roll_Rate_Dps() const
{
    return roll_rate_dps_;
}

int8_t Class_Up_Stair_Behind_Motor_FSM::Get_Motor_Direction(uint8_t id) const
{
    const uint8_t index = To_Index(id);
    return index <= 1U ? config_.motor_direction[index] : 0;
}

bool Class_Up_Stair_Behind_Motor_FSM::Is_Motor_Controllable(uint8_t id) const
{
    const uint8_t index = To_Index(id);
    return index <= 1U && Get_State() != UP_STAIR_BEHIND_MOTOR_DISABLED &&
           motor_controllable_[index];
}

float Class_Up_Stair_Behind_Motor_FSM::Limit_Torque(
    uint8_t id, float raw_torque_nm) const
{
    const uint8_t index = To_Index(id);
    if (index > 1U || !Is_Motor_Controllable(id) ||
        (Get_State() != UP_STAIR_BEHIND_MOTOR_RECOVERING &&
         Get_State() != UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD) ||
        !std::isfinite(raw_torque_nm))
    {
        return 0.0f;
    }

    const float angle = To_Unwrapped_Angle(index, motor_angle_rad_[index]);
    const float lower = config_.angle_start_rad[index];
    const float upper = config_.angle_start_rad[index] <
                                config_.angle_end_rad[index]
                            ? config_.angle_end_rad[index]
                            : config_.angle_end_rad[index] + TWO_PI_RAD;
    const bool at_lower_margin = angle <= lower + LIMIT_MARGIN_RAD;
    const bool at_upper_margin = angle >= upper - LIMIT_MARGIN_RAD;
    if ((at_lower_margin && raw_torque_nm < 0.0f) ||
        (at_upper_margin && raw_torque_nm > 0.0f))
    {
        return 0.0f;
    }
    return raw_torque_nm * output_scale_;
}

bool Class_Up_Stair_Behind_Motor_FSM::Is_Config_Valid() const
{
    return config_valid_;
}
