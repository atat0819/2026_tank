#include "up_stair_behind_motor_fsm.hpp"

#include <math.h>

namespace
{
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
      feedback_degraded_(false),
      feedback_valid_previous_{false, false}
{
    Class_FSM::Init(UP_STAIR_BEHIND_MOTOR_COUNT,
                    UP_STAIR_BEHIND_MOTOR_DISABLED);
    config_valid_ = Validate_Config();
}

void Class_Up_Stair_Behind_Motor_FSM::Reset()
{
    Class_FSM::Init(UP_STAIR_BEHIND_MOTOR_COUNT,
                    UP_STAIR_BEHIND_MOTOR_DISABLED);
    output_scale_ = 0.0f;
    recovery_start_tick_ = 0U;
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
    Set_Status(UP_STAIR_BEHIND_MOTOR_DISABLED);
    output_scale_ = 0.0f;
    recovery_start_tick_ = 0U;
    motor_controllable_[0] = false;
    motor_controllable_[1] = false;
    feedback_degraded_ = false;
    feedback_valid_previous_[0] = false;
    feedback_valid_previous_[1] = false;
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
    const bool feedback_recovered =
        (!feedback_valid_previous_[0] && motor_controllable_[0]) ||
        (!feedback_valid_previous_[1] && motor_controllable_[1]);
    feedback_valid_previous_[0] = motor_controllable_[0];
    feedback_valid_previous_[1] = motor_controllable_[1];

    if (!control_enabled || !imu_valid || !config_valid_)
    {
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
        Set_Status(UP_STAIR_BEHIND_MOTOR_RECOVERING);
        recovery_start_tick_ = now_tick;
        output_scale_ = 0.0f;
        return;
    }

    if (feedback_recovered)
    {
        Set_Status(UP_STAIR_BEHIND_MOTOR_RECOVERING);
        recovery_start_tick_ = now_tick;
        output_scale_ = 0.0f;
        feedback_degraded_ = !both_feedback_valid;
        return;
    }

    if (Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING)
    {
        const uint32_t elapsed = now_tick - recovery_start_tick_;
        if (elapsed >= RECOVERY_TIME_MS)
        {
            Set_Status(UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
            output_scale_ = 1.0f;
        }
        else
        {
            output_scale_ = static_cast<float>(elapsed) /
                            static_cast<float>(RECOVERY_TIME_MS);
        }
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
