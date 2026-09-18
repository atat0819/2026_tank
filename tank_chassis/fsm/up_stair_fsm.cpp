#include "up_stair_fsm.hpp"

constexpr float Class_Up_Stair_FSM::LIMIT_START_RAD[2];
constexpr float Class_Up_Stair_FSM::LIMIT_END_RAD[2];
constexpr float Class_Up_Stair_FSM::HOME_ANGLE_RAD[2];
constexpr float Class_Up_Stair_FSM::TARGET_ANGLE_RAD[2];

namespace
{
constexpr float TWO_PI_RAD = 2.0f * 3.14159265359f;
}

void Class_Up_Stair_FSM::Init(uint32_t action_sequence)
{
    Class_FSM::Init(UP_STAIR_STATUS_COUNT, UP_STAIR_DISABLED);
    last_action_sequence_ = action_sequence;
    config_valid_ = Validate_Configuration();
    Reset_Feedback();
}

void Class_Up_Stair_FSM::Reset_Feedback()
{
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        raw_angle_[i] = 0.0f;
        control_angle_[i] = 0.0f;
        target_angle_[i] = To_Control_Angle(i, HOME_ANGLE_RAD[i]);
    }
}

bool Class_Up_Stair_FSM::Is_Angle_Valid(uint8_t id, float angle) const
{
    if (id < 1U || id > 2U)
    {
        return false;
    }

    const uint8_t index = id - 1U;
    if (LIMIT_START_RAD[index] < LIMIT_END_RAD[index])
    {
        return angle >= LIMIT_START_RAD[index] &&
               angle <= LIMIT_END_RAD[index];
    }

    if (LIMIT_START_RAD[index] > LIMIT_END_RAD[index])
    {
        return angle >= LIMIT_START_RAD[index] ||
               angle <= LIMIT_END_RAD[index];
    }

    return false;
}

float Class_Up_Stair_FSM::To_Control_Angle(uint8_t index, float raw_angle) const
{
    if (LIMIT_START_RAD[index] > LIMIT_END_RAD[index] &&
        raw_angle < LIMIT_START_RAD[index])
    {
        return raw_angle + TWO_PI_RAD;
    }
    return raw_angle;
}

bool Class_Up_Stair_FSM::Validate_Configuration() const
{
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        if (LIMIT_START_RAD[i] == LIMIT_END_RAD[i] ||
            !Is_Angle_Valid(i + 1U, HOME_ANGLE_RAD[i]) ||
            !Is_Angle_Valid(i + 1U, TARGET_ANGLE_RAD[i]))
        {
            return false;
        }
    }
    return true;
}

void Class_Up_Stair_FSM::Refresh_Target()
{
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        const float raw_target = (Get_Now_Status_Serial() == UP_STAIR_TARGET)
                                      ? TARGET_ANGLE_RAD[i]
                                      : HOME_ANGLE_RAD[i];
        target_angle_[i] = To_Control_Angle(i, raw_target);
    }
}

void Class_Up_Stair_FSM::Update(float current_left_angle,
                                float current_right_angle,
                                bool left_feedback_valid,
                                bool right_feedback_valid,
                                bool enabled,
                                uint32_t action_sequence)
{
    const bool left_valid = left_feedback_valid &&
                            Is_Angle_Valid(1U, current_left_angle);
    const bool right_valid = right_feedback_valid &&
                             Is_Angle_Valid(2U, current_right_angle);

    if (!enabled || !config_valid_ || (!left_valid && !right_valid))
    {
        Set_Status(UP_STAIR_DISABLED);
        last_action_sequence_ = action_sequence;
        target_angle_[0] = To_Control_Angle(0U, HOME_ANGLE_RAD[0]);
        target_angle_[1] = To_Control_Angle(1U, HOME_ANGLE_RAD[1]);
        return;
    }

    if (left_valid)
    {
        raw_angle_[0] = current_left_angle;
        control_angle_[0] = To_Control_Angle(0U, raw_angle_[0]);
    }
    if (right_valid)
    {
        raw_angle_[1] = current_right_angle;
        control_angle_[1] = To_Control_Angle(1U, raw_angle_[1]);
    }

    if (Get_Now_Status_Serial() == UP_STAIR_DISABLED)
    {
        Set_Status(UP_STAIR_HOME);
        Refresh_Target();
    }

    // Each debounced B press increments the action sequence and toggles the
    // commanded position between home and target.
    if (action_sequence != last_action_sequence_)
    {
        last_action_sequence_ = action_sequence;
        Set_Status(Get_Now_Status_Serial() == UP_STAIR_TARGET
                       ? UP_STAIR_HOME
                       : UP_STAIR_TARGET);
        Refresh_Target();
    }
}

float Class_Up_Stair_FSM::Get_Target_Angle(uint8_t id) const
{
    if (id < 1U || id > 2U)
    {
        return 0.0f;
    }
    return target_angle_[id - 1U];
}

float Class_Up_Stair_FSM::Get_Current_Angle(uint8_t id) const
{
    if (id < 1U || id > 2U)
    {
        return 0.0f;
    }
    return raw_angle_[id - 1U];
}

float Class_Up_Stair_FSM::Get_Position_Feedback(uint8_t id) const
{
    if (id < 1U || id > 2U)
    {
        return 0.0f;
    }
    return control_angle_[id - 1U];
}

float Class_Up_Stair_FSM::Get_Position_Error(uint8_t id) const
{
    if (id < 1U || id > 2U)
    {
        return 0.0f;
    }
    return target_angle_[id - 1U] - control_angle_[id - 1U];
}

uint8_t Class_Up_Stair_FSM::Get_State() const
{
    return Now_Status_Serial;
}

bool Class_Up_Stair_FSM::Is_Enabled() const
{
    return Now_Status_Serial != UP_STAIR_DISABLED;
}
