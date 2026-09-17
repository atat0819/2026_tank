#include "up_stair_fsm.hpp"

constexpr float Class_Up_Stair_FSM::LIMIT_MIN_RAD[2];
constexpr float Class_Up_Stair_FSM::LIMIT_MAX_RAD[2];
constexpr float Class_Up_Stair_FSM::HOME_ANGLE_RAD[2];
constexpr float Class_Up_Stair_FSM::TARGET_ANGLE_RAD[2];

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
        target_angle_[i] = HOME_ANGLE_RAD[i];
    }
}

bool Class_Up_Stair_FSM::Validate_Configuration() const
{
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        if (LIMIT_MIN_RAD[i] >= LIMIT_MAX_RAD[i] ||
            HOME_ANGLE_RAD[i] < LIMIT_MIN_RAD[i] ||
            HOME_ANGLE_RAD[i] > LIMIT_MAX_RAD[i] ||
            TARGET_ANGLE_RAD[i] < LIMIT_MIN_RAD[i] ||
            TARGET_ANGLE_RAD[i] > LIMIT_MAX_RAD[i])
        {
            return false;
        }
    }
    return true;
}

bool Class_Up_Stair_FSM::Is_Angle_Valid(uint8_t id, float angle) const
{
    if (id < 1U || id > 2U)
    {
        return false;
    }

    const uint8_t index = id - 1U;
    return angle >= LIMIT_MIN_RAD[index] && angle <= LIMIT_MAX_RAD[index];
}

void Class_Up_Stair_FSM::Refresh_Target()
{
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        target_angle_[i] = (Get_Now_Status_Serial() == UP_STAIR_TARGET)
                               ? TARGET_ANGLE_RAD[i]
                               : HOME_ANGLE_RAD[i];
    }
}

void Class_Up_Stair_FSM::Update(float current_left_angle,
                                float current_right_angle,
                                bool feedback_valid,
                                bool enabled,
                                uint32_t action_sequence)
{
    if (!feedback_valid || !enabled || !config_valid_ ||
        !Is_Angle_Valid(1U, current_left_angle) ||
        !Is_Angle_Valid(2U, current_right_angle))
    {
        Set_Status(UP_STAIR_DISABLED);
        last_action_sequence_ = action_sequence;
        target_angle_[0] = HOME_ANGLE_RAD[0];
        target_angle_[1] = HOME_ANGLE_RAD[1];
        return;
    }

    raw_angle_[0] = current_left_angle;
    raw_angle_[1] = current_right_angle;

    if (Get_Now_Status_Serial() == UP_STAIR_DISABLED)
    {
        Set_Status(UP_STAIR_HOME);
        Refresh_Target();
    }

    // B is represented by an action-sequence increment. The target remains
    // the configured target until the controller is disabled again.
    if (action_sequence != last_action_sequence_)
    {
        last_action_sequence_ = action_sequence;
        Set_Status(UP_STAIR_TARGET);
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

uint8_t Class_Up_Stair_FSM::Get_State() const
{
    return Now_Status_Serial;
}

bool Class_Up_Stair_FSM::Is_Enabled() const
{
    return Now_Status_Serial != UP_STAIR_DISABLED;
}
