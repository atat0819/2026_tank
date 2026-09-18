#include "up_stair_fsm.hpp"
#include <math.h>

constexpr float Class_Up_Stair_FSM::LIMIT_START_RAD[2];
constexpr float Class_Up_Stair_FSM::LIMIT_END_RAD[2];
constexpr float Class_Up_Stair_FSM::HOME_ANGLE_RAD[2];
constexpr float Class_Up_Stair_FSM::TARGET_ANGLE_RAD[2];

namespace
{
constexpr float TWO_PI_RAD = 2.0f * 3.14159265359f;
constexpr float POSITION_TOLERANCE_RAD = 2.0f * 3.14159265359f / 180.0f;
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
        const float raw_target = (Get_Now_Status_Serial() == UP_STAIR_MOVING_TO_TARGET ||
                                  Get_Now_Status_Serial() == UP_STAIR_TARGET_HOLD)
                                      ? TARGET_ANGLE_RAD[i]
                                      : HOME_ANGLE_RAD[i];
        target_angle_[i] = To_Control_Angle(i, raw_target);
    }
}

bool Class_Up_Stair_FSM::Both_Motors_At_Home(bool left_valid,
                                             bool right_valid) const
{
    return left_valid && right_valid &&
           fabsf(control_angle_[0] - To_Control_Angle(0U, HOME_ANGLE_RAD[0])) <=
               POSITION_TOLERANCE_RAD &&
           fabsf(control_angle_[1] - To_Control_Angle(1U, HOME_ANGLE_RAD[1])) <=
               POSITION_TOLERANCE_RAD;
}

bool Class_Up_Stair_FSM::Both_Motors_At_Target(bool left_valid,
                                               bool right_valid) const
{
    return left_valid && right_valid &&
           fabsf(control_angle_[0] - To_Control_Angle(0U, TARGET_ANGLE_RAD[0])) <=
               POSITION_TOLERANCE_RAD &&
           fabsf(control_angle_[1] - To_Control_Angle(1U, TARGET_ANGLE_RAD[1])) <=
               POSITION_TOLERANCE_RAD;
}

void Class_Up_Stair_FSM::Update(float current_left_angle,
                                float current_right_angle,
                                bool left_feedback_valid,
                                bool right_feedback_valid,
                                bool mechanism_enabled,
                                bool stair_command_enabled,
                                uint32_t action_sequence)
{
    const bool left_valid = left_feedback_valid &&
                            Is_Angle_Valid(1U, current_left_angle);
    const bool right_valid = right_feedback_valid &&
                             Is_Angle_Valid(2U, current_right_angle);

    if (!mechanism_enabled || !config_valid_ || (!left_valid && !right_valid))
    {
        Set_Status(UP_STAIR_DISABLED);
        last_action_sequence_ = action_sequence;
        Refresh_Target();
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
        Set_Status(UP_STAIR_RETURNING_HOME);
        Refresh_Target();
        last_action_sequence_ = action_sequence;
        if (Both_Motors_At_Home(left_valid, right_valid))
        {
            Set_Status(UP_STAIR_HOME_HOLD);
        }
        return;
    }

    if (!stair_command_enabled)
    {
        if (Get_Now_Status_Serial() != UP_STAIR_HOME_HOLD ||
            !Both_Motors_At_Home(left_valid, right_valid))
        {
            Set_Status(UP_STAIR_RETURNING_HOME);
            Refresh_Target();
        }
        if (Both_Motors_At_Home(left_valid, right_valid))
        {
            Set_Status(UP_STAIR_HOME_HOLD);
        }
        last_action_sequence_ = action_sequence;
        return;
    }

    // Each debounced B press increments the action sequence and toggles the
    // commanded position between home and target.
    if (action_sequence != last_action_sequence_)
    {
        last_action_sequence_ = action_sequence;
        if (Get_Now_Status_Serial() == UP_STAIR_HOME_HOLD ||
            Get_Now_Status_Serial() == UP_STAIR_RETURNING_HOME)
        {
            Set_Status(UP_STAIR_MOVING_TO_TARGET);
        }
        else
        {
            Set_Status(UP_STAIR_RETURNING_HOME);
        }
        Refresh_Target();
    }

    if (Get_Now_Status_Serial() == UP_STAIR_MOVING_TO_TARGET &&
        Both_Motors_At_Target(left_valid, right_valid))
    {
        Set_Status(UP_STAIR_TARGET_HOLD);
    }
    else if ((Get_Now_Status_Serial() == UP_STAIR_RETURNING_HOME ||
              Get_Now_Status_Serial() == UP_STAIR_HOME_HOLD) &&
             Both_Motors_At_Home(left_valid, right_valid))
    {
        Set_Status(UP_STAIR_HOME_HOLD);
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
