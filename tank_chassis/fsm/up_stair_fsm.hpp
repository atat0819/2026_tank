#ifndef UP_STAIR_FSM_HPP
#define UP_STAIR_FSM_HPP

#include "../user/core/Alg/FSM/alg_fsm.hpp"
#include <stdint.h>

enum Enum_Up_Stair_Status
{
    UP_STAIR_DISABLED = 0,
    UP_STAIR_HOME,
    UP_STAIR_TARGET,
    UP_STAIR_STATUS_COUNT
};

class Class_Up_Stair_FSM : public Class_FSM
{
public:
    // Configure these values independently for motor 1 and motor 2.
    static constexpr float LIMIT_MIN_RAD[2] = {
        10.0f * 3.14159265359f / 180.0f,
        10.0f * 3.14159265359f / 180.0f};
    static constexpr float LIMIT_MAX_RAD[2] = {
        178.0f * 3.14159265359f / 180.0f,
        178.0f * 3.14159265359f / 180.0f};
    static constexpr float HOME_ANGLE_RAD[2] = {
        34.0f * 3.14159265359f / 180.0f,
        34.0f * 3.14159265359f / 180.0f};
    static constexpr float TARGET_ANGLE_RAD[2] = {
        122.0f * 3.14159265359f / 180.0f,
        122.0f * 3.14159265359f / 180.0f};

    void Init(uint32_t action_sequence);

    void Update(float current_left_angle,
                float current_right_angle,
                bool feedback_valid,
                bool enabled,
                uint32_t action_sequence);

    float Get_Target_Angle(uint8_t id) const;
    float Get_Current_Angle(uint8_t id) const;
    bool Is_Angle_Valid(uint8_t id, float angle) const;
    uint8_t Get_State() const;
    bool Is_Enabled() const;

private:
    void Reset_Feedback();
    void Refresh_Target();
    bool Validate_Configuration() const;

private:
    float raw_angle_[2] = {0.0f, 0.0f};
    float target_angle_[2] = {0.0f, 0.0f};

    uint32_t last_action_sequence_ = 0U;
    bool config_valid_ = false;
};

#endif // UP_STAIR_FSM_HPP
