#ifndef UP_STAIR_FSM_HPP
#define UP_STAIR_FSM_HPP

#include "../user/core/Alg/FSM/alg_fsm.hpp"
#include <stdint.h>

enum Enum_Up_Stair_Status
{
    UP_STAIR_DISABLED = 0,          // 未允许控制，输出应为零力矩
    UP_STAIR_HOME_HOLD,              // 前连杆保持机械初始位置
    UP_STAIR_MOVING_TO_TARGET,       // 前连杆受控运动到上台阶目标位置
    UP_STAIR_TARGET_HOLD,             // 到达目标位置后保持，用于卡住台阶
    UP_STAIR_RETURNING_HOME,          // 受控返回机械初始位置
    UP_STAIR_STATUS_COUNT
};

class Class_Up_Stair_FSM : public Class_FSM
{
public:
    // 编码器和机械限位配置：下标 0 对应电机 1，下标 1 对应电机 2。
    // 起始角大于终止角时，表示有效区间跨过 0 弧度。
    static constexpr float LIMIT_START_RAD[2] = {
        10.0f * 3.14159265359f / 180.0f,
        300.0f * 3.14159265359f / 180.0f};
    static constexpr float LIMIT_END_RAD[2] = {
        178.0f * 3.14159265359f / 180.0f,
        70.0f * 3.14159265359f / 180.0f};
    static constexpr float HOME_ANGLE_RAD[2] = {
        34.0f * 3.14159265359f / 180.0f,
        340.0f * 3.14159265359f / 180.0f};
    static constexpr float TARGET_ANGLE_RAD[2] = {
        122.0f * 3.14159265359f / 180.0f,
        60.0f * 3.14159265359f / 180.0f};

    void Init(uint32_t action_sequence);

    void Update(float current_left_angle,
                float current_right_angle,
                bool left_feedback_valid,
                bool right_feedback_valid,
                bool mechanism_enabled,
                bool stair_command_enabled,
                uint32_t action_sequence);

    float Get_Target_Angle(uint8_t id) const;
    float Get_Current_Angle(uint8_t id) const;
    float Get_Position_Feedback(uint8_t id) const;
    float Get_Position_Error(uint8_t id) const;
    bool Is_Angle_Valid(uint8_t id, float angle) const;
    uint8_t Get_State() const;
    bool Is_Enabled() const;

private:
    void Reset_Feedback();
    void Refresh_Target();
    bool Validate_Configuration() const;
    bool Both_Motors_At_Home(bool left_valid, bool right_valid) const;
    bool Both_Motors_At_Target(bool left_valid, bool right_valid) const;
    float To_Control_Angle(uint8_t index, float raw_angle) const;

private:
    float raw_angle_[2] = {0.0f, 0.0f};
    float control_angle_[2] = {0.0f, 0.0f};
    float target_angle_[2] = {0.0f, 0.0f};

    uint32_t last_action_sequence_ = 0U;
    bool config_valid_ = false;
};

#endif // UP_STAIR_FSM_HPP
