#ifndef UP_STAIR_FSM_HPP
#define UP_STAIR_FSM_HPP

#include "../user/core/Alg/FSM/alg_fsm.hpp"
#include "../user/core/Alg/UtilityFunction/SlopePlanning.hpp"
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
    // 起始角大于终止角时，表示有效区间跨过 ±π 接缝。
    // 左前：-67° -> 53.6°；右前：-110° -> -180°/+180° -> 128.35°。
    static constexpr float LIMIT_MARGIN_DEG = 15.0f; // 两侧电机在初始位、目标位两端各放宽的角度，单位：度
    static constexpr float HOME_ANGLE_RAD[2] = {    // 两侧电机的机械初始角
        -67.0f * 3.14159265359f / 180.0f,
        -110.0f * 3.14159265359f / 180.0f};
    static constexpr float TARGET_ANGLE_RAD[2] = {    // 两侧电机的上台阶目标角
        33.6f * 3.14159265359f / 180.0f,
        142.35f * 3.14159265359f / 180.0f};
    static constexpr float LIMIT_START_RAD[2] = {   // 左前从初始位向外放宽，右前从目标位向外放宽
        HOME_ANGLE_RAD[0] - LIMIT_MARGIN_DEG * 3.14159265359f / 180.0f,
        TARGET_ANGLE_RAD[1] - LIMIT_MARGIN_DEG * 3.14159265359f / 180.0f};
    static constexpr float LIMIT_END_RAD[2] = {    // 左前从目标位向外放宽，右前从初始位向外放宽
        TARGET_ANGLE_RAD[0] + LIMIT_MARGIN_DEG * 3.14159265359f / 180.0f,
        HOME_ANGLE_RAD[1] + LIMIT_MARGIN_DEG * 3.14159265359f / 180.0f};
    static constexpr uint32_t SPEED_RAMP_TIME_MS = 1650U; // 动作开始后，速度比例从 0 到 1 的时间

    // 初始化状态机和角度配置；启动后默认保持初始位置。
    void Init();

    // 输入两侧编码器反馈和当前控制权限，推进前 4340 状态机。
    // target_requested 是协调层给出的明确目标；false=初始位，true=目标位。
    void Update(float current_left_angle,
                float current_right_angle,
                bool left_feedback_valid,
                bool right_feedback_valid,
                bool mechanism_enabled,
                bool stair_command_enabled,
                bool target_requested,
                uint32_t now_tick); // 当前毫秒 tick，用于速度比例软启动

    // 获取指定电机当前状态对应的目标角度，单位：弧度。
    float Get_Target_Angle(uint8_t id) const;
    // 获取指定电机最近一次有效的原始编码器角度，单位：弧度。
    float Get_Current_Angle(uint8_t id) const;
    // 获取用于控制的连续角度；跨过 ±π 接缝的区间会被展开。
    float Get_Position_Feedback(uint8_t id) const;
    // 返回目标角度减控制反馈角度的误差，单位：弧度。
    float Get_Position_Error(uint8_t id) const;
    // 左右位置环输出共用的速度启动比例，范围 0 到 1。
    float Get_Speed_Scale() const;
    // 判断指定电机角度是否处于配置的机械安全范围内。
    bool Is_Angle_Valid(uint8_t id, float angle) const;
    // 返回当前前部状态枚举值。
    uint8_t Get_State() const;
    // 判断前部当前是否允许位置力矩控制。
    bool Is_Enabled() const;

private:
    // 清空反馈并将两个目标位置恢复为机械初始位置。
    void Reset_Feedback();
    // 根据当前状态刷新两个电机的目标角度。
    void Refresh_Target();
    // 检查机械限位、初始角和目标角配置是否自洽。
    bool Validate_Configuration() const;
    // 判断左右两个电机是否都已到达初始位置容差内。
    bool Both_Motors_At_Home(bool left_valid, bool right_valid) const;
    // 判断左右两个电机是否都已到达上台阶目标位置容差内。
    bool Both_Motors_At_Target(bool left_valid, bool right_valid) const;
    // 将跨越 ±π 接缝的原始角度转换为连续控制角度。
    float To_Control_Angle(uint8_t index, float raw_angle) const;

private:
    float raw_angle_[2] = {0.0f, 0.0f};       // 两个电机最近一次有效的原始角度
    float control_angle_[2] = {0.0f, 0.0f};   // 展开后的连续控制角度
    float target_angle_[2] = {0.0f, 0.0f};    // 状态机当前要求的目标角度
    Alg::Utility::SlopePlanning speed_planner_{0.0f, 0.0f}; // 只规划速度比例，最终目标角不变
    uint32_t speed_last_tick_ = 0U;

    bool last_target_requested_ = false;    // 恢复时同步目标，避免重放故障前动作
    bool config_valid_ = false;               // 静态机械配置是否有效
};

#endif // UP_STAIR_FSM_HPP
