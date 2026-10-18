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
    // 起始角大于终止角时，表示有效区间跨过 ±π 接缝。
    // 左前：139° -> -160°，连续控制为 139° -> 200°；两端各放宽 15°。
    // 右前：-109° -> 172°，连续控制为 251° -> 172°；安全区间为 157° -> -94°。
    static constexpr float LIMIT_START_RAD[2] = {   // 两侧电机的机械限位起始角
        124.0f * 3.14159265359f / 180.0f,
        157.0f * 3.14159265359f / 180.0f};
    static constexpr float LIMIT_END_RAD[2] = {    // 两侧电机的机械限位终止角
        -145.0f * 3.14159265359f / 180.0f,
        -94.0f * 3.14159265359f / 180.0f};
    static constexpr float HOME_ANGLE_RAD[2] = {    // 两侧电机的机械初始角
        139.0f * 3.14159265359f / 180.0f,
        -109.0f * 3.14159265359f / 180.0f};
    static constexpr float TARGET_ANGLE_RAD[2] = {    // 两侧电机的上台阶目标角
        -160.0f * 3.14159265359f / 180.0f,
        172.0f * 3.14159265359f / 180.0f};

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
                bool target_requested);

    // 获取指定电机当前状态对应的目标角度，单位：弧度。
    float Get_Target_Angle(uint8_t id) const;
    // 获取指定电机最近一次有效的原始编码器角度，单位：弧度。
    float Get_Current_Angle(uint8_t id) const;
    // 获取用于控制的连续角度；跨过 ±π 接缝的区间会被展开。
    float Get_Position_Feedback(uint8_t id) const;
    // 返回目标角度减控制反馈角度的误差，单位：弧度。
    float Get_Position_Error(uint8_t id) const;
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

    bool last_target_requested_ = false;    // 恢复时同步目标，避免重放故障前动作
    bool config_valid_ = false;               // 静态机械配置是否有效
};

#endif // UP_STAIR_FSM_HPP
