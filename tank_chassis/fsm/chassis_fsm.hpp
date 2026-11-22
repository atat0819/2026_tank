/**
 * @file chassis_fsm.hpp
 * @brief 底盘状态机 - 九种拨杆组合直接选择停车、遥控或键鼠
 *
 * 状态切换规则（基于遥控器拨杆 s1/s2）：
 *   - s1=2, s2=2 (或离线) -> STOP
 *   - s1=3, s2=3          -> MOUSE_ROTATE（键鼠失效时 STOP）
 *   - 其余七种有效组合    -> JOYSTICK_ROTATE
 * FOLLOW_GIMBAL / NOT_FOLLOW 保留，当前拨杆映射不进入这两个模式。
 *
 * 使用方法：
 *   1. 调用 Init() 初始化
 *   2. 调用 StateUpdate(s1, s2, online, keyboard_valid) 更新状态
 *   3. 调用 Get_wz_cmd(yaw_offset_rad, stick_x, mouse_x, max_wz) 获取角速度
 *   4. 定时调用 TIM_Calculate_PeriodElapsedCallback() 维护计时
 */

#ifndef CHASSIS_FSM_HPP
#define CHASSIS_FSM_HPP

#include "Alg/FSM/alg_fsm.hpp"
#include "Alg/PID/pid.hpp"
#include "main.h"

/* 底盘状态枚举 */
enum Enum_Chassis_Mode
{
    CHASSIS_STOP = 0,        // 停止
    CHASSIS_FOLLOW_GIMBAL,   // 跟随云台
    CHASSIS_NOT_FOLLOW,      // 不跟随云台（独立运动）
    CHASSIS_JOYSTICK_ROTATE, // 右摇杆直接控制旋转
    CHASSIS_MOUSE_ROTATE,    // 鼠标 X 直接控制旋转
    CHASSIS_MODE_COUNT       // 状态总数
};

/**
 * @brief 底盘状态机，继承自 Class_FSM
 */
class Chassis_FSM : public Class_FSM
{
public:
    /**
     * @brief 初始化状态机，默认进入 STOP 状态
     */
    void Init();

    /**
     * @brief 根据遥控器拨杆状态更新底盘模式
     * @param s1  左拨杆值 (1/2/3)
     * @param s2  右拨杆值 (1/2/3)
     * @param equipment_online  遥控器/云台是否在线
     */
    // 双中时 keyboard_valid 为 false 则停车，其余档位不依赖键盘。
    void StateUpdate(uint8_t s1, uint8_t s2, bool equipment_online,
                     bool keyboard_valid);

    /**
     * @brief 获取当前底盘模式
     */
    inline Enum_Chassis_Mode Get_Mode();

    /**
     * @brief 获取当前模式下的旋转角速度指令 wz_cmd
     * @param yaw_offset_rad  云台与底盘的偏航角差（弧度）
     * @return float  旋转角速度指令
     *
     * FOLLOW_GIMBAL 模式下返回 PID 跟随输出；
     * JOYSTICK_ROTATE / MOUSE_ROTATE 分别返回摇杆 / 鼠标角速度；
     * STOP / NOT_FOLLOW 返回 0。
     */
    // 原始操作量统一在状态机内换算；任务保留车体目标的统一限幅。
    float Get_wz_cmd(float yaw_offset_rad, float right_stick_x,
                     int16_t mouse_x, float max_wz_rad_s, bool yaw_mouse_active = false);

    // 保留原跟随 PID 接口，当前 Z 键跟随停用。
    float Get_follow_wz_cmd(float yaw_offset_rad);

    /**
     * @brief 定时器回调，驱动内部计时
     */
    void TIM_Calculate_PeriodElapsedCallback();

    /**
     * @brief 获取跟随 PID 实例引用（便于外部调参）
     */
    ALG::PID::PID& Get_Follow_PID() { return follow_pid; }

    // 跟随模式最大旋转角速度 (rad/s)，放 public 便于 Keil 实时调试
    float max_follow_speed = 4.0f;

private:
    Enum_Chassis_Mode current_mode = CHASSIS_STOP;
    // 跟随 PID，复用 ALG::PID::PID 库
    // 参数: kp, ki, kd, max_out, integral_limit, integral_separation
    ALG::PID::PID follow_pid;

    // 死区阈值（弧度）
    static constexpr float DEADZONE = 0.030f;

public:
    Chassis_FSM(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f,
                float max_out = 100.0f, float integral_limit = 25.0f, float integral_sep = 2.5f)
        : follow_pid(kp, ki, kd, max_out, integral_limit, integral_sep) {}
};

/* ---- 内联实现 ---- */

inline Enum_Chassis_Mode Chassis_FSM::Get_Mode()
{
    return current_mode;
}

#endif // CHASSIS_FSM_HPP
