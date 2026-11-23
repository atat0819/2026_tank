/**
 * @file chassis_fsm.cpp
 * @brief 底盘状态机实现
 */

#include "chassis_fsm.hpp"
#include <cstdlib>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

constexpr int MOUSE_TURN_DEADZONE_COUNTS = 6;

void Chassis_FSM::Init()
{
    Class_FSM::Init(CHASSIS_MODE_COUNT, CHASSIS_STOP);
    current_mode = CHASSIS_STOP;
    follow_pid.reset();
}

/**
 * @brief 拨杆值 -> 模式映射（逐挡位列出）
 *
 *   s1=上, s2=上 -> JOYSTICK_ROTATE（遥控）
 *   s1=上, s2=中 -> JOYSTICK_ROTATE（遥控）
 *   s1=上, s2=下 -> JOYSTICK_ROTATE（遥控）
 *   s1=中, s2=上 -> JOYSTICK_ROTATE（遥控）
 *   s1=中, s2=中 -> MOUSE_ROTATE（键鼠，失效则 STOP）
 *   s1=中, s2=下 -> JOYSTICK_ROTATE（遥控）
 *   s1=下, s2=上 -> JOYSTICK_ROTATE（遥控）
 *   s1=下, s2=中 -> JOYSTICK_ROTATE（遥控）
 *   s1=下, s2=下 -> STOP（停止）
 *   离线         -> STOP（停止）
 */
void Chassis_FSM::StateUpdate(uint8_t s1, uint8_t s2, bool equipment_online,
                              bool keyboard_valid)
{
    Enum_Chassis_Mode target_mode;

    // 离线 -> 停止
    if (!equipment_online)
    {
        target_mode = CHASSIS_STOP;
    }
    // s1上, s2上 -> 遥控
    else if (s1 == 1 && s2 == 1)
    {
        target_mode = CHASSIS_JOYSTICK_ROTATE;
    }
    // s1上, s2中 -> 遥控
    else if (s1 == 1 && s2 == 3)
    {
        target_mode = CHASSIS_JOYSTICK_ROTATE;
    }
    // s1上, s2下 -> 遥控
    else if (s1 == 1 && s2 == 2)
    {
        target_mode = CHASSIS_JOYSTICK_ROTATE;
    }
    // s1中, s2上 -> 遥控
    else if (s1 == 3 && s2 == 1)
    {
        target_mode = CHASSIS_JOYSTICK_ROTATE;
    }
    // s1中, s2中 -> 键鼠
    else if (s1 == 3 && s2 == 3)
    {
        target_mode = keyboard_valid ? CHASSIS_MOUSE_ROTATE : CHASSIS_STOP;
    }
    // s1中, s2下 -> 遥控
    else if (s1 == 3 && s2 == 2)
    {
        target_mode = CHASSIS_JOYSTICK_ROTATE;
    }
    // s1下, s2上 -> 遥控
    else if (s1 == 2 && s2 == 1)
    {
        target_mode = CHASSIS_JOYSTICK_ROTATE;
    }
    // s1下, s2中 -> 遥控
    else if (s1 == 2 && s2 == 3)
    {
        target_mode = CHASSIS_JOYSTICK_ROTATE;
    }
    // s1下, s2下 -> 停止
    else if (s1 == 2 && s2 == 2)
    {
        target_mode = CHASSIS_STOP;
    }
    // 其他情况 -> 停止
    else
    {
        target_mode = CHASSIS_STOP;
    }

    if (target_mode != current_mode)
    {
        Set_Status(static_cast<uint8_t>(target_mode));
        current_mode = target_mode;
        follow_pid.reset();
    }
}

/**
 * @brief 核心输出：根据当前模式返回旋转角速度指令
 *
 * - FOLLOW_GIMBAL: 使用 ALG::PID::PID 计算跟随输出（带死区）
 * - JOYSTICK_ROTATE / MOUSE_ROTATE: 摇杆 / 鼠标角速度
 * - STOP / NOT_FOLLOW: 返回 0
 */
float Chassis_FSM::Get_wz_cmd(float yaw_offset_rad, float right_stick_x,
                             int16_t mouse_x, float max_wz_rad_s, bool yaw_mouse_active)
{
    switch (current_mode)
    {
    case CHASSIS_FOLLOW_GIMBAL:
        return Get_follow_wz_cmd(yaw_offset_rad);

    case CHASSIS_JOYSTICK_ROTATE:
        return -right_stick_x * max_wz_rad_s;

    case CHASSIS_MOUSE_ROTATE:
        if (yaw_mouse_active || std::abs(static_cast<int>(mouse_x)) <= MOUSE_TURN_DEADZONE_COUNTS)
            return 0.0f;
        return -static_cast<float>(mouse_x) * (1.0f * M_PI / 180.0f);

    case CHASSIS_STOP:
        return 0.0f;
    case CHASSIS_NOT_FOLLOW:
        return 0.0f;
    default:
        return 0.0f;
    }
}

float Chassis_FSM::Get_follow_wz_cmd(float yaw_offset_rad)
{
    // 归一化到 [-π, π]
        while (yaw_offset_rad >  3.14159265f) 
				{
					yaw_offset_rad -= 2 * 3.14159265f;
				}
        while (yaw_offset_rad < -3.14159265f)
				{
					yaw_offset_rad += 2 * 3.14159265f;
				}

        if (fabsf(yaw_offset_rad) < DEADZONE)
        {
            follow_pid.reset();
            return 0.0f;
        }

        // UpDate(target, feedback): 目标是0偏移，反馈是当前偏移
        float output = follow_pid.UpDate(yaw_offset_rad, 0.0f);

        // 限速：钳位到 [-max_follow_speed, max_follow_speed]
        if (output >  max_follow_speed) 
				{
				output =  max_follow_speed;
				}
        if (output < -max_follow_speed) 
				{
				output = -max_follow_speed;
				}

    return output;
}

void Chassis_FSM::TIM_Calculate_PeriodElapsedCallback()
{
    Class_FSM::TIM_Calculate_PeriodElapsedCallback();
}
