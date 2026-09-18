#ifndef CAN_SEND_TASK_HPP
#define CAN_SEND_TASK_HPP

#include "cmsis_os.h"
#include "HAL/FDCAN/fdcan_hal.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void can_send_task(void *argument);

typedef struct
{
    float motor_speeds_1;
    float motor_speeds_2;
    float motor_speeds_3;
    float motor_speeds_4;
} MotorSpeedTarget_t;

typedef struct
{
    float vx;
    float vy;
    float wz;
} chassisCurrentData_t;

typedef struct
{
    float angle_deg;
    float angle_rad;
    float last_angle;
    float delta_angle;
    float speed_rpm;
    float speed_rads;
    float current;
    float torque;
    float temp;
} MotorCurrentData_t;

typedef struct
{
    float yaw_offset_deg; // 云台偏移量
    float vx;
    float vy;
    float s1;
    float s2;

} Gimbal_Chassis_communicate_t;

extern float yaw_offset_deg;
extern bool yaw_offset_updated;

extern Gimbal_Chassis_communicate_t gimbalChassis_communicate;
extern uint8_t gimbalChassisSpeedUpdated;

// 云台发送的键盘位掩码，供底盘控制任务按位判断按键状态
extern volatile uint16_t gimbal_keyboard;
extern volatile uint32_t gimbal_keyboard_last_tick; // 最近一次合法键盘帧时间
extern volatile bool gimbal_keyboard_received;      // 是否收到过合法键盘帧
extern volatile uint32_t gimbal_switch_last_tick;   // 最近一次合法档位帧时间
extern volatile bool gimbal_switch_received;  // 是否收到过合法档位帧


#ifdef __cplusplus
}
#endif

#endif // CAN_SEND_TASK_HPP
