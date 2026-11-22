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

// 云台通过 RS485 回传的 Yaw 偏角，供观测。
extern float yaw_offset_deg;

extern void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6);
// USB CDC JustFloat，最多 15 通道；未连接或忙时跳过，不阻塞控制任务。
void vofa_sendN(const float *data, uint8_t count);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include "../fsm/control_input_selector.hpp"

struct ControlInputSnapshot {
    ControlInputSource source = ControlInputSource::NONE;
    bool keyboard_online = false;
    uint8_t s1 = 0U;
    uint8_t s2 = 0U;
    uint16_t keyboard = 0U;
    int16_t mouse_x = 0;
    bool yaw_mouse_active = false;
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
};

extern ControlInputSnapshot chassis_input_watch;

ControlInputSnapshot GetControlInputSnapshot();
#endif

#endif // CAN_SEND_TASK_HPP
