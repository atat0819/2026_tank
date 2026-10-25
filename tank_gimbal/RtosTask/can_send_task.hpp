#ifndef CAN_SEND_TASK_HPP
#define CAN_SEND_TASK_HPP

#ifdef __cplusplus
#include "cmsis_os.h"
#include "remote_control_task.hpp"
#include "Alg/PID/pid.hpp"
#include "boards_communication.hpp"
#include "gimbal_task.hpp"
#include "Alg/Filter/Filter.hpp"
extern "C" {



typedef struct {
    float angle_deg;      // 实时角度 (度)
    float angle_rad;      // 实时角度 (rad)
    float velocity_rpm;   // 输出轴转速 (RPM)
    float velocity_rads;   // 输出轴转速 (RADS)   
    float current_a;      // 实时电流 (A)
    float delta_angle;    // 与上次读取的角度差值 (度)，用于多圈计算
    uint8_t temperature;  // 电机温度

} DM4340_State_t;
extern DM4340_State_t dm4340_state[1]; // index 0=Pitch, physical CAN ID8, logical slot 1

extern RemoteData_t RemoteData;
extern volatile uint8_t gimbal_remote_offline;

#endif

void can_send_task(void *argument);

#ifdef __cplusplus

extern float pitch_target_angle ; // 来自遥控器的目标角度
extern float pitch_target_speed ; // 来自pid计算的目标速度


void vofa_sendN(const float *data, uint8_t count);
void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6);


}
#endif

#endif // CAN_SEND_TASK_HPP
