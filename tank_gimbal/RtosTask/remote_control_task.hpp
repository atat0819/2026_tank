#ifndef REMOTE_CONTROL_TASK_HPP
#define REMOTE_CONTROL_TASK_HPP

#ifdef __cplusplus
#include "FreeRTOS.h"   // FreeRTOS 核心头文件
#include "queue.h"      // 队列相关类型/函数定义
#include "cmsis_os.h"
#include "../user/core/BSP/RemoteControl/DT7.hpp"
#include "../user/core/BSP/version/vision_communication.hpp"


extern "C" {

typedef struct
{
    float chassis_vx;
    float chassis_vy;
    float gimbal_yaw;
    float gimbal_pitch;
    float gimbal_roll;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left;
    uint8_t mouse_right;
    uint16_t keyboard;
    uint8_t s1;
    uint8_t s2;

} RemoteData_t;
extern RemoteData_t remote;



#endif

void remote_control_task(void *argument);

#ifdef __cplusplus


extern QueueHandle_t remoteDataQueue; // 声明一个全局队列句柄，用于在任务之间传递遥控器数据



extern BSP::REMOTE_CONTROL::RemoteController remoteController;



}

extern BSP::Vision::VisionCommunicator vision_comm;
#endif

#endif // REMOTE_CONTROL_TASK_HPP