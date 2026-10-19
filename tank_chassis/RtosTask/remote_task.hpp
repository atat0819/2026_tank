#ifndef REMOTE_TASK_HPP
#define REMOTE_TASK_HPP

#include "FreeRTOS.h"   // FreeRTOS 核心头文件
#include "queue.h"      // 队列相关类型/函数定义
#include "cmsis_os.h"
#include "../user/core/BSP/RemoteControl/DT7.hpp"

#ifdef __cplusplus
#include "../user/core/BSP/Hi14/HI14.hpp"

// 后腿 IMU 数据最大年龄，单位 ms；快照接口和控制任务共用此门限。
constexpr uint32_t IMU_CONTROL_TIMEOUT_MS = 100U;

// HI14 后腿控制反馈接口，实现在 remote_task.cpp；不创建 IMU 任务。
struct ImuControlSnapshot
{
    float pitch_deg;          // 去零偏前的 Pitch 角度，单位：度
    float pitch_rate_dps;     // Pitch 角速度，单位：度/秒
    float roll_deg;           // 去零偏前的 Roll 角度，单位：度；左侧抬高时增大
    uint32_t tick;            // 有效帧解析完成时的板端时间，单位：毫秒
    bool valid;
};

// 仅供 UART7 接收/错误中断调用。
void PublishHi14ImuFromISR(const IMU_Float_t &data, uint32_t sample_tick, bool valid);
// 在临界区复制整帧；数据年龄达到 IMU_CONTROL_TIMEOUT_MS 时返回 valid=false。
void GetImuControlSnapshot(ImuControlSnapshot &snapshot);

extern "C" {
#endif

typedef struct
{
    float vx = 0.0f; // 前后速度，单位 m/s
    float vy = 0.0f; // 左右速度，单位 m/s
    float wz = 0.0f; // 旋转速度，单位 rad/s
    uint8_t s1 = 0; // 功能按键 S1 状态
    uint8_t s2 = 0; // 功能按键 S2 状态
} RemoteData_t;
extern RemoteData_t remoteData;     // 解析后的遥控器数据结构

typedef struct 
{
 // 加速度，单位 g（1g = 9.8 m/s²）
    float acc_x;
    float acc_y;
    float acc_z;
    // 静止时，Z 轴约等于 1g（重力）
    // 可以用来判断机器人是否在斜坡上
}IMUData_t;
extern IMUData_t imuData;        // 解析后的 IMU 数据结构

typedef struct 
{
 // 加速度，单位 g（1g = 9.8 m/s²）
    float voltage;
    float current;
    float power;
    // 静止时，Z 轴约等于 1g（重力）
    // 可以用来判断机器人是否在斜坡上
}PowerData_t;
extern PowerData_t PowerData;

void remote_task(void *argument);

extern uint8_t receivedata[18]; // 声明全局缓冲区，用于存储接收到的数据
extern uint8_t imu_rx_buffer[64]; // 声明全局缓冲区，用于存储接收到的IMU数据
extern QueueHandle_t remoteDataQueue; // 声明一个全局队列句柄，用于在任务之间传递遥控器数据
extern QueueHandle_t IMUDataQueue;  // 用于传递 IMU 数据的队列

extern BSP::REMOTE_CONTROL::RemoteController remoteController;
extern volatile uint16_t local_remote_keyboard;
extern volatile int16_t local_remote_mouse_x;
extern volatile uint32_t local_remote_last_tick;
extern volatile bool local_remote_received;






#ifdef __cplusplus
}
#endif

#endif // REMOTE_TASK_HPP
