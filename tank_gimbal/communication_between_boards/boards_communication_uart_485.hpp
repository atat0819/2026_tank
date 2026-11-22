#pragma once
#include <stdint.h>

// 云台与底盘的 RS485 收发入口，使用 USART3。
void Gimbal485_Send();
void Gimbal485_Parse(const uint8_t *data, uint16_t size);

// 控制任务查询输入是否有效；CAN 回调更新 Yaw 编码器反馈。
bool Gimbal485_RemoteOnline();
void Gimbal485_UpdateYawFromISR(float angle_deg);

struct Gimbal485YawMouseInput
{
    bool online = false;
    bool mouse_active = false;
    bool reset = false;
    bool reanchor = false;
    int32_t mouse_delta_x = 0;
};

Gimbal485YawMouseInput Gimbal485_ConsumeYawMouseInput();

struct Gimbal485RefereeData
{
    uint16_t cooling_value = 0;
    uint16_t heat_limit = 0;
    uint16_t heat_42mm = 0;
    bool referee_online = false;
    bool received = false;
    uint32_t age_ms = UINT32_MAX;
};

// 一次读取同一帧的裁判数据和数据年龄，供发射任务使用。
void Gimbal485_GetRefereeData(Gimbal485RefereeData &data);
