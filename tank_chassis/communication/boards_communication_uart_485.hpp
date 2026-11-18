#pragma once
#include <stdint.h>

// 底盘与云台的 RS485 收发入口，使用 USART3。
void Chassis485_Send();
void Chassis485_Parse(const uint8_t *data, uint16_t size);

// 遥控接收回调保存有效的 18 字节原始帧，控制任务更新裁判数据。
void Chassis485_UpdateRemote(const uint8_t *dbus);
void Chassis485_UpdateReferee(uint16_t cooling_value, uint16_t heat_limit,
                              uint16_t heat_42mm, uint8_t referee_online);
void Chassis485_ResetRx();

// 收到有效 Yaw 后交给底盘控制文件保存，当前不参与跟随控制。
struct Chassis485Data
{
    float yaw_offset_deg;
};
void Chassis485_ApplyData(const Chassis485Data &data);
