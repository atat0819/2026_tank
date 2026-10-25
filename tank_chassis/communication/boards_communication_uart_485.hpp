#pragma once
#include <stdint.h>

// 云台 -> 底盘：78 91 + 24 字节数据 + CRC16，共 28 字节。
// 多字节字段为小端；2..3=摇杆 X，4..5=最新 DT7 鼠标 X 采样值，16..23=停用 IMU 的保留字段。
typedef struct Chassis485Data
{
    int16_t right_stick_x;
    int16_t mouse_x;
    float vx;
    float vy;
    uint8_t s1;
    uint8_t s2;
    float imu_yaw_deg;
    float imu_gyro_z_deg_s;
    uint16_t keyboard;
} Chassis485Data;

// 从 UART lambda 调用，允许拆帧、粘帧和前导噪声；仅有效帧刷新数据并触发回复。
void Chassis485_Parse(const uint8_t *data, uint16_t size);
void Chassis485_ResetRx();
// 在底盘控制任务文件实现，ISR 内一次更新全部输入及心跳。
void Chassis485_ApplyData(const Chassis485Data &data);
