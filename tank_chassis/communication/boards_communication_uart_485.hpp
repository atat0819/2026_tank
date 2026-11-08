#pragma once
#include <stdint.h>

// 云台 -> 底盘：78 91 + 18 字节数据 + CRC16，共 22 字节，不包含云台 IMU 数据。
// 多字节字段为小端；2..5=归一化摇杆 X float，6..7=最新 DT7 鼠标 X int16。
typedef struct Chassis485Data
{
    float right_stick_x;
    int16_t mouse_x;
    float vx;
    float vy;
    uint8_t s1;
    uint8_t s2;
    uint16_t keyboard;
} Chassis485Data;

// 从 UART lambda 调用，允许拆帧、粘帧和前导噪声；仅有效帧刷新数据并触发回复。
void Chassis485_Parse(const uint8_t *data, uint16_t size);
void Chassis485_ResetRx();
// 在底盘控制任务文件实现，ISR 内一次更新全部输入及心跳。
void Chassis485_ApplyData(const Chassis485Data &data);
