#pragma once

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// 第一条链路 P5 / USART3，固定 28 字节，硬件 DE 自动控制收发方向。
// 仅由 can_send_task 调用；HAL_OK 表示 DMA 已启动，并非对端已接收。
// 收到有效回复后下一轮任务即可发送；未收到回复则等待最多 2ms，不阻塞任务。
// HAL_BUSY 时丢弃本次快照，下次调用发送最新数据，不排队重放旧操作。
// Vx/Vy 保持原遥控器归一化量，Yaw 偏移为度；IMU Yaw 为度、gyro_z 为度/秒。
// 协议：0..1=78 91；2..5=Yaw 偏移；6..9=Vx；10..13=Vy；14=S1；15=S2；
//       16..19=IMU Yaw [0,360)；20..23=gyro_z；24..25=键盘；26..27=CRC16。
// 所有多字节字段为小端，float 为 IEEE754 单精度。
// CRC16/MCRF4XX：初值 FFFF，反射多项式 8408，无最终异或，覆盖字节 0..25。
// 接收方需搜索帧头、累计 28 字节并校验 CRC，不能把一次 Idle 事件当成一整帧。
HAL_StatusTypeDef RS485_SendBoardData(float chassis_vx, float chassis_vy,
                                    uint8_t s1, uint8_t s2,
                                    float imu_yaw_deg, float gyro_z_deg_s,
                                    uint16_t keyboard);
// 仅校验通过的底盘回复调用，结束本轮等待。
void RS485_ReplyReceived(void);

#ifdef __cplusplus
}
#endif
