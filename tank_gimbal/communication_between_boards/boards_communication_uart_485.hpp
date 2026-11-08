#pragma once

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// 第一条链路 P5 / USART3，固定 22 字节，硬件 DE 自动控制收发方向。
// 仅由 can_send_task 调用；HAL_OK 表示 DMA 已启动，并非对端已接收。
// 收到有效回复后下一轮任务即可发送；未收到回复则等待最多 2ms，不阻塞任务。
// HAL_BUSY/HAL_ERROR 不覆盖在途 DMA 缓冲区；下次发送最新状态，不排队或累加鼠标 X。
// 摇杆 X 为与 Vx/Vy 相同的归一化 float（-1..1），沿用现有中值和死区处理。
// 鼠标 X 为最新 DT7 帧的采样值：下一帧到来前持续发送，新帧为 0 时立即更新为 0。
// 鼠标 X 为 int16_t；两项均保留原始正负号，底盘覆盖保存并每周期用于目标角速度计算。
// Pitch 的鼠标 Y 角度增量仍由任务单次消费，不受本字段语义影响。
// 失联或非键鼠来源时鼠标 X 为 0；Vx/Vy 保持原遥控器归一化量。
// 协议不包含云台 IMU 数据。
// 协议：0..1=78 91；2..5=右摇杆 X；6..7=最新鼠标 X；8..11=Vx；12..15=Vy；16=S1；17=S2；
//       18..19=键盘；20..21=CRC16。
// 所有多字节字段为小端，float 为 IEEE754 单精度。
// CRC16/MCRF4XX：初值 FFFF，反射多项式 8408，无最终异或，覆盖字节 0..19。
// 接收方需搜索帧头、累计 22 字节并校验 CRC，不能把一次 Idle 事件当成一整帧。
HAL_StatusTypeDef RS485_SendBoardData(float right_stick_x, int16_t mouse_x,
                                    float chassis_vx, float chassis_vy,
                                    uint8_t s1, uint8_t s2,
                                    uint16_t keyboard);
// 仅校验通过的底盘回复调用，结束本轮等待。
void RS485_ReplyReceived(void);

#ifdef __cplusplus
}
#endif
