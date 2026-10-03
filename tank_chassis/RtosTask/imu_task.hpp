#ifndef IMU_TASK_HPP
#define IMU_TASK_HPP

#include "cmsis_os.h"
#include <stdint.h>

#ifdef __cplusplus
#include "../user/core/BSP/simple_bmi088/simple_bmi088.hpp"

struct ImuControlSnapshot
{
    float pitch_deg;          // 去零偏前的 pitch 角度，单位：度
    float pitch_rate_dps;     // pitch 角速度，单位：度/秒
    float roll_deg;           // roll 角度，仅供 VOFA 观测，单位：度
    float roll_rate_dps;      // X 轴陀螺角速度，仅供 VOFA 观测，单位：度/秒
    float accel_x_mps2;       // X 轴加速度，仅供 VOFA 观测，单位：米/秒²
    float accel_y_mps2;       // Y 轴加速度，仅供 VOFA 观测，单位：米/秒²
    float accel_z_mps2;       // Z 轴加速度，仅供 VOFA 观测，单位：米/秒²
    float yaw_rate_dps;       // Z 轴陀螺角速度，仅供 VOFA 观测，单位：度/秒
    uint32_t tick;            // 本帧 IMU 读取完成后的板端时间，单位：毫秒
    bool valid;               // pitch 角度和角速度是否完整、有限且可用于控制
};

extern BSP::simplebmi088::BMI088 &bmi088;

// 在临界区内复制一整帧，避免任务读到不同时间的混合数据。
void GetImuControlSnapshot(ImuControlSnapshot &snapshot);

extern "C" {
#endif

void imu_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // IMU_TASK_HPP
