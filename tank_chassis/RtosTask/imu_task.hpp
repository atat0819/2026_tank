#ifndef IMU_TASK_HPP
#define IMU_TASK_HPP

#include "cmsis_os.h"
#include <stdint.h>

#ifdef __cplusplus
#include "../user/core/BSP/simple_bmi088/simple_bmi088.hpp"

struct ImuControlSnapshot
{
    float pitch_deg;
    float roll_deg;
    float pitch_rate_dps;
    float roll_rate_dps;
    uint32_t tick;
    bool valid;
};

extern BSP::simplebmi088::BMI088 &bmi088;

void GetImuControlSnapshot(ImuControlSnapshot &snapshot);

extern "C" {
#endif

void imu_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // IMU_TASK_HPP
