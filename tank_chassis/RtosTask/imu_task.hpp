#ifndef IMU_TASK_HPP
#define IMU_TASK_HPP

#include "cmsis_os.h"

#ifdef __cplusplus
#include "../user/core/BSP/simple_bmi088/simple_bmi088.hpp"

extern BSP::simplebmi088::BMI088 &bmi088;

extern "C" {
#endif

void imu_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // IMU_TASK_HPP
