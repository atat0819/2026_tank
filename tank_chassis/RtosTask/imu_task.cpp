#include "imu_task.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "spi.h"
#include <math.h>
#include "../user/core/BSP/simple_bmi088/simple_bmi088.hpp"

extern "C" TaskHandle_t xImuHandle;
BSP::simplebmi088::BMI088 &bmi088 = BSP::simplebmi088::GetOnboardBMI088(); // 板载 BMI088 实例
volatile HAL_StatusTypeDef bmi088_init_status = HAL_ERROR; // BMI088 初始化结果

static ImuControlSnapshot imu_control_snapshot = {}; // pitch 控制与 VOFA 观测共用快照

// 在临界区复制最近一帧完整 IMU 控制数据，供上台阶任务安全读取。
void GetImuControlSnapshot(ImuControlSnapshot &snapshot)
{
    taskENTER_CRITICAL();
    snapshot = imu_control_snapshot;
    taskEXIT_CRITICAL();
}

// 将本周期读取到的 IMU 数据整体发布；失败时只更新 valid=false。
static void PublishImuControlSnapshot(bool valid)
{
    // 在读取完成后记录时间，不使用 VOFA 发送时间；此时间不是 DRDY 硬件采样时戳。
    const uint32_t sample_tick = HAL_GetTick();
    // pitch 用于后腿控制；其余传感器输入随同一帧发布，仅供 VOFA 观测。
    const float pitch = bmi088.GetPitchAngleDeg();
    const float pitch_rate = bmi088.GetGyroRateYDps();
    const bool finite = std::isfinite(pitch) && std::isfinite(pitch_rate);

    taskENTER_CRITICAL();
    if (valid && finite)
    {
        imu_control_snapshot.pitch_deg = pitch;
        imu_control_snapshot.pitch_rate_dps = pitch_rate;
        imu_control_snapshot.roll_deg = bmi088.GetRollAngleDeg();
        imu_control_snapshot.roll_rate_dps = bmi088.GetGyroRateXDps();
        imu_control_snapshot.accel_x_mps2 = bmi088.GetAccelXMps2();
        imu_control_snapshot.accel_y_mps2 = bmi088.GetAccelYMps2();
        imu_control_snapshot.accel_z_mps2 = bmi088.GetAccelZMps2();
        imu_control_snapshot.yaw_rate_dps = bmi088.GetGyroRateZDps();
        imu_control_snapshot.tick = sample_tick;
        imu_control_snapshot.valid = true;
    }
    else
    {
        // 读取失败或数据不完整时，整帧必须失效，不能继续使用旧数据。
        imu_control_snapshot.valid = false;
    }
    taskEXIT_CRITICAL();
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if ((GPIO_Pin == GPIO_PIN_12) && (xImuHandle != NULL))
    {
        vTaskNotifyGiveFromISR(xImuHandle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}


float pitch =0.0f;
/**
 * @brief 上台阶任务（空实现占位）
 * @note  当前暂无上台阶控制逻辑，仅保留任务入口，保证 Core/Src/freertos.c 中的
 *        xTaskCreate(up_stair_task, ...) 能够链接通过。后续实现时在此补充。
 */
extern "C" void imu_task(void *argument)
{
    (void)argument;

        /* 等待任务调度器和 SPI 外设稳定后，再开始读取 IMU。 */
    vTaskDelay(pdMS_TO_TICKS(100U));

    for (;;)
    {
        if (bmi088_init_status != HAL_OK)
        {
            bmi088_init_status = BMI088_Init(&hspi2);
            bmi088.RefreshData();

            if (bmi088_init_status != HAL_OK)
            {
                PublishImuControlSnapshot(false);
                vTaskDelay(pdMS_TO_TICKS(1000U));
                continue;
            }
        }

        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100U));
        bmi088_init_status = bmi088.Read();
        PublishImuControlSnapshot(bmi088_init_status == HAL_OK &&
                                  bmi088.IsReady());

        // VOFA 九路数据由 up_stair_task 读取完整快照后统一发送。
				pitch = bmi088.GetPitchAngleDeg();
    }
}
