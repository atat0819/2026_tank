#include "imu_task.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "spi.h"
#include <string.h>
#include <math.h>
#include "../user/core/BSP/simple_bmi088/simple_bmi088.hpp"
#include "../user/core/HAL/UART/uart_hal.hpp"

extern "C" TaskHandle_t xImuHandle;
auto &bmi088 = BSP::simplebmi088::GetOnboardBMI088();
volatile HAL_StatusTypeDef bmi088_init_status = HAL_ERROR;

/* VOFA+ JustFloat 帧: 9 个通道 + 1 个帧尾, 用 float 数组保证 4 字节对齐 */
static float vofa_frame[10];
static ImuControlSnapshot imu_control_snapshot = {0.0f, 0.0f, 0.0f, 0.0f,
                                                  0U, false};

void GetImuControlSnapshot(ImuControlSnapshot &snapshot)
{
    taskENTER_CRITICAL();
    snapshot = imu_control_snapshot;
    taskEXIT_CRITICAL();
}

static void PublishImuControlSnapshot(bool valid)
{
    // 将完整的一帧姿态和角速度数据一次性发布给上台阶任务。
    const float pitch = bmi088.GetPitchAngleDeg();
    const float roll = bmi088.GetRollAngleDeg();
    const float pitch_rate = bmi088.GetGyroRateYDps();
    const float roll_rate = bmi088.GetGyroRateXDps();
    const bool finite = std::isfinite(pitch) && std::isfinite(roll) &&
                        std::isfinite(pitch_rate) && std::isfinite(roll_rate);

    taskENTER_CRITICAL();
    if (valid && finite)
    {
        imu_control_snapshot.pitch_deg = pitch;
        imu_control_snapshot.roll_deg = roll;
        imu_control_snapshot.pitch_rate_dps = pitch_rate;
        imu_control_snapshot.roll_rate_dps = roll_rate;
        imu_control_snapshot.tick = HAL_GetTick();
        imu_control_snapshot.valid = true;
    }
    else
    {
        // 读取失败或数据不完整时，整帧必须失效，不能继续使用旧数据。
        imu_control_snapshot.valid = false;
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief 按 VOFA+ JustFloat 协议经 UART10 (TTL, 115200) 上报 9 个通道
 * @note  帧格式 = 9×float(小端) + 帧尾 0x7F800000 (小端即 00 00 80 7F)。
 *        UART10 是普通 TTL 串口 (PE2=RX, PE3=TX), 没有 DE 方向控制,
 *        接 USB转TTL 模块即可; 上位机波特率要设 115200。
 *        一帧 40 字节在 115200 下需要 3.47ms, 比本任务约 1ms 的循环周期还长,
 *        因此忙的时候会丢弃部分帧, 实测有效上报率约 290Hz —— 看姿态够用。
 */
static void vofa_send9(float x1, float x2, float x3, float x4, float x5,
                       float x6, float x7, float x8, float x9)
{
    /* 忙判断和发送必须用同一个设备, 换串口时只改这一行 */
    HAL::UART::IUartDevice &uart = HAL::UART::get_uart_bus_instance().get_uart10();

    /* 上一帧的 DMA 可能还在读这个缓冲区, 此时改写会发出半新半旧的残帧, 直接丢弃本帧。
     * 115200 下发一帧 40 字节要 3.47ms, 而本任务约 1ms 就被 DRDY 唤醒一次,
     * 所以这里会频繁命中, 等于把上报率自动降到约 290Hz —— 这是预期行为, 不是故障。
     * 想跑满速率就把 UART10 的波特率提上去(CubeMX 里改, 上位机同步改)。 */
    if (uart.get_handle()->gState != HAL_UART_STATE_READY)
    {
        return;
    }

    vofa_frame[0] = x1;
    vofa_frame[1] = x2;
    vofa_frame[2] = x3;
    vofa_frame[3] = x4;
    vofa_frame[4] = x5;
    vofa_frame[5] = x6;
    vofa_frame[6] = x7;
    vofa_frame[7] = x8;
    vofa_frame[8] = x9;

    /* 帧尾 0x7F800000, 小端存储为 00 00 80 7F */
    const uint32_t vofa_tail = 0x7F800000U;
    memcpy(&vofa_frame[9], &vofa_tail, sizeof(vofa_tail));

    HAL::UART::Data tx_data{reinterpret_cast<uint8_t *>(vofa_frame),
                            static_cast<uint16_t>(sizeof(vofa_frame))};
    uart.transmit_dma(tx_data);
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

        /* VOFA 通道顺序:
         *   [0..2] roll / pitch / yaw (deg) —— 上台阶主要看 pitch
         *   [3..5] 陀螺角速度 xyz (deg/s, 已减零偏)
         *   [6..8] 加速度 xyz (m/s²)
         */
        vofa_send9(bmi088.GetRollAngleDeg(), bmi088.GetPitchAngleDeg(), bmi088.GetYawAngleDeg(),
                   bmi088.GetGyroRateXDps(), bmi088.GetGyroRateYDps(), bmi088.GetGyroRateZDps(),
                   bmi088.GetAccelXMps2(),   bmi088.GetAccelYMps2(),   bmi088.GetAccelZMps2());
    }
}
