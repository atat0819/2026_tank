#include "up_stair.hpp"

/**
 * @brief 上台阶任务（空实现占位）
 * @note  当前暂无上台阶控制逻辑，仅保留任务入口，保证 Core/Src/freertos.c 中的
 *        xTaskCreate(up_stair_task, ...) 能够链接通过。后续实现时在此补充。
 */
extern "C" void up_stair_task(void *argument)
{
    (void)argument;

    for (;;)
    {
        osDelay(100);
    }
}
