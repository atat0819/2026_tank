#include "remote_control_task.hpp"
#include "watchdog_heartbeat.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart.h"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include "../user/core/BSP/version/vision_communication.hpp"
#include "../communication_between_boards/input_dispatcher.hpp"
#include "../communication_between_boards/boards_communication_uart_485.hpp"
#include "gimbal_task.hpp"

QueueHandle_t remoteDataQueue;

uint8_t receivedata[18];
RemoteData_t remote;
BSP::REMOTE_CONTROL::RemoteController remoteController(100);
// UART5 接收到首个完整帧后置位，避免上电时误报在线。
static volatile bool arm_remote_received = false;

extern DMA_HandleTypeDef hdma_uart5_rx;

// 视觉通信实例（文件作用域，HAL 回调需要访问其 RX 缓冲区）
BSP::Vision::VisionCommunicator vision_comm;

// ==================== HAL UART 回调 ====================

// DT7 和 RS485 使用 DMA+Idle，此回调无需处理接收数据。
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // RS485 DMA+Idle：忽略半传输，完整事件处理后复用原缓冲区重新接收。
    if (huart == &huart2 || huart == &huart3)
    {
        if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
        {
            return;
        }
        auto &bus = HAL::UART::get_uart_bus_instance();
        auto &uart = huart == &huart2 ? bus.get_uart2() : bus.get_uart3();
        HAL::UART::Data next_rx{huart->pRxBuffPtr, huart->RxXferSize};
        HAL::UART::Data rx{next_rx.buffer, Size};
        uart.trigger_rx_callbacks(rx);
        if (uart.receive_dma_idle(next_rx))
        {
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        }
        return;
    }
    // UART5 — DT7 遥控器（100000 baud，DBUS 协议）
    auto &uart5 = HAL::UART::get_uart_bus_instance().get_uart5();
    if (huart == uart5.get_handle())
    {
        HAL::UART::Data rx{receivedata, Size};
        uart5.trigger_rx_callbacks(rx);

        HAL::UART::Data next_rx{receivedata, sizeof(receivedata)};
        uart5.receive_dma_idle(next_rx);
        __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
    }


}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2 || huart == &huart3)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        // RX/TX 共用 UART：DMA 错误后先释放接收流，不改动发送状态。
        if (HAL_UART_AbortReceive(huart) != HAL_OK)
        {
            return;
        }
        auto &bus = HAL::UART::get_uart_bus_instance();
        auto &uart = huart == &huart2 ? bus.get_uart2() : bus.get_uart3();
        HAL::UART::Data rx{huart->pRxBuffPtr, huart->RxXferSize};
        if (uart.receive_dma_idle(rx))
        {
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        }
        return;
    }
    auto &uart5 = HAL::UART::get_uart_bus_instance().get_uart5();

    if (huart == uart5.get_handle())
    {
        // 读 ISR 再读 RDR 清除 ORE/NE/FE/PE（H7 为 ISR/RDR，F4 时代为 SR/DR）
        volatile uint32_t isr = huart->Instance->ISR;
        volatile uint32_t rdr = huart->Instance->RDR;
        (void)isr;
        (void)rdr;

        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        huart->RxState = HAL_UART_STATE_READY;
        huart->Lock = HAL_UNLOCKED;

        HAL::UART::Data rx{receivedata, sizeof(receivedata)};
        uart5.receive_dma_idle(rx);
        __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
    }

}

//虚拟串口假回调 (见USB_DEVICE\App\usbd_cdc_if.c)
extern "C" void USB_Receive_Callback(uint8_t *Buf, uint32_t Len)
{
    vision_comm.ParseRxData(Buf, Len);
}

// ==================== 任务函数 ====================

extern "C" void remote_control_task(void *argument)
{
    remoteDataQueue = xQueueCreate(1, sizeof(RemoteData_t));

    // 初始化 UART 总线（懒加载单例，触发所有设备 init + start）
    HAL::UART::get_uart_bus_instance();

    auto &uart2_485 = HAL::UART::get_uart_bus_instance().get_uart2();
    auto &uart3_485 = HAL::UART::get_uart_bus_instance().get_uart3();
    auto &uart5 = HAL::UART::get_uart_bus_instance().get_uart5();

    // UART2 — RS485 第二路，预留接收数据处理。
    uart2_485.register_rx_callback([](const HAL::UART::Data &data) {
        (void)data;
        // 在这里添加接收数据处理。
    });

    // USART3：接收底盘的遥控器、键鼠和裁判系统数据。
    uart3_485.register_rx_callback([](const HAL::UART::Data &data) {
        Gimbal485_Parse(data.buffer, data.size);
    });

    // UART5 回调：DMA+空闲 → 遥控器解析
    // 启动接收
    // USART3：接收底盘的遥控器、键鼠和裁判系统数据。
    static uint8_t uart2_rx_buffer[64];
    static uint8_t uart3_rx_buffer[64];
    HAL::UART::Data uart2_rx{uart2_rx_buffer, sizeof(uart2_rx_buffer)};
    if (uart2_485.receive_dma_idle(uart2_rx))
    {
        __HAL_DMA_DISABLE_IT(uart2_485.get_handle()->hdmarx, DMA_IT_HT);
    }

    HAL::UART::Data uart3_rx{uart3_rx_buffer, sizeof(uart3_rx_buffer)};
    if (uart3_485.receive_dma_idle(uart3_rx))
    {
        __HAL_DMA_DISABLE_IT(uart3_485.get_handle()->hdmarx, DMA_IT_HT);
    }

    remoteController.SetDeadzone(66.0f);


    for (;;)
    {
        // 只在临界区内读取一份一致的遥控器快照；打包和 DMA 发送在任务中进行。
        bool remote_online;
        uint8_t s1;
        uint8_t s2;
        taskENTER_CRITICAL();
        remote_online = Gimbal485_RemoteOnline();
        s1 = remoteController.get_s1();
        s2 = remoteController.get_s2();
        // 看门狗复位后仅接受有效遥控双下解锁；已处于双下也可解锁。
        if (g_watchdog_rearm_required && remote_online &&
            s1 == BSP::REMOTE_CONTROL::RemoteController::DOWN &&
            s2 == BSP::REMOTE_CONTROL::RemoteController::DOWN)
        {
            g_watchdog_rearm_required = 0;
        }
        taskEXIT_CRITICAL();
        if (!remote_online)
        {
            remote.chassis_vx = 0.0f;
            remote.chassis_vy = 0.0f;
            remote.gimbal_yaw = 0.0f;
            remote.gimbal_pitch = 0.0f;
            remote.s1 = remote.s2 = 0;
            xQueueOverwrite(remoteDataQueue, &remote);

            // 遥控器失联保护：拨弹轮(1)和三路摩擦轮(2/3/4)立刻发0电流刹车
            friction_motor.setCAN(0, 1);  // 拨弹轮
            friction_motor.setCAN(0, 2);  // 左摩擦轮
            friction_motor.setCAN(0, 3);  // 右摩擦轮
            friction_motor.setCAN(0, 4);  // 下方摩擦轮
            friction_motor.sendCAN();
        }

        // 无 IMU 绝对姿态，停止向视觉发送姿态；USB 接收接口保留。

        Watchdog_RecordHeartbeat(WATCHDOG_TASK_REMOTE);
        vTaskDelay(2);
    }
}
