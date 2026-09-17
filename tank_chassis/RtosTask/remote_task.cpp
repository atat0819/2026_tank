#include "remote_task.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "usart.h"
#include <string.h>
#include "BSP/IMU/HI12_imu.hpp"
#include "../user/core/BSP/Hi14/HI14.hpp"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include "../user/core/APP/Referee/RM_RefereeSystem.h"

// ================= 全局变量定义 =================
// 1. IMU 相关
BSP::IMU::HI12_float imu;    // 创建 IMU 对象
uint8_t power_rx_buffer[12];   // 功率计接收缓冲区
QueueHandle_t IMUDataQueue;  // 用于传递 IMU 数据的队列
IMUData_t imuData;           // IMU 数据结构体
PowerData_t PowerData;

// 2. 遥控器相关
uint8_t receivedata[18];     // 遥控器接收缓冲区
QueueHandle_t remoteDataQueue; // 用于传递遥控器数据的队列
RemoteData_t remoteData;     // 遥控器解析后的数据结构体
float a = 0;
uint8_t imu_rx_buffer[64];
extern DMA_HandleTypeDef hdma_usart10_rx;
static HAL::UART::Data uart10_rx_data;

BSP::REMOTE_CONTROL::RemoteController remoteController(100);
extern DMA_HandleTypeDef hdma_uart5_rx;   // 遥控器
extern DMA_HandleTypeDef hdma_uart7_rx;   // 功率计
extern DMA_HandleTypeDef hdma_usart1_rx;  // 裁判系统

// 3. 裁判系统相关
uint8_t referee_buffer[512];    // 裁判系统 DMA 接收缓冲区

// ================= UART 库静态缓冲区描述符 =================
// 对标 CAN 的 Frame 静态分配模式
static HAL::UART::Data uart5_rx_data;   // 遥控器 DMA+空闲接收
static HAL::UART::Data uart7_rx_data;   // 功率计 DMA+空闲接收
HAL::UART::Data uart1_rx_data;          // 裁判系统 DMA+空闲接收（非static，RM_RefereeSystemInit 需引用）

// ================= 中断回调函数（只做路由，不写业务逻辑） =================

/**
 * @brief UART DMA 接收完成/空闲中断回调函数
 * @note  1. 先重启 DMA（减少死区）再处理数据
 *        2. huart == get_handle() 防止设备查找失败时操作到错误的 UART
 *        3. 局部 Data 避免跨中断静态变量污染
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5) // 遥控器
    {
        HAL::UART::Data rx_data{receivedata, sizeof(receivedata)};
        auto &uart5 = HAL::UART::get_uart_bus_instance().get_uart5();
        if (huart == uart5.get_handle())
        {
            uart5.receive_dma_idle(rx_data);
            __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
            rx_data.size = Size;  // 实际接收字节数
            uart5.trigger_rx_callbacks(rx_data);
        }
    }
    else if (huart->Instance == UART7) // 功率计
    {
        HAL::UART::Data rx_data{power_rx_buffer, sizeof(power_rx_buffer)};
        auto &uart7 = HAL::UART::get_uart_bus_instance().get_uart7();
        if (huart == uart7.get_handle())
        {
            uart7.receive_dma_idle(rx_data);
            __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
            rx_data.size = Size;
            uart7.trigger_rx_callbacks(rx_data);
        }
    }
    else if (huart->Instance == USART1) // 裁判系统
    {
        HAL::UART::Data rx_data{referee_buffer, sizeof(referee_buffer)};
        auto &uart1 = HAL::UART::get_uart_bus_instance().get_uart1();
        if (huart == uart1.get_handle())
        {
            uart1.receive_dma_idle(rx_data);
            __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
            rx_data.size = Size;
            uart1.trigger_rx_callbacks(rx_data);
        }
    }
    else if (huart->Instance == USART10)
    {
        HAL::UART::Data rx_data{imu_rx_buffer, sizeof(imu_rx_buffer)};
        auto &uart10 = HAL::UART::get_uart_bus_instance().get_uart10();
        if (huart == uart10.get_handle())
        {
            uart10.receive_dma_idle(rx_data);
            __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);
            rx_data.size = Size;
            uart10.trigger_rx_callbacks(rx_data);
        }
    }
}

/**
 * @brief UART 错误回调
 * @note  完整错误恢复流程：清标志 → 解锁 HAL 状态机 → 重启 DMA 接收
 */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5) // 遥控器
    {
        // 1. 关闭 RXNE 中断，清除全部错误标志
        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        // 2. 强制解锁 HAL 状态机（解决 HAL_BUSY 导致 DMA 无法重启的关键）
        huart->RxState = HAL_UART_STATE_READY;
        huart->Lock = HAL_UNLOCKED;

        // 3. 重启 DMA+空闲接收
        HAL::UART::get_uart_bus_instance().get_uart5().receive_dma_idle(uart5_rx_data);
    }
    else if (huart->Instance == UART7) // 功率计
    {
        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        huart->RxState = HAL_UART_STATE_READY;
        huart->Lock = HAL_UNLOCKED;

        HAL::UART::get_uart_bus_instance().get_uart7().receive_dma_idle(uart7_rx_data);
    }
    else if (huart->Instance == USART1) // 裁判系统
    {
        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        huart->RxState = HAL_UART_STATE_READY;
        huart->Lock = HAL_UNLOCKED;

        HAL::UART::get_uart_bus_instance().get_uart1().receive_dma_idle(uart1_rx_data);
    }
    else if (huart->Instance == USART10)
    {
        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        huart->RxState = HAL_UART_STATE_READY;
        huart->Lock = HAL_UNLOCKED;

        HAL::UART::get_uart_bus_instance().get_uart10().receive_dma_idle(uart10_rx_data);
    }
}

//虚拟串口假回调 (见USB_DEVICE\App\usbd_cdc_if.c)
extern "C" void USB_Receive_Callback(uint8_t *Buf, uint32_t Len)
{
}


// ================= FreeRTOS 任务 =================

/**
 * @brief 遥控器与传感器监控任务
 */
extern "C" void remote_task(void *argument)
{
    // ---- 1. 初始化 UART 总线（懒加载单例） ----
    HAL::UART::get_uart_bus_instance();

    // ---- 2. 获取设备引用（对标 FDCAN 的 get_fdcan1() / get_fdcan2()） ----
    auto &uart5 = HAL::UART::get_uart_bus_instance().get_uart5();
    auto &uart7 = HAL::UART::get_uart_bus_instance().get_uart7();
    auto &uart10 = HAL::UART::get_uart_bus_instance().get_uart10();

    // ---- 3. 初始化参数 ----
    remoteController.SetDeadzone(0.0f);

    // ---- 4. UART5 遥控器：注册回调 + 启动 DMA 空闲接收 ----
    uart5_rx_data.buffer = receivedata;
    uart5_rx_data.size   = sizeof(receivedata);
    uart5.register_rx_callback([](const HAL::UART::Data &data) {
        if (data.size == 18)
        {
            remoteController.parseData(data.buffer);
            remoteController.updateTimestamp();
            remoteData.vx = remoteController.DeadzoneCompensation(remoteController.get_left_x());
            remoteData.vy = remoteController.DeadzoneCompensation(remoteController.get_left_y());
            remoteData.wz = remoteController.DeadzoneCompensation(remoteController.get_right_x());
            remoteData.s1 = remoteController.get_s1();
            remoteData.s2 = remoteController.get_s2();
        }
    });
    uart5.receive_dma_idle(uart5_rx_data);
    __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);

    // ---- 5. UART7 功率计：注册回调 + 启动 DMA 空闲接收 ----
    uart7_rx_data.buffer = power_rx_buffer;
    uart7_rx_data.size   = sizeof(power_rx_buffer);
    uart7.register_rx_callback([](const HAL::UART::Data &data) {
        if (data.size == 12)
        {
            memcpy(&PowerData, data.buffer, 12);
        }
    });
    uart7.receive_dma_idle(uart7_rx_data);
    __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);

    hi14.Init();
    uart10_rx_data.buffer = imu_rx_buffer;
    uart10_rx_data.size = sizeof(imu_rx_buffer);
    uart10.register_rx_callback([](const HAL::UART::Data &data) {
        if (data.buffer == nullptr)
        {
            return;
        }

        for (uint16_t i = 0; i < data.size; ++i)
        {
            hi14.UartRxCallBack(data.buffer[i]);
        }
    });
    uart10.receive_dma_idle(uart10_rx_data);
    __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);

    // ---- 6. USART1 裁判系统：注册回调 + 启动 DMA 空闲接收 ----
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_uart1();
    uart1_rx_data.buffer = referee_buffer;
    uart1_rx_data.size   = sizeof(referee_buffer);
    uart1.register_rx_callback([](const HAL::UART::Data &data) {
        if (data.size > 0 && data.buffer != nullptr)
        {
            RM_RefereeSystem::RM_RefereeSystemParse(data.buffer, data.size);
        }
    });
    uart1.receive_dma_idle(uart1_rx_data);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    // ---- 7. 主循环 ----
    for (;;)
    {
        // 裁判系统掉线检测（超时1000ms无数据判定掉线，内部会更新RM_RefereeSystemDirFlag并重连DMA）
        RM_RefereeSystem::RM_RefereeSystemDir();
        osDelay(1);
    }
}
