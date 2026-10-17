#include "remote_control_task.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart.h"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include "../user/core/BSP/version/vision_communication.hpp"
#include "../communication_between_boards/input_dispatcher.hpp"
#include "../communication_between_boards/refree_receive_uart_485.hpp"
#include "gimbal_task.hpp"

QueueHandle_t remoteDataQueue;
QueueHandle_t IMUDataQueue;

uint8_t receivedata[18];
RemoteData_t remote;
BSP::REMOTE_CONTROL::RemoteController remoteController(100);
BSP::IMU::HI12_float imu;

extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;

// IMU circular DMA receive
#define IMU_DMA_BUF_SIZE 164  // 两个 HI12 帧；半满、全满及 IDLE 时处理新增字节。
static uint8_t imu_dma_buffer[IMU_DMA_BUF_SIZE];
static uint16_t imu_dma_read_pos = 0U;
static volatile bool imu_rx_active = false;

static void StartImuReceive()
{
    auto &uart7 = HAL::UART::get_uart_bus_instance().get_uart7();
    imu_rx_active = false;
    if (HAL_UART_AbortReceive(uart7.get_handle()) != HAL_OK) return;
    __HAL_UART_CLEAR_OREFLAG(uart7.get_handle());
    imu.ResetReceiveBuffer();
    imu_dma_read_pos = 0U;
    HAL::UART::Data rx{imu_dma_buffer, IMU_DMA_BUF_SIZE};
    imu_rx_active = uart7.receive_dma_idle(rx); // 启动失败时任务循环会重试。
}

// End IMU circular DMA receive

// 视觉通信实例（文件作用域，HAL 回调需要访问其 RX 缓冲区）
BSP::Vision::VisionCommunicator vision_comm;

// ==================== HAL UART 回调 ====================

// UART7 (IMU) 改用 DMA+Idle，不再使用逐字节 IT，此回调不再处理 IMU
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

    // UART7 — IMU DMA+Idle 连续接收，Size 是环形缓冲区写入位置。
    auto &uart7 = HAL::UART::get_uart_bus_instance().get_uart7();
    if (huart == uart7.get_handle())
    {
        if (!imu_rx_active || Size == 0U || Size > IMU_DMA_BUF_SIZE || Size == imu_dma_read_pos) return;
        if (Size < imu_dma_read_pos)
        {
            // 循环 DMA 回绕：先处理缓冲区尾部，再处理头部。
            HAL::UART::Data tail{imu_dma_buffer + imu_dma_read_pos,
                static_cast<uint16_t>(IMU_DMA_BUF_SIZE - imu_dma_read_pos)};
            if (tail.size != 0U) uart7.trigger_rx_callbacks(tail);
            imu_dma_read_pos = 0U;
        }
        HAL::UART::Data rx{imu_dma_buffer + imu_dma_read_pos,
            static_cast<uint16_t>(Size - imu_dma_read_pos)};
        uart7.trigger_rx_callbacks(rx);
        imu_dma_read_pos = Size; // 相同位置的 HT/TC/IDLE 回调不重复解析。
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
    auto &uart7 = HAL::UART::get_uart_bus_instance().get_uart7();

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
    else if (huart == uart7.get_handle())
    {
        // HAL 已处理接收错误；交给任务终止残留 DMA 并重新启动，失败会继续重试。
        imu_rx_active = false;
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
    auto &uart7 = HAL::UART::get_uart_bus_instance().get_uart7();

    // UART2 — RS485 第二路，预留接收数据处理。
    uart2_485.register_rx_callback([](const HAL::UART::Data &data) {
        (void)data;
        // 在这里添加接收数据处理。
    });

    // UART3 — RS485 第一路，接收底盘的裁判数据回复。
    uart3_485.register_rx_callback([](const HAL::UART::Data &data) {
        Communication::GimbalRefree485::instance().parse485(data.buffer, data.size);
    });

    // UART5 回调：DMA+空闲 → 遥控器解析
    uart5.register_rx_callback([](const HAL::UART::Data &data) {
        if (data.size == 18) {
            remoteController.parseData(data.buffer);
            remoteController.updateTimestamp();
        }
    });

    // UART7 回调：DMA 新增字节 → HI12 驱动跨回调拼帧。
    uart7.register_rx_callback([](const HAL::UART::Data &data) {
        imu.DataUpdate(data.buffer, data.size);
    });

    // 启动接收
    // UART3 板间接收区为 64 字节；static 保证 DMA 使用期间一直有效。
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

    HAL::UART::Data uart5_rx{receivedata, sizeof(receivedata)};
    uart5.receive_dma_idle(uart5_rx);
    __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);

    taskENTER_CRITICAL();
    StartImuReceive(); // 循环 DMA 保留半满中断，正常接收期间不重启。
    taskEXIT_CRITICAL();

    remoteController.SetDeadzone(66.0f);

    static uint32_t vision_tick = 0;

    for (;;)
    {
        // UART7/DMA 中断优先级均为 5，临界区避免恢复操作与接收回调交错。
        taskENTER_CRITICAL();
        if (!imu_rx_active) StartImuReceive();
        taskEXIT_CRITICAL();
        if (!remoteController.isConnected())
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

        //if (imu.isConnected())
        {
            // 视觉模式：遥控器 S1上+S2上，或键鼠右键按住 ≥2s
            uint8_t vision_mode = ((remoteController.get_s1() == BSP::REMOTE_CONTROL::RemoteController::UP &&
                                    remoteController.get_s2() == BSP::REMOTE_CONTROL::RemoteController::UP) ||
                                   input_dispatcher.IsVisionMode()) ? 1 : 0;

            float quaternion[4] = {0.0f, 0.0f, 0.0f, 1.0f};

            // 弹丸预估初速度：取2个摩擦轮转速的绝对值平均值，换算为线速度
            // v (m/s) = RPM × π × d / 60,  d = 64.1mm
            // 右侧摩擦轮反向旋转，取绝对值后参与平均
            float abs_left  = friction_current_speed_left  < 0.0f ? -friction_current_speed_left  : friction_current_speed_left;
            float abs_right = friction_current_speed_right < 0.0f ? -friction_current_speed_right : friction_current_speed_right;
            float abs_bottom = friction_current_speed_bottom < 0.0f ? -friction_current_speed_bottom : friction_current_speed_bottom;
            float avg_friction_rpm = (abs_left + abs_right + abs_bottom) / 3.0f;
            float bullet_speed = avg_friction_rpm * 3.14159265358979323846f * 0.0641f / 60.0f;

            constexpr float deg_to_rad = 3.14159265358979323846f / 180.0f;           
						vision_comm.SendToVision(quaternion,
                                     imu.GetAngle(2) * deg_to_rad,
                                     imu.GetGyro(2) * deg_to_rad,
                                     imu.GetAngle(1) * deg_to_rad,
                                     imu.GetGyro(0) * deg_to_rad,
                                     16.0f, 0U, vision_mode); // 暂不统计累计发弹数

        }
       

        vTaskDelay(2);
    }
}
