#include "boards_communication_uart_485.hpp"
#include "../../rs485_protocol.hpp"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

static uint16_t cooling_value = 0;
static uint16_t heat_limit = 0;
static uint16_t heat_42mm = 0;
static uint8_t referee_online = 0;
static uint8_t remote_frame[BoardLink::kDbusSize];
static uint16_t remote_sequence = 0;
static uint32_t remote_receive_tick = 0;
static bool remote_received = false;
static uint8_t rx_frame[BoardLink::kYawSize];
static unsigned rx_size = 0;

// 发送短暂繁忙时按顺序保存原始帧，避免新帧覆盖尚未发送的鼠标或按键数据。
struct PendingRemoteFrame
{
    uint8_t dbus[BoardLink::kDbusSize];
    uint16_t sequence;
    uint32_t receive_tick;
};
static constexpr unsigned kRemoteQueueSize = 8;
static PendingRemoteFrame remote_queue[kRemoteQueueSize];
static unsigned remote_queue_head = 0;
static unsigned remote_queue_count = 0;
static volatile uint32_t remote_queue_dropped = 0; // Keil 可观察：缓存满或数据过期时增加。


// UART5 接收到并校验遥控帧后调用；保存副本，避免 DMA 再次接收时覆盖。
void Chassis485_UpdateRemote(const uint8_t *dbus)
{
    if (dbus == nullptr) return;
    memcpy(remote_frame, dbus, sizeof(remote_frame));
    ++remote_sequence;
    remote_receive_tick = HAL_GetTick();
    remote_received = true;

    // 容量有限；满时丢弃最旧一帧并计数，保留最近的控制输入。
    if (remote_queue_count == kRemoteQueueSize)
    {
        remote_queue_head = (remote_queue_head + 1U) % kRemoteQueueSize;
        --remote_queue_count;
        ++remote_queue_dropped;
    }
    const unsigned tail = (remote_queue_head + remote_queue_count) % kRemoteQueueSize;
    memcpy(remote_queue[tail].dbus, dbus, BoardLink::kDbusSize);
    remote_queue[tail].sequence = remote_sequence;
    remote_queue[tail].receive_tick = remote_receive_tick;
    ++remote_queue_count;
}

void Chassis485_UpdateReferee(uint16_t cooling, uint16_t limit,
                              uint16_t heat, uint8_t online)
{
    taskENTER_CRITICAL();
    cooling_value = cooling;
    heat_limit = limit;
    heat_42mm = heat;
    referee_online = online ? 1U : 0U;
    taskEXIT_CRITICAL();
}

// 发送：底盘每隔至少 2 ms，向云台发送遥控器、键鼠和裁判系统数据。
void Chassis485_Send()
{
    auto &uart3 = HAL::UART::get_uart_bus_instance().get_uart3();
    static uint8_t tx_frame[BoardLink::kChassisSize];
    static uint32_t last_send_tick = 0;
    const uint32_t now = HAL_GetTick();

    if (now - last_send_tick < 2U || uart3.get_handle()->gState != HAL_UART_STATE_READY)
    {
        return;
    }

    taskENTER_CRITICAL();
    BoardLink::ChassisData data{};
    const uint32_t sample_tick = HAL_GetTick();
    data.remote_online = remote_received && sample_tick - remote_receive_tick < 100U;

    // 不重放超过 100 ms 的旧操作；遥控失联时清空积压。
    while (remote_queue_count != 0U &&
           (!data.remote_online || sample_tick - remote_queue[remote_queue_head].receive_tick >= 100U))
    {
        remote_queue_head = (remote_queue_head + 1U) % kRemoteQueueSize;
        --remote_queue_count;
        ++remote_queue_dropped;
    }
    const bool send_queued_frame = data.remote_online && remote_queue_count != 0U;
    data.remote_sequence = remote_sequence;
    if (send_queued_frame)
    {
        memcpy(data.dbus, remote_queue[remote_queue_head].dbus, BoardLink::kDbusSize);
        data.remote_sequence = remote_queue[remote_queue_head].sequence;
    }
    else if (data.remote_online)
    {
        memcpy(data.dbus, remote_frame, sizeof(remote_frame));
    }
    data.yaw_mouse_active = data.remote_online && BoardLink::dbusYawMouseActive(data.dbus);
    data.cooling = cooling_value;
    data.heat_limit = heat_limit;
    data.heat_42mm = heat_42mm;
    data.referee_online = referee_online;

    BoardLink::encodeChassis(tx_frame, data);
    HAL::UART::Data tx{tx_frame, sizeof(tx_frame)};
    if (uart3.transmit_dma(tx))
    {
        if (send_queued_frame)
        {
            remote_queue_head = (remote_queue_head + 1U) % kRemoteQueueSize;
            --remote_queue_count;
        }
        last_send_tick = now;
    }
    taskEXIT_CRITICAL();
}

// 解析：接收云台回复的 8 字节 Yaw 帧，校验通过后保存偏移角。
void Chassis485_Parse(const uint8_t *data, uint16_t size)
{
    if (data == nullptr) return;

    for (uint16_t i = 0; i < size; ++i)
    {
        rx_frame[rx_size++] = data[i];
        if (rx_size < sizeof(rx_frame)) continue;

        float yaw_offset;
        if (!BoardLink::decodeYaw(rx_frame, yaw_offset))
        {
            memmove(rx_frame, rx_frame + 1, sizeof(rx_frame) - 1);
            rx_size = sizeof(rx_frame) - 1;
            continue;
        }
        rx_size = 0;
        if (isfinite(yaw_offset) && yaw_offset >= -180.0f && yaw_offset < 180.0f)
        {
            Chassis485_ApplyData({yaw_offset});
        }
    }
}

void Chassis485_ResetRx()
{
    rx_size = 0;
}
