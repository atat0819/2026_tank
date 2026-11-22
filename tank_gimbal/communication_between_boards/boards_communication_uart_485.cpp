#include "boards_communication_uart_485.hpp"
#include "../../rs485_protocol.hpp"
#include "../RtosTask/remote_control_task.hpp"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

static volatile bool remote_online = false;
static volatile bool remote_received = false;
static volatile uint32_t remote_receive_tick = 0;
static volatile uint32_t remote_sample_tick = 0;
static Gimbal485YawMouseInput yaw_mouse_input;
static volatile float yaw_encoder_deg = 0;
static volatile bool yaw_received = false;
static volatile uint32_t yaw_receive_tick = 0;
static Gimbal485RefereeData referee_data;
static uint32_t referee_receive_tick = 0;

static bool RemoteOnline(uint32_t now)
{
    return remote_received && remote_online &&
           now - remote_receive_tick < 100U && now - remote_sample_tick < 100U;
}

bool Gimbal485_RemoteOnline()
{
    taskENTER_CRITICAL();
    const bool online = RemoteOnline(HAL_GetTick());
    taskEXIT_CRITICAL();
    return online;
}

Gimbal485YawMouseInput Gimbal485_ConsumeYawMouseInput()
{
    taskENTER_CRITICAL();
    const bool online = RemoteOnline(HAL_GetTick());
    if (!online)
        yaw_mouse_input = {};
    auto input = yaw_mouse_input;
    input.online = online;
    yaw_mouse_input.mouse_delta_x = 0;
    yaw_mouse_input.reset = false;
    yaw_mouse_input.reanchor = false;
    taskEXIT_CRITICAL();
    return input;
}

void Gimbal485_UpdateYawFromISR(float angle_deg)
{
    yaw_encoder_deg = angle_deg;
    yaw_receive_tick = HAL_GetTick();
    yaw_received = true;
}

void Gimbal485_GetRefereeData(Gimbal485RefereeData &data)
{
    taskENTER_CRITICAL();
    data = referee_data;
    data.age_ms = data.received ? HAL_GetTick() - referee_receive_tick : UINT32_MAX;
    taskEXIT_CRITICAL();
}

// 发送：收到有效底盘帧后回复 Yaw 偏移角，正前方编码器零点为 24 度。
void Gimbal485_Send()
{
    auto &uart3 = HAL::UART::get_uart_bus_instance().get_uart3();
    if (uart3.get_handle()->gState != HAL_UART_STATE_READY)
    {
        return;
    }

    static uint8_t tx_frame[BoardLink::kYawSize];
    const bool yaw_online = yaw_received && HAL_GetTick() - yaw_receive_tick < 100U;
    const float yaw_offset = yaw_online ? BoardLink::yawOffsetDeg(yaw_encoder_deg) : NAN;
    BoardLink::encodeYaw(tx_frame, yaw_offset);
    HAL::UART::Data tx{tx_frame, sizeof(tx_frame)};
    uart3.transmit_dma(tx);
}

// 解析底盘的 32 字节帧，直接将其中的 DBUS 原始数据交给遥控驱动。
void Gimbal485_Parse(const uint8_t *data, uint16_t size)
{
    static uint8_t frame[BoardLink::kChassisSize];
    static unsigned received = 0;
    static uint16_t last_remote_sequence = 0;
    static bool dbus_received = false;

    if (data == nullptr)
    {
        return;
    }

    for (uint16_t i = 0; i < size; ++i)
    {
        frame[received++] = data[i];
        if (received < sizeof(frame))
        {
            continue;
        }

        // 检查帧头和 CRC，失败时向后查找下一帧。
        BoardLink::ChassisData chassis_data{};
        if (!BoardLink::decodeChassis(frame, chassis_data))
        {
            memmove(frame, frame + 1, sizeof(frame) - 1);
            received = sizeof(frame) - 1;
            continue;
        }
        received = 0;

        if (chassis_data.remote_online && !BoardLink::dbusValid(chassis_data.dbus))
        {
            continue;
        }

        // 更新裁判系统数据和本次接收时间。
        referee_data.cooling_value = chassis_data.cooling;
        referee_data.heat_limit = chassis_data.heat_limit;
        referee_data.heat_42mm = chassis_data.heat_42mm;
        referee_data.referee_online = chassis_data.referee_online != 0U;
        referee_receive_tick = HAL_GetTick();
        referee_data.received = true;

        // 底盘可能多次发送同一帧，只在序号变化时解析，避免重复累计鼠标位移。
        const bool link_recovered = !remote_received || HAL_GetTick() - remote_receive_tick >= 100U;
        if (chassis_data.remote_online)
        {
            if (!dbus_received || chassis_data.remote_sequence != last_remote_sequence || link_recovered)
            {
                const uint32_t sample_tick = HAL_GetTick();
                remoteController.parseData(chassis_data.dbus);
                const bool mouse_active = chassis_data.yaw_mouse_active &&
                                          BoardLink::dbusYawMouseActive(chassis_data.dbus);
                if (!RemoteOnline(sample_tick))
                {
                    yaw_mouse_input = {};
                    yaw_mouse_input.reset = true;
                }
                if (mouse_active != yaw_mouse_input.mouse_active)
                {
                    yaw_mouse_input.mouse_delta_x = 0;
                    yaw_mouse_input.reanchor = mouse_active;
                }
                yaw_mouse_input.mouse_active = mouse_active;
                if (mouse_active)
                    yaw_mouse_input.mouse_delta_x +=
                        static_cast<int16_t>(BoardLink::get16(chassis_data.dbus + 6));

                remote_sample_tick = sample_tick;
                last_remote_sequence = chassis_data.remote_sequence;
                dbus_received = true;
            }
        }
        else
        {
            dbus_received = false;
            yaw_mouse_input = {};
        }
        remote_online = chassis_data.remote_online != 0U;
        remote_receive_tick = HAL_GetTick();
        remote_received = true;
        Gimbal485_Send();
    }
}
