/**
 * @file gimbal_refree.cpp
 * @brief 裁判系统数据 → 云台转发类实现 (英雄机器人)
 * @version 0.0.1
 * @date 2025-07-16
 */

#include "gimbal_refree.hpp"
#include "HAL/FDCAN/fdcan_hal.hpp"

namespace Communication
{

HAL_StatusTypeDef GimbalRefree::send(uint16_t cooling_value,
                                      uint16_t heat_limit,
                                      uint16_t heat_42mm,
                                      uint8_t ref_online)
{
    HAL::FDCAN::Frame frame = {};

    frame.id              = TX_ID;
    frame.dlc             = 8;
    frame.is_extended_id  = false;
    frame.is_remote_frame = false;

    // Byte 0-1: 枪管每秒冷却值 (uint16_t 大端)
    frame.data[0] = (cooling_value >> 8) & 0xFF;
    frame.data[1] = cooling_value & 0xFF;

    // Byte 2-3: 枪管热量上限 (uint16_t 大端)
    frame.data[2] = (heat_limit >> 8) & 0xFF;
    frame.data[3] = heat_limit & 0xFF;

    // Byte 4-5: 42mm枪管当前热量 (uint16_t 大端)
    frame.data[4] = (heat_42mm >> 8) & 0xFF;
    frame.data[5] = heat_42mm & 0xFF;

    // Byte 6: 裁判系统在线标志 (1=在线 0=掉线)
    frame.data[6] = ref_online ? 1 : 0;

    // Byte 7: 保留
    frame.data[7] = 0;

    auto &fdcan_bus = HAL::FDCAN::get_fdcan_bus_instance();
    bool success    = fdcan_bus.get_device(HAL::FDCAN::FdcanDeviceId::HAL_Fdcan2).send(frame);

    return success ? HAL_OK : HAL_ERROR;
}

} // namespace Communication
