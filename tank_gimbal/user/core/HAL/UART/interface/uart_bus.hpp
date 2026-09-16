#pragma once
#include "uart_device.hpp"
#include <cstdint>

namespace HAL::UART
{

// UART设备ID枚举
enum class UartDeviceId : uint8_t
{
    HAL_Uart1 = 0,
    HAL_Uart5 = 1,
    HAL_Uart7 = 2,
    HAL_Uart10 = 3,
    // 可以在此处添加更多UART设备，无需修改接口
    MAX_DEVICES
};

// UART总线抽象接口
class IUartBus
{
  public:
    virtual ~IUartBus() = default;

    // 获取指定ID的UART设备
    virtual IUartDevice &get_device(UartDeviceId id) = 0;

    // 检查指定ID的设备是否存在
    virtual bool has_device(UartDeviceId id) const = 0;

    // 便捷访问方法（对标 FDCAN 的 get_fdcan1() / get_fdcan2()）
    IUartDevice &get_uart1() { return get_device(UartDeviceId::HAL_Uart1); }
    IUartDevice &get_uart5() { return get_device(UartDeviceId::HAL_Uart5); }
    IUartDevice &get_uart7() { return get_device(UartDeviceId::HAL_Uart7); }
    IUartDevice &get_uart10() { return get_device(UartDeviceId::HAL_Uart10); }
};

// 获取UART总线单例实例
IUartBus &get_uart_bus_instance();

} // namespace HAL::UART