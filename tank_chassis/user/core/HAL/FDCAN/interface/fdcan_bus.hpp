#pragma once
#include "fdcan_device.hpp"

namespace HAL::FDCAN {

enum class FdcanDeviceId : uint8_t { HAL_Fdcan1 = 0, HAL_Fdcan2, HAL_Fdcan3, HAL_Can1 = HAL_Fdcan1, HAL_Can2 = HAL_Fdcan2, HAL_Can3 = HAL_Fdcan3, MAX_DEVICES };

class IFdcanBus {
public:
    virtual ~IFdcanBus() = default;
    virtual IFdcanDevice &get_device(FdcanDeviceId id) = 0;
    IFdcanDevice &get_can1() { return get_device(FdcanDeviceId::HAL_Fdcan1); }
    IFdcanDevice &get_can2() { return get_device(FdcanDeviceId::HAL_Fdcan2); }
    IFdcanDevice &get_can3() { return get_device(FdcanDeviceId::HAL_Fdcan3); }
    virtual bool has_device(FdcanDeviceId id) const = 0;
};

using ICanBus = IFdcanBus;
using CanDeviceId = FdcanDeviceId;

IFdcanBus &get_fdcan_bus_instance();
inline IFdcanBus &get_can_bus_instance() { return get_fdcan_bus_instance(); }

} // namespace HAL::FDCAN
