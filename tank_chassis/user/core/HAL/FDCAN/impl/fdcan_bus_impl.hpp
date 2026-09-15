#pragma once
#include "../interface/fdcan_bus.hpp"

namespace HAL::FDCAN {
class FdcanBus : public IFdcanBus {
public:
    static FdcanBus &instance();
    IFdcanDevice &get_device(FdcanDeviceId id) override;
    bool has_device(FdcanDeviceId id) const override;
private:
    FdcanBus();
    bool initialized_ = false;
    FdcanDevice devices_[3];
};
}
