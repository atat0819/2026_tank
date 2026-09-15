#include "fdcan_bus_impl.hpp"
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
namespace HAL::FDCAN {
FdcanBus::FdcanBus() : devices_{ FdcanDevice(&hfdcan1,0), FdcanDevice(&hfdcan2,0), FdcanDevice(&hfdcan3,0) } {}
FdcanBus &FdcanBus::instance() { static FdcanBus b; if (!b.initialized_) { for (int i=0;i<3;++i) { b.devices_[i].init(); b.devices_[i].start(); } b.initialized_=true; } return b; }
IFdcanDevice &FdcanBus::get_device(FdcanDeviceId id) { auto i=(size_t)id; return (i<3)?devices_[i]:devices_[0]; }
bool FdcanBus::has_device(FdcanDeviceId id) const { return (size_t)id<3; }
}
