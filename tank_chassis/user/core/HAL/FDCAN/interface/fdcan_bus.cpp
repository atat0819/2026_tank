#include "fdcan_bus.hpp"
#include "../impl/fdcan_bus_impl.hpp"
namespace HAL::FDCAN { IFdcanBus &get_fdcan_bus_instance() { return FdcanBus::instance(); } }
