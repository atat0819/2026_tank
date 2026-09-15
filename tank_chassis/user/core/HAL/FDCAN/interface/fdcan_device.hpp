#pragma once

#include "fdcan.h"
#include <functional>
#include <vector>

namespace HAL::FDCAN {

using ID_t = uint32_t;

struct Frame {
    uint8_t data[8]{};
    ID_t id = 0;
    uint8_t dlc = 0;
    uint32_t mailbox = 0;
    bool is_extended_id = false;
    bool is_remote_frame = false;
};

using RxCallback = std::function<void(const Frame &)>;

class IFdcanDevice {
public:
    virtual ~IFdcanDevice() = default;
    virtual void init() = 0;
    virtual void start() = 0;
    virtual bool send(const Frame &frame) = 0;
    virtual bool receive(Frame &frame) = 0;
    virtual FDCAN_HandleTypeDef *get_handle() const = 0;
    virtual void register_rx_callback(RxCallback callback) = 0;
    virtual void trigger_rx_callbacks(const Frame &frame) = 0;
};

class FdcanDevice : public IFdcanDevice {
public:
    explicit FdcanDevice(FDCAN_HandleTypeDef *handle,
                         uint32_t filter_index = 0,
                         uint32_t fifo = FDCAN_RX_FIFO0);
    FdcanDevice(const FdcanDevice &) = delete;
    FdcanDevice &operator=(const FdcanDevice &) = delete;

    void init() override;
    void start() override;
    bool send(const Frame &frame) override;
    bool receive(Frame &frame) override;
    FDCAN_HandleTypeDef *get_handle() const override { return handle_; }
    void register_rx_callback(RxCallback callback) override;
    void trigger_rx_callbacks(const Frame &frame) override;

private:
    FDCAN_HandleTypeDef *handle_;
    uint32_t filter_index_;
    uint32_t fifo_;
    std::vector<RxCallback> callbacks_;
    void configure_filter();
};

using ICanDevice = IFdcanDevice;
using CanDevice = FdcanDevice;

} // namespace HAL::FDCAN

// Compatibility alias: existing motor and communication code uses HAL::CAN.
// The implementation is backed by the FDCAN adapter above.

namespace HAL { namespace CAN = FDCAN; }
