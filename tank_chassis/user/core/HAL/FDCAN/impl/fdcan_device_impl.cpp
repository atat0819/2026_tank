#include "../interface/fdcan_device.hpp"
#include <vector>

namespace HAL::FDCAN {

static uint32_t to_dlc(uint8_t n) {
    switch (n) {
    case 0: return FDCAN_DLC_BYTES_0; case 1: return FDCAN_DLC_BYTES_1;
    case 2: return FDCAN_DLC_BYTES_2; case 3: return FDCAN_DLC_BYTES_3;
    case 4: return FDCAN_DLC_BYTES_4; case 5: return FDCAN_DLC_BYTES_5;
    case 6: return FDCAN_DLC_BYTES_6; case 7: return FDCAN_DLC_BYTES_7;
    default: return FDCAN_DLC_BYTES_8;
    }
}

static uint8_t from_dlc(uint32_t dlc) {
    switch (dlc) {
    case FDCAN_DLC_BYTES_1: return 1; case FDCAN_DLC_BYTES_2: return 2;
    case FDCAN_DLC_BYTES_3: return 3; case FDCAN_DLC_BYTES_4: return 4;
    case FDCAN_DLC_BYTES_5: return 5; case FDCAN_DLC_BYTES_6: return 6;
    case FDCAN_DLC_BYTES_7: return 7; case FDCAN_DLC_BYTES_8: return 8;
    default: return 0;
    }
}

FdcanDevice::FdcanDevice(FDCAN_HandleTypeDef *handle, uint32_t filter_index, uint32_t fifo)
    : handle_(handle), filter_index_(filter_index), fifo_(fifo) {}

void FdcanDevice::init() { configure_filter(); }

void FdcanDevice::start() {
    if (!handle_) return;
    HAL_FDCAN_Start(handle_);
    HAL_FDCAN_ActivateNotification(handle_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

bool FdcanDevice::send(const Frame &frame) {
    if (!handle_ || frame.dlc > 8) return false;
    FDCAN_TxHeaderTypeDef h{};
    h.Identifier = frame.id;
    h.IdType = frame.is_extended_id ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    h.TxFrameType = frame.is_remote_frame ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    h.DataLength = to_dlc(frame.dlc);
    h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    h.BitRateSwitch = FDCAN_BRS_OFF;
    h.FDFormat = FDCAN_CLASSIC_CAN;
    h.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    h.MessageMarker = 0;
    return HAL_FDCAN_AddMessageToTxFifoQ(handle_, &h, frame.data) == HAL_OK;
}

bool FdcanDevice::receive(Frame &frame) {
    if (!handle_ || HAL_FDCAN_GetRxFifoFillLevel(handle_, fifo_) == 0) return false;
    FDCAN_RxHeaderTypeDef h{};
    if (HAL_FDCAN_GetRxMessage(handle_, fifo_, &h, frame.data) != HAL_OK) return false;
    frame.id = h.Identifier;
    frame.dlc = from_dlc(h.DataLength);
    frame.is_extended_id = (h.IdType == FDCAN_EXTENDED_ID);
    frame.is_remote_frame = (h.RxFrameType == FDCAN_REMOTE_FRAME);
    trigger_rx_callbacks(frame);
    return true;
}

void FdcanDevice::configure_filter() {
    if (!handle_) return;
    FDCAN_FilterTypeDef f{};
    f.IdType = FDCAN_STANDARD_ID;
    f.FilterIndex = filter_index_;
    f.FilterType = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1 = 0;
    f.FilterID2 = 0;
    HAL_FDCAN_ConfigFilter(handle_, &f);
    HAL_FDCAN_ConfigGlobalFilter(handle_, FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_REJECT_REMOTE,
                                 FDCAN_REJECT_REMOTE);
}

void FdcanDevice::register_rx_callback(RxCallback callback) {
    if (callback) callbacks_.push_back(callback);
}

void FdcanDevice::trigger_rx_callbacks(const Frame &frame) {
    for (auto &cb : callbacks_) if (cb) cb(frame);
}

} // namespace HAL::FDCAN
