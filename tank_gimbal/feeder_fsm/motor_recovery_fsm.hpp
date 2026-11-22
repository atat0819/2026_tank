#ifndef MOTOR_RECOVERY_FSM_HPP
#define MOTOR_RECOVERY_FSM_HPP

#include <stdint.h>

// 根据遥控停车条件与新鲜驱动状态，限频请求 On 或 Off。
class MotorRecoveryFSM
{
public:
    enum class Request : uint8_t { None, On, Off };
    static constexpr uint32_t ENABLE_RETRY_MS = 100U;
    static constexpr uint8_t UNKNOWN_DRIVE_STATUS = 0xFFU;

    void Reset() { last_request_ = Request::None; }
    bool HasPendingOn() const { return last_request_ == Request::On; }

    Request Update(bool control_allowed, uint8_t drive_status, uint32_t now_tick)
    {
        Request request = Request::None;
        if (!control_allowed)
            request = Request::Off;
        else if (drive_status == 0U)
            request = Request::On;

        // 请求切换时立即发送，同类请求持续时按周期补发。
        if (request != last_request_ || now_tick - last_request_tick_ >= ENABLE_RETRY_MS)
        {
            last_request_ = request;
            last_request_tick_ = now_tick;
            return request;
        }
        return Request::None;
    }

private:
    Request last_request_ = Request::None;
    uint32_t last_request_tick_ = 0U;
};

#endif // MOTOR_RECOVERY_FSM_HPP
