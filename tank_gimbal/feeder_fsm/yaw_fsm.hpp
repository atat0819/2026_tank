#ifndef YAW_FSM_HPP
#define YAW_FSM_HPP

#include "../../rs485_protocol.hpp"
#include <stdint.h>

struct Struct_Yaw_Input
{
    bool enabled = false;       // 当前输入是否允许控制 Yaw，false 时状态机复位。
    bool is_keymouse = false;   // true 为双中键鼠挡，false 为遥控挡。
    bool mouse_active = false;  // 键鼠挡下鼠标 X 是否分配给 Yaw，即右键按住。
    bool reanchor = false;      // 鼠标重新接管时，先以当前编码器角重建目标。
    int32_t mouse_delta_x = 0;  // 本轮消费的鼠标 X 累计位移，单位为鼠标计数。
};

// 图传相机 Yaw，所有角度使用电机编码器的绝对角度，单位 rad。
class Yaw_FSM
{
public:
    // Stop 无有效目标；Mouse 鼠标控制；Center 向中位回归。
    enum class Mode : uint8_t { Stop, Mouse, Center };
    // 回中目标推进速度，同时也是任务中回中目标速度的限幅，单位 rad/s。
    static constexpr float CENTER_SPEED_RAD_S = 15.0f * (3.14159265358979323846f / 180.0f);

    // 清除状态和目标角；实际发帧由调用任务负责。
    void Reset();
    // input 为本周期输入，current_angle 为编码器角(rad)，now_tick 为 HAL 毫秒时间戳。
    void Update(const Struct_Yaw_Input &input, float current_angle, uint32_t now_tick);
    Mode Get_Mode() const { return mode_; }
    // 返回状态机目标角，单位 rad；Stop 时不能作为有效控制目标。
    float Get_Target_Angle() const { return target_angle_; }
    bool Is_Target_Valid() const { return mode_ != Mode::Stop; }
    // torque_nm 为待保护的估算力矩(N·m)，current_angle 为编码器角(rad)。
    float Limit_Torque(float torque_nm, float current_angle) const;
    // 反馈位于物理端点及容差范围内时返回 true。
    static bool Is_Feedback_In_Travel(float current_angle);
    // 编码器角到达或超过目标软限位时返回 true。
    static bool Is_Soft_Limit_Active(float current_angle);

private:
    static constexpr float kDegToRad = 0.017453292519943295f; // 角度转弧度系数。
    static constexpr float kLeftEndDeg = 19.0f;               // 左侧实测端点，编码器角度(deg)。
    static constexpr float kRightEndDeg = 29.0f;              // 右侧实测端点，编码器角度(deg)。
    static constexpr float kLimitMarginDeg = 0.5f;             // 目标软限位距离各端点的余量(deg)。
    static constexpr float kFeedbackToleranceDeg = 0.1f;      // 物理端点之外允许的反馈容差(deg)。
    static constexpr float kMinAngleRad = (kLeftEndDeg + kLimitMarginDeg) * kDegToRad;  // 目标下限(rad)。
    static constexpr float kMaxAngleRad = (kRightEndDeg - kLimitMarginDeg) * kDegToRad; // 目标上限(rad)。
    static constexpr float kCenterAngleRad = BoardLink::kYawForwardDeg * kDegToRad;    // 正前方目标角，当前 24°。
    static constexpr float kMouseRadPerCount = 0.025f * kDegToRad; // 鼠标 X 每计数改变的目标角(rad)。
    static constexpr float kMaxTorqueNm = 7.0f;     // 经过 Limit_Torque 后的力矩幅值上限(N·m)。
    static_assert(kMinAngleRad < kCenterAngleRad && kCenterAngleRad < kMaxAngleRad,
                  "Camera Yaw center must be inside the soft limits");

    Mode mode_ = Mode::Stop;       // 当前控制状态。
    float target_angle_ = 0.0f;   // 当前目标编码器角(rad)。
    uint32_t last_tick_ = 0U;     // 上次 Update 的 HAL 时间戳(ms)。
};

#endif
