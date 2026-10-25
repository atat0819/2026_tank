#pragma once
#include <cstdint>

enum class GantryState : uint8_t
{
    Disabled, Idle, LiftFirst, Extend, LiftSecond, Retract, Lower, Fault
};

enum class GantryFault : uint8_t
{
    None, NotHome, Feedback, Timeout, Travel, LiftMismatch, SwitchSequence
};

class Class_Gantry_FSM
{
public:
    // 限位依次为 PA0..4；角度依次为 ID5、ID6、DM ID9 的累计角度，deg。
    void Update(bool control_enabled, bool feedback_ok,
                bool right_raw, bool right_pressed,
                bool lift_first_limit, bool extend_limit, bool lift_second_limit,
                bool retract_limit, bool lower_limit,
                double lift_id5_angle_deg, double lift_id6_angle_deg, double dm_angle_deg,
                uint32_t now_ms);
    static bool ConfigValid();
    GantryState GetState() const { return state_; }
    GantryFault GetFault() const { return fault_; }
    // 三轴查询下标均为 0=ID5、1=ID6、2=DM ID9。
    bool IsActive() const { return active_; } // false 时任务清零输出并复位 PID
    bool IsMoving(unsigned axis) const { return moving_[axis]; }
    bool IsKeeping(unsigned axis) const { return active_ && !moving_[axis]; } // 该轴主动保持位置
    float GetTargetSpeed(unsigned axis) const { return target_speed_rad_s_[axis]; } // rad/s
    double GetHoldAngle(unsigned axis) const { return active_ ? hold_deg_[axis] : 0; } // deg
    float GetProgress() const { return progress_; }
    float GetScale() const { return active_ && Running() ? scale_ : 1; }

private:
    bool Running() const;
    void Enter(GantryState state, uint32_t now_ms,
               const double (&angle_deg)[3], const bool (&limit_pressed)[5]);
    void Fail(GantryFault fault);
    GantryState state_ = GantryState::Disabled;
    GantryFault fault_ = GantryFault::None;
    bool start_armed_ = false;
    bool holding_ = false;
    uint32_t entered_ms_ = 0;
    double start_deg_[3] = {};
    double hold_deg_[3] = {};
    float scale_ = 1;
    bool active_ = false;
    bool moving_[3] = {};
    float target_speed_rad_s_[3] = {};
    float progress_ = 0;
};

// 龙门架运动标定和 DM 控制参数集中于此；角度外环和 3508 速度 PID 在 gantry_task.cpp 中填写。
// 零值只是占位，不是可运行参数。
// 先读取 gantry_watch.angle_deg 手动记录，再填写本文件并重新编译。
namespace GantryTuning
{
static constexpr bool CALIBRATED = false; // 完成标定和控制参数整定后才改 true

// 同一次连续反馈期间记录的累计角度，单位 deg；3508 为减速比 19 的输出轴端，保留正负号。
static constexpr double LIFT_ID5_HOME_DEG = 0.0;   // ID5 升降原点
static constexpr double LIFT_ID6_HOME_DEG = 0.0;   // ID6 升降原点
static constexpr double LIFT_ID5_FIRST_DEG = 0.0;  // ID5 第一次抬升终点
static constexpr double LIFT_ID6_FIRST_DEG = 0.0;  // ID6 第一次抬升终点
static constexpr double LIFT_ID5_SECOND_DEG = 0.0; // ID5 第二次抬升终点
static constexpr double LIFT_ID6_SECOND_DEG = 0.0; // ID6 第二次抬升终点
static constexpr double DM_RETRACT_DEG = 0.0;      // DM 后收终点
static constexpr double DM_EXTEND_DEG = 0.0;       // DM 前伸终点

// 正的速度幅值，方向由标定行程差决定；3508 减速比为 19，使用输出轴端 rad/s。
static constexpr float LIFT_ID5_SPEED_RAD_S = 0.0f; // ID5 升降速度
static constexpr float LIFT_ID6_SPEED_RAD_S = 0.0f; // ID6 升降速度
static constexpr float DM_SPEED_RAD_S = 0.0f;       // DM 伸缩速度，不超过 30 rad/s

static constexpr uint32_t LIFT_FIRST_TIMEOUT_MS = 0;  // 第一次抬升超时，ms
static constexpr uint32_t EXTEND_TIMEOUT_MS = 0;      // 前伸超时，ms
static constexpr uint32_t LIFT_SECOND_TIMEOUT_MS = 0; // 第二次抬升超时，ms
static constexpr uint32_t RETRACT_TIMEOUT_MS = 0;     // 后收超时，ms
static constexpr uint32_t LOWER_TIMEOUT_MS = 0;       // 下降超时，ms
static constexpr float TRAVEL_MARGIN = 0.10f;        // 超出行程或反向走 10% 判故障
static constexpr float LIFT_MISMATCH = 0.10f;        // 双升降归一化进度差允许 10%

static constexpr float DM_MOVE_KD = 0.0f;      // DM 前伸/后收 MIT 速度阻尼，Nm/(rad/s)，>0 且 <=5
static constexpr float DM_HOLD_KD = 0.0f;      // DM 位置保持 MIT 速度阻尼，Nm/(rad/s)，>0 且 <=5
static constexpr float DM_TORQUE_LIMIT_NM = 0.0f; // DM 总阻尼力矩限幅，>0 且 <=3 Nm
}
