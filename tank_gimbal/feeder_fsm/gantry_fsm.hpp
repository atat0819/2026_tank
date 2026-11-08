#pragma once
#include <cstdint>

enum class GantryState : uint8_t
{
    Disabled, Idle, LiftFirst, Extend, LiftSecond, Retract, Lower, Fault
};

enum class GantryFault : uint8_t
{
    None, NotHome, Feedback, Timeout, Travel, LiftMismatch, SwitchSequence, Stall
};

class Class_Gantry_FSM
{
public:
    // 限位依次为 PE13、PE9、PA2、PA3、PD10；角度为 ID5、ID6、DM ID9 的累计角度 deg。
    // 两台 3508 的反馈电流为 A，速度为输出轴端 rad/s。
    void Update(bool control_enabled, bool feedback_ok, bool lift_feedback_ok,
                bool right_raw, bool right_pressed,
                bool lift_first_limit, bool extend_limit, bool lift_second_limit,
                bool retract_limit, bool lower_limit,
                double lift_id5_angle_deg, double lift_id6_angle_deg, double dm_angle_deg,
                float lift_id5_current_a, float lift_id6_current_a,
                float lift_id5_speed_rad_s, float lift_id6_speed_rad_s,
                uint32_t now_ms);
    static bool ConfigValid();
    GantryState GetState() const { return state_; }
    GantryFault GetFault() const { return fault_; }
    // 三轴查询下标均为 0=ID5、1=ID6、2=DM ID9。
    bool IsActive() const { return active_; } // false 时任务清零输出并复位 PID
    bool IsMoving(unsigned axis) const { return moving_[axis]; }
    bool IsKeeping(unsigned axis) const {
        return active_ && !moving_[axis] && (axis == 2 || state_ != GantryState::Idle);
    } // 回到底部后两台 3508 零电流，DM 继续保持
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
    bool home_recorded_ = false; // 启动时机构已靠重力落底，只记录一次，不随切模式重置
    bool stall_pending_ = false;
    uint32_t stall_since_ms_ = 0;
    double home_deg_[2] = {};
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
// 行程和目标速度暂填 0；填写实际值并通过校验后才允许动作。
// 实测时读取 gantry_watch.angle_deg，再填写本文件并重新编译。
namespace GantryTuning
{
static constexpr bool CALIBRATED = true; // 行程和速度仍需填写有效值才能通过校验

// 行程标定使用 deg 和端点差值；3508 减速比 19，使用输出轴端角度。
static constexpr double LIFT_ID5_HOME_DEG = 0.0;    // ID5 原点
static constexpr double LIFT_ID6_HOME_DEG = 0.0;    // ID6 原点
static constexpr double LIFT_ID5_FIRST_DEG = 0.0;   // ID5 第一段终点
static constexpr double LIFT_ID6_FIRST_DEG = 0.0;   // ID6 第一段终点
static constexpr double LIFT_ID5_SECOND_DEG = 0.0;  // ID5 第二段终点
static constexpr double LIFT_ID6_SECOND_DEG = 0.0;  // ID6 第二段终点
static constexpr double DM_RETRACT_DEG = 0.0;       // DM 后收终点
static constexpr double DM_EXTEND_DEG = 0.0;        // DM 前伸终点

// 速度填写轴端 deg/s，经除以 180 再乘 π 转为 rad/s；方向由行程差决定。
static constexpr float LIFT_ID5_SPEED_RAD_S = 0.0f / 180.0f * 3.1415926f; // ID5 输出轴
static constexpr float LIFT_ID6_SPEED_RAD_S = 0.0f / 180.0f * 3.1415926f; // ID6 输出轴
static constexpr float DM_SPEED_RAD_S = 0.0f / 180.0f * 3.1415926f;        // DM 轴

// 空载初始超时，实测行程确定后须复核，包含原有分段减速时间。
static constexpr uint32_t LIFT_FIRST_TIMEOUT_MS = 30000;  // 第一次抬升超时，ms
static constexpr uint32_t EXTEND_TIMEOUT_MS = 60000;      // 前伸超时，ms
static constexpr uint32_t LIFT_SECOND_TIMEOUT_MS = 30000; // 第二次抬升超时，ms
static constexpr uint32_t RETRACT_TIMEOUT_MS = 60000;     // 后收超时，ms
static constexpr uint32_t LOWER_TIMEOUT_MS = 30000;       // 下降超时，ms
static constexpr float TRAVEL_MARGIN = 0.20f;        // 超出行程或反向走 10% 判故障
static constexpr float LIFT_MISMATCH = 0.10f;        // 双升降归一化进度差允许 10%

// 以下为本次空载试转初值，并非实测触底判据；不取消任何堵转、原点或行程保护。
// 下降到位由 PD10 限位判断，电流和低速仅用于检测堵转故障。
static constexpr float LOWER_CURRENT_LIMIT_A = 20.0f; // 空载下降限流；还需实测触底电流
static constexpr float LOWER_STALL_CURRENT_A = 0.3f; // 任一台反馈电流绝对值门槛，>0 且小于下降限流
static constexpr float LOWER_STALL_SPEED_RAD_S = 0.4f / 19.0f; // 两台输出轴低速门槛，约 0.0211 rad/s，待实测
static constexpr uint32_t LOWER_STALL_CONFIRM_MS = 50; // 电流和低速条件连续成立时间，>0 且小于下降超时
static constexpr double LIFT_HOME_TOLERANCE_DEG = 10.0; // 两台输出轴相对上电原点允许偏差 ±10 deg，待实测

static constexpr float DM_MOVE_KD = 1.0f;      // 空载起始速度阻尼，Nm/(rad/s)，待实机整定
static constexpr float DM_HOLD_KD = 1.0f;      // 空载起始保持阻尼，Nm/(rad/s)
static constexpr float DM_TORQUE_LIMIT_NM = 0.15f; // 空载总阻尼力矩限幅，Nm，非带载参数
}
