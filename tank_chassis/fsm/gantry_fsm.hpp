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
    // 限位依次为 PE13、PE9、PA2、PA0、PD10；角度为 ID5、ID6、DM ID9 的累计角度 deg。
    // 两台 3508 的反馈电流为 A，速度为输出轴端 rad/s。
    void Update(bool control_enabled, bool feedback_ok, bool lift_feedback_ok,
                bool start_key_raw, bool start_key_pressed,
                bool lift_first_limit, bool extend_limit, bool lift_second_limit,
                bool retract_limit, bool lower_limit,
                float lift_id5_angle_deg, float lift_id6_angle_deg, float dm_angle_deg,
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
    float GetHoldAngle(unsigned axis) const { return active_ ? hold_deg_[axis] : 0; } // deg
    float GetProgress() const { return progress_; }
    float GetLiftSyncErrorDeg() const { return active_ ? lift_sync_error_deg_ : 0; }
    float GetGravityLoadFactor() const { return active_ ? gravity_load_factor_ : 0; }
    float GetScale() const { return active_ && Running() ? scale_ : 1; }

private:
    bool Running() const;
    void Enter(GantryState state, uint32_t now_ms,
               const float (&angle_deg)[3], const bool (&limit_pressed)[5]);
    void Fail(GantryFault fault);
    GantryState state_ = GantryState::Disabled;
    GantryFault fault_ = GantryFault::None;
    bool start_armed_ = false;
    bool start_hold_pending_ = false;
    uint32_t start_hold_since_ms_ = 0;
    bool home_recorded_ = false; // 通电前手动压到底并齐平；首次有效低速反馈记录一次，不随切模式重置
    bool gravity_unloading_ = false;
    uint32_t gravity_unload_since_ms_ = 0;
    float gravity_load_factor_ = 0;
    float home_deg_[2] = {};
    bool holding_ = false;
    uint32_t entered_ms_ = 0;
    float start_deg_[3] = {};
    float hold_deg_[3] = {};
    float scale_ = 1;
    bool active_ = false;
    bool moving_[3] = {};
    float target_speed_rad_s_[3] = {};
    float progress_ = 0;
    float lift_sync_error_deg_ = 0; // 向上为正的 ID5 位移减 ID6 位移，输出轴 deg
};

// 龙门架运动标定和 DM 控制参数集中于此；角度外环和 3508 速度 PID 在 gantry_task.cpp 中填写。
// 行程和目标速度暂填 0；填写实际值并通过校验后才允许动作。
// 实测时读取 gantry_watch.angle_deg，再填写本文件并重新编译。
// 龙门架使用 float，与电机角度读取接口一致；小数常量保留 f 后缀，避免运算被提升为 double。
namespace GantryTuning
{
static constexpr uint32_t START_HOLD_MS = 2000; // ms：Z 键需连续按住的时间；越大越不易误触，松开立即取消计时
static constexpr bool CALIBRATED = true; // 总标定开关：false 禁止动作；true 仍须行程/速度等校验通过，不代表 PID 已整定

// 角度均为 gantry_watch.angle_deg 的累计 deg；3508 实例减速比为 19.2，Watch 已换算为输出轴，不要再除减速比。
// 每台各端点在同一次连续反馈中记录，保留正负号；只用端点差计算方向、行程进度和保护，不作为运动位置目标。
// 3508 每次通电读数可不同：运行同步原点另取本次上电的首次有效低速反馈，通电前必须手动压到底并齐平。
// 也可将各 HOME 填 0，将 FIRST/SECOND 填相对底部的有符号总增量；每台两段抬升方向必须一致且行程非零。
static constexpr float LIFT_ID5_HOME_DEG = 0.0f;    // deg：ID5 标定时底部角度，与 FIRST/SECOND 配对计算行程；不是写死的上电齐平原点
static constexpr float LIFT_ID6_HOME_DEG = 0.0f;    // deg：ID6 标定时底部角度；两台 HOME 数值不必相同
static constexpr float LIFT_ID5_FIRST_DEG = 0.0f;   // deg：ID5 第一次抬升开关触发位置；FIRST-HOME 的符号决定 ID5 向上方向
static constexpr float LIFT_ID6_FIRST_DEG = 0.0f;   // deg：ID6 第一次抬升开关触发位置；允许 FIRST-HOME 与 ID5 异号
static constexpr float LIFT_ID5_SECOND_DEG = 0.0f;  // deg：ID5 第二次抬升开关触发位置；SECOND-FIRST 为第二段，SECOND-HOME 为完整下降行程的反向
static constexpr float LIFT_ID6_SECOND_DEG = 0.0f;  // deg：ID6 第二次抬升开关触发位置；填总端点，不是第二段单独的角度增量
static constexpr float DM_RETRACT_DEG = 0.0f;       // deg：4310 后收开关触发时的累计角度；与 EXTEND 在同一反馈坐标下标定
static constexpr float DM_EXTEND_DEG = 0.0f;        // deg：4310 前伸开关触发时的累计角度；EXTEND-RETRACT 决定前伸方向和行程，差值不能为 0

// 表达式最前面的数填写正的输出轴 deg/s，换算结果为 rad/s；任一速度为 0 时禁止启动。
// 升降两侧共用本段速度幅值，各自的转向由标定行程决定。
static constexpr float LIFT_FIRST_SPEED_RAD_S = 0.0f / 180.0f * 3.1415926f;  // 第一次抬升
static constexpr float DM_EXTEND_SPEED_RAD_S = 0.0f / 180.0f * 3.1415926f;   // 前伸，换算后不超过 30 rad/s
static constexpr float LIFT_SECOND_SPEED_RAD_S = 0.0f / 180.0f * 3.1415926f; // 第二次抬升
static constexpr float DM_RETRACT_SPEED_RAD_S = 0.0f / 180.0f * 3.1415926f;  // 后收，换算后不超过 30 rad/s
static constexpr float LOWER_SPEED_RAD_S = 0.0f / 180.0f * 3.1415926f;       // 下降

// 每段从进入状态开始计时；应覆盖正常行程、减速、同步等待和开关消抖，过大将延迟故障发现。
// 单位 ms；有效范围 1..0x7FFFFFFF，目标开关未触发且达到超时则锁存 Timeout。
static constexpr uint32_t LIFT_FIRST_TIMEOUT_MS = 30000;  // 第一次抬升等 PE13 的最长时间，当前 30 s
static constexpr uint32_t EXTEND_TIMEOUT_MS = 60000;      // 4310 前伸等 PE9 的最长时间，当前 60 s
static constexpr uint32_t LIFT_SECOND_TIMEOUT_MS = 30000; // 第二次抬升等 PA2 的最长时间，当前 30 s
static constexpr uint32_t RETRACT_TIMEOUT_MS = 60000;     // 4310 后收等 PA0 的最长时间，当前 60 s
static constexpr uint32_t LOWER_TIMEOUT_MS = 30000;       // 下降等 PD10 的最长时间，当前 30 s，包含卸载后的剩余下降
static constexpr float TRAVEL_MARGIN = 0.20f;        // 无量纲 (0,1)：本段进度小于 -20% 或大于 120% 判 Travel；越大允许的越界越多
static constexpr float LIFT_MISMATCH = 0.10f;        // 无量纲 (0,1)：两台本段归一化进度差超过 10 个百分点判 LiftMismatch；不是同步增益或实测机械容差

// 两侧链轮/外部传动相同；方向由各自 FIRST_DEG - HOME_DEG 决定。
static constexpr float LIFT_SYNC_KP = 1.0f; // 1/s，>=0：同步角差 rad 乘此值得到纠偏 rad/s；越大纠偏越强，过大可能振荡；0 禁用，初始值待实测
static constexpr float LIFT_SYNC_MAX_SPEED_RATIO = 0.20f; // 无量纲 [0,1]：纠偏上限=当前公共速度×此值；0 禁用，0.2 最多让等待侧降速 20%，1 允许降至停住

// 上电记录原点和启动时，两台升降输出轴的静止速度门槛。
static constexpr float HOME_STOP_SPEED_RAD_S = 0.4f / 19.0f; // 输出轴 rad/s，>0：两侧速度绝对值均不超过此值才可记原点/启动；现值约 0.0211，越大越易将运动判为静止
static constexpr float LIFT_HOME_TOLERANCE_DEG = 20.0f; // 输出轴 deg，>0：启动时各自相对本次上电原点允许 ±20 deg；越大越容易放行未到底的位置

// C620 有符号原始电流指令 [-16384,16384]，不是 A；标称 16384 对应 20 A，实际承载能力需测量。
// 按两台各自向上出力的电流方向填写，允许一正一负；下降时仍向上抗重力，不因运动方向翻转补偿。
// 补偿加在速度 PID 输出后，总指令再次限幅 ±16384；补偿越大，剩余向上加速的电流余量越小。
static constexpr float LIFT_ID5_GRAVITY_EMPTY_CMD = 0.0f; // ID5 空载支撑指令：第一次抬升/前伸保持，以及卸载后的下降；0 表示无补偿
static constexpr float LIFT_ID6_GRAVITY_EMPTY_CMD = 0.0f; // ID6 空载支撑指令：单独整定其方向和大小，不要求与 ID5 同号或同幅值
static constexpr float LIFT_ID5_GRAVITY_LOADED_CMD = 0.0f; // ID5 带载支撑指令：第二次抬升一开始、后收保持、下降卸载前使用
static constexpr float LIFT_ID6_GRAVITY_LOADED_CMD = 0.0f; // ID6 带载支撑指令：按该侧实际承重整定；过大可能上窜，过小由速度环补足
static constexpr float LOWER_UNLOAD_PROGRESS = 0.40f; // 无量纲 (0,1)：两侧都走完各自下降行程的 40% 才开始卸载；减小会更早切换，不检测真实接触
static constexpr uint32_t LOWER_UNLOAD_BLEND_MS = 200; // ms，>0：带载补偿线性切至空载补偿的时间；越大越平缓，但载荷已卸下时额外支撑持续更久

// 以下 KD 是 4310 MIT 速度阻尼，不是上位机角度 PID 的微分 Kd；MIT 位置 Kp 固定为 0。
static constexpr float DM_MOVE_KD = 1.0f;      // Nm/(rad/s)，(0,5]：前伸/后收速度误差乘此值产生力矩；越大响应越强，也更易放大速度噪声
static constexpr float DM_HOLD_KD = 1.0f;      // Nm/(rad/s)，(0,5]：4310 位置保持时的速度阻尼，配合 hold_angle_pid[2]；独立于运动 KD 整定
static constexpr float DM_TORQUE_LIMIT_NM = 7.0f; // Nm，(0,3]：前伸/后收/保持共用的力矩绝对值上限；当前 0.15 为空载起始值，过小会跟不上目标
}
