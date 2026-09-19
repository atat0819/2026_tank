#include "up_stair_behind_motor_fsm.hpp"

#include <math.h>

/*
 * 后部上台阶电机状态机的实现。
 *
 * 本文件只负责检查 IMU、左右电机反馈和机械角度，维护后部状态，执行
 * 300 ms 力矩软启动，并对最终力矩进行机械边界和反馈安全保护。
 * 本文件不执行姿态 PID，也不直接发送电机命令；任务层读取本文件提供的
 * 姿态反馈，完成 pitch/roll 串级 PID 和左右混控后，再调用 Limit_Torque()。
 *
 * 对外电机编号使用 1、2 表示左、右两个逻辑通道，内部数组下标为 0、1。
 * 具体 CAN ID 由后部电机对象负责映射，不能把这里的逻辑编号直接当作
 * 物理 CAN ID 使用。
 */

namespace
{
// 圆周常量；用于角度相关实现的统一单位说明。
const float PI_RAD = 3.14159265358979323846f;
}

/*
 * 构造默认配置。
 * 默认机械起止角相等，因此会被 Validate_Config() 判定为无效，保证完成
 * 实测机械标定之前后部电机保持安全禁用状态。
 */
Class_Up_Stair_Behind_Motor_FSM::Config::Config()
    : pitch_zero_deg(0.0f),
      roll_zero_deg(0.0f),
      angle_start_rad{0.0f, 0.0f},
      angle_end_rad{0.0f, 0.0f},
      motor_direction{1, 1},
      motor_torque_gain{1.0f, 1.0f}
{
}

/*
 * 使用默认配置构造后部状态机。
 * 初始状态为 DISABLED，所有反馈和输出比例清零；随后检查默认配置是否
 * 通过安全校验。
 */
Class_Up_Stair_Behind_Motor_FSM::Class_Up_Stair_Behind_Motor_FSM()
    : config_(),
      config_valid_(false),
      motor_controllable_{false, false},
      motor_angle_rad_{0.0f, 0.0f},
      feedback_pitch_deg_(0.0f),
      feedback_roll_deg_(0.0f),
      pitch_rate_dps_(0.0f),
      roll_rate_dps_(0.0f),
      output_scale_(0.0f),
      recovery_start_tick_(0U),
      recovery_last_tick_(0U),
      recovery_planner_(0.0f, 0.0f),
      feedback_degraded_(false),
      feedback_valid_previous_{false, false}
{
    Class_FSM::Init(UP_STAIR_BEHIND_MOTOR_COUNT,
                    UP_STAIR_BEHIND_MOTOR_DISABLED);
    config_valid_ = Validate_Config();
}

/*
 * 使用指定机械配置构造后部状态机。
 * config 包含 IMU 零偏、左右电机机械安全区间和电机方向系数。配置有效性
 * 在构造时检查并锁存到 config_valid_，运行过程中不会自动重新标定。
 */
Class_Up_Stair_Behind_Motor_FSM::Class_Up_Stair_Behind_Motor_FSM(
    const Config &config)
    : config_(config),
      config_valid_(false),
      motor_controllable_{false, false},
      motor_angle_rad_{0.0f, 0.0f},
      feedback_pitch_deg_(0.0f),
      feedback_roll_deg_(0.0f),
      pitch_rate_dps_(0.0f),
      roll_rate_dps_(0.0f),
      output_scale_(0.0f),
      recovery_start_tick_(0U),
      recovery_last_tick_(0U),
      recovery_planner_(0.0f, 0.0f),
      feedback_degraded_(false),
      feedback_valid_previous_{false, false}
{
    Class_FSM::Init(UP_STAIR_BEHIND_MOTOR_COUNT,
                    UP_STAIR_BEHIND_MOTOR_DISABLED);
    config_valid_ = Validate_Config();
}

/*
 * 将后部状态机恢复到初始禁用状态。
 * 同时清除恢复斜坡时间、规划器输出、电机可控标志和反馈恢复沿检测标志；
 * 不改变 config_ 和 config_valid_。
 */
void Class_Up_Stair_Behind_Motor_FSM::Reset()
{
    // 完整复位后部状态、反馈有效标志和恢复斜坡规划器。
    Class_FSM::Init(UP_STAIR_BEHIND_MOTOR_COUNT,
                    UP_STAIR_BEHIND_MOTOR_DISABLED);
    output_scale_ = 0.0f;
    recovery_start_tick_ = 0U;
    recovery_last_tick_ = 0U;
    recovery_planner_.SetNowReal(0.0f);
    recovery_planner_.SetTarget(0.0f);
    recovery_planner_.TIM_Calculate_PeriodElapsedCallback(0.0f, 0.0f);
    motor_controllable_[0] = false;
    motor_controllable_[1] = false;
    feedback_degraded_ = false;
    feedback_valid_previous_[0] = false;
    feedback_valid_previous_[1] = false;
}

/*
 * 将对外逻辑电机编号转换为内部数组下标。
 * id=1 对应左电机下标 0，id=2 对应右电机下标 1；其他编号返回 2 作为
 * 非法下标哨兵，供调用者通过 index > 1U 统一拒绝。
 */
uint8_t Class_Up_Stair_Behind_Motor_FSM::To_Index(uint8_t id) const
{
    return (id >= 1U && id <= 2U) ? static_cast<uint8_t>(id - 1U) : 2U;
}

/*
 * 检查姿态零偏、机械安全区间和电机方向配置。
 * 起止角可以构成普通区间或跨越 0 弧度的区间，但不能相等；方向系数只能
 * 是 +1 或 -1；左右最终力矩增益必须为正的有限值。任何一项不满足都会使
 * 整个后部状态机禁止输出。
 */
bool Class_Up_Stair_Behind_Motor_FSM::Validate_Config() const
{
    // 检查零位、机械安全区间和电机正负方向；无效配置必须禁止出力。
    for (uint8_t index = 0U; index < 2U; ++index)
    {
        const float start = config_.angle_start_rad[index];
        const float end = config_.angle_end_rad[index];
        if (!std::isfinite(start) || !std::isfinite(end) ||
            start < 0.0f || start > TWO_PI_RAD || end < 0.0f ||
            end > TWO_PI_RAD || start == end ||
            (config_.motor_direction[index] != 1 &&
             config_.motor_direction[index] != -1) ||
            !std::isfinite(config_.motor_torque_gain[index]) ||
            config_.motor_torque_gain[index] <= 0.0f)
        {
            return false;
        }
        if (start == TWO_PI_RAD && end == 0.0f)
        {
            return false;
        }
    }
    return std::isfinite(config_.pitch_zero_deg) &&
           std::isfinite(config_.roll_zero_deg);
}

/*
 * 将跨越 0 弧度的原始角度展开成连续角度。
 * 例如安全区间为 300°～60° 时，原始 30° 会转换为 390°，从而可以和连续
 * 区间 300°～420° 一起进行机械边界判断；非跨零区间保持原值。
 */
float Class_Up_Stair_Behind_Motor_FSM::To_Unwrapped_Angle(
    uint8_t index, float raw_angle_rad) const
{
    if (config_.angle_start_rad[index] > config_.angle_end_rad[index] &&
        raw_angle_rad < config_.angle_start_rad[index])
    {
        return raw_angle_rad + TWO_PI_RAD;
    }
    return raw_angle_rad;
}

/*
 * 判断指定逻辑电机的编码器角度是否可用于控制。
 * 除了检查编号、配置和有限值外，还要求原始角度位于 [0, 2π]，并落在当前
 * 电机配置的普通安全区间或跨零安全区间内，起止边界均包含在内。
 */
bool Class_Up_Stair_Behind_Motor_FSM::Is_Angle_Valid(
    uint8_t id, float raw_angle_rad) const
{
    // 编码器原始角度必须在 [0, 2π] 内，并落在允许的普通区间或跨零区间内。
    const uint8_t index = To_Index(id);
    if (index > 1U || !config_valid_ || !std::isfinite(raw_angle_rad) ||
        raw_angle_rad < 0.0f || raw_angle_rad > TWO_PI_RAD)
    {
        return false;
    }

    if (config_.angle_start_rad[index] < config_.angle_end_rad[index])
    {
        return raw_angle_rad >= config_.angle_start_rad[index] &&
               raw_angle_rad <= config_.angle_end_rad[index];
    }

    return raw_angle_rad >= config_.angle_start_rad[index] ||
           raw_angle_rad <= config_.angle_end_rad[index];
}

/*
 * 关闭后部姿态控制。
 * 进入 DISABLED 后，Limit_Torque() 会返回 0；同时恢复规划器和反馈恢复检测
 * 状态清零，下一次恢复必须重新从 0 开始软启动。
 */
void Class_Up_Stair_Behind_Motor_FSM::Disable()
{
    // 任何控制链路、IMU 或配置故障都回到禁用态，并清零恢复规划器。
    Set_Status(UP_STAIR_BEHIND_MOTOR_DISABLED);
    output_scale_ = 0.0f;
    recovery_start_tick_ = 0U;
    recovery_last_tick_ = 0U;
    recovery_planner_.SetNowReal(0.0f);
    recovery_planner_.SetTarget(0.0f);
    recovery_planner_.TIM_Calculate_PeriodElapsedCallback(0.0f, 0.0f);
    motor_controllable_[0] = false;
    motor_controllable_[1] = false;
    feedback_degraded_ = false;
    feedback_valid_previous_[0] = false;
    feedback_valid_previous_[1] = false;
}

/*
 * 启动一次后部控制恢复过程。
 * 用于后部首次上线或任一反馈从无效恢复的场景。状态切换到 RECOVERING，
 * 输出比例从 0 开始，避免姿态力矩瞬间恢复。
 */
void Class_Up_Stair_Behind_Motor_FSM::Start_Recovery(uint32_t now_tick)
{
    // 后部重新上线时从零比例开始，避免姿态力矩突然恢复。
    Set_Status(UP_STAIR_BEHIND_MOTOR_RECOVERING);
    recovery_start_tick_ = now_tick;
    recovery_last_tick_ = now_tick;
    output_scale_ = 0.0f;
    recovery_planner_.SetNowReal(0.0f);
    recovery_planner_.SetTarget(0.0f);
    recovery_planner_.TIM_Calculate_PeriodElapsedCallback(0.0f, 0.0f);
}

/*
 * 更新恢复斜坡输出比例。
 * 使用当前 tick 与上次 tick 的差值计算本周期增量，而不是假设 Update() 固定
 * 周期调用。输出比例在 300 ms 内由 0 增加到 1，达到目标后切换到
 * ATTITUDE_HOLD。
 */
void Class_Up_Stair_Behind_Motor_FSM::Update_Recovery_Scale(uint32_t now_tick)
{
    const uint32_t elapsed = now_tick - recovery_start_tick_;
    const uint32_t delta_tick = now_tick - recovery_last_tick_;
    recovery_last_tick_ = now_tick;

    // 根据真实经过时间设置本周期增量，SlopePlanning 输出约 300 ms 内从 0 到 1。
    float step = static_cast<float>(delta_tick) /
                 static_cast<float>(RECOVERY_TIME_MS);
    if (step > 1.0f)
    {
        step = 1.0f;
    }
    recovery_planner_.SetIncreaseValue(step);
    recovery_planner_.SetDecreaseValue(step);
    recovery_planner_.TIM_Calculate_PeriodElapsedCallback(1.0f, 0.0f);
    output_scale_ = recovery_planner_.GetOut();

    if (elapsed >= RECOVERY_TIME_MS || output_scale_ >= 1.0f)
    {
        Set_Status(UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
        output_scale_ = 1.0f;
    }
}

/*
 * 更新后部状态机。
 *
 * 参数说明：
 * - control_enabled：上层档位和控制策略是否允许后部姿态控制；
 * - imu_valid：本次 IMU 快照是否有效；
 * - left/right_feedback_valid：左右电机反馈链路是否在线；
 * - pitch_deg、roll_deg：IMU 姿态角，单位为度；
 * - pitch_rate_dps、roll_rate_dps：姿态角速度，单位为度/秒；
 * - left/right_angle_rad：左右编码器原始角度，单位为弧度；
 * - now_tick：当前系统 tick，用于计算恢复斜坡时间。
 *
 * 函数先检查 IMU 数值并保存校准后的反馈，再分别判断左右电机是否可控，
 * 最后根据控制权限、反馈恢复沿和当前状态执行禁用、软启动或正常保持。
 * 这里不执行 PID，也不发送电机命令。
 */
void Class_Up_Stair_Behind_Motor_FSM::Update(
    bool control_enabled,
    bool imu_valid,
    bool left_feedback_valid,
    bool right_feedback_valid,
    float pitch_deg,
    float roll_deg,
    float pitch_rate_dps,
    float roll_rate_dps,
    float left_angle_rad,
    float right_angle_rad,
    uint32_t now_tick)
{
    if (!std::isfinite(pitch_deg) || !std::isfinite(roll_deg) ||
        !std::isfinite(pitch_rate_dps) || !std::isfinite(roll_rate_dps))
    {
        // IMU 任一通道出现非有限值，整帧姿态数据作废并立即禁用后部输出。
        feedback_pitch_deg_ = 0.0f;
        feedback_roll_deg_ = 0.0f;
        pitch_rate_dps_ = 0.0f;
        roll_rate_dps_ = 0.0f;
        motor_angle_rad_[0] = 0.0f;
        motor_angle_rad_[1] = 0.0f;
        Disable();
        return;
    }

    const float calibrated_pitch_deg = pitch_deg - config_.pitch_zero_deg;
    const float calibrated_roll_deg = roll_deg - config_.roll_zero_deg;
    if (!std::isfinite(calibrated_pitch_deg) ||
        !std::isfinite(calibrated_roll_deg))
    {
        // 零偏相减发生浮点溢出时同样不能继续使用该姿态数据。
        feedback_pitch_deg_ = 0.0f;
        feedback_roll_deg_ = 0.0f;
        pitch_rate_dps_ = 0.0f;
        roll_rate_dps_ = 0.0f;
        motor_angle_rad_[0] = 0.0f;
        motor_angle_rad_[1] = 0.0f;
        Disable();
        return;
    }

    feedback_pitch_deg_ = calibrated_pitch_deg;
    feedback_roll_deg_ = calibrated_roll_deg;
    pitch_rate_dps_ = pitch_rate_dps;
    roll_rate_dps_ = roll_rate_dps;
    motor_angle_rad_[0] = left_angle_rad;
    motor_angle_rad_[1] = right_angle_rad;
    motor_controllable_[0] = config_valid_ && left_feedback_valid &&
                              Is_Angle_Valid(1U, left_angle_rad);
    motor_controllable_[1] = config_valid_ && right_feedback_valid &&
                              Is_Angle_Valid(2U, right_angle_rad);
    // 每个电机独立判断反馈和机械角度；无效侧由 Limit_Torque 强制为零。
    const bool feedback_recovered =
        (!feedback_valid_previous_[0] && motor_controllable_[0]) ||
        (!feedback_valid_previous_[1] && motor_controllable_[1]);
    feedback_valid_previous_[0] = motor_controllable_[0];
    feedback_valid_previous_[1] = motor_controllable_[1];

    if (!control_enabled || !imu_valid || !config_valid_)
    {
        // 档位、IMU 或标定配置任意一项不满足，后部不允许姿态控制。
        Disable();
        return;
    }

    const bool both_feedback_valid = motor_controllable_[0] &&
                                     motor_controllable_[1];
    if (Get_State() != UP_STAIR_BEHIND_MOTOR_DISABLED &&
        !both_feedback_valid)
    {
        feedback_degraded_ = true;
    }

    if (Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED)
    {
        Start_Recovery(now_tick);
        return;
    }

    if (feedback_recovered)
    {
        // 任意一侧从无效恢复，都重新启动全局软启动，防止单侧突加力矩。
        Start_Recovery(now_tick);
        feedback_degraded_ = !both_feedback_valid;
        return;
    }

    if (Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING)
    {
        Update_Recovery_Scale(now_tick);
    }
    else
    {
        output_scale_ = 1.0f;
    }
}

/* 返回当前状态机状态：DISABLED、RECOVERING 或 ATTITUDE_HOLD。 */
uint8_t Class_Up_Stair_Behind_Motor_FSM::Get_State() const
{
    return Now_Status_Serial;
}

/* 返回当前恢复斜坡比例，正常范围为 0.0～1.0。 */
float Class_Up_Stair_Behind_Motor_FSM::Get_Output_Scale() const
{
    return output_scale_;
}

/* 后部姿态控制的目标 pitch；当前设计目标为水平姿态 0°。 */
float Class_Up_Stair_Behind_Motor_FSM::Get_Target_Pitch_Deg() const
{
    return 0.0f;
}

/* 后部姿态控制的目标 roll；当前设计目标为水平姿态 0°。 */
float Class_Up_Stair_Behind_Motor_FSM::Get_Target_Roll_Deg() const
{
    return 0.0f;
}

/* 返回去除 IMU pitch 零偏后的角度反馈，单位为度。 */
float Class_Up_Stair_Behind_Motor_FSM::Get_Feedback_Pitch_Deg() const
{
    return feedback_pitch_deg_;
}

/* 返回去除 IMU roll 零偏后的角度反馈，单位为度。 */
float Class_Up_Stair_Behind_Motor_FSM::Get_Feedback_Roll_Deg() const
{
    return feedback_roll_deg_;
}

/* 返回最近一次保存的 pitch 角速度反馈，单位为度/秒。 */
float Class_Up_Stair_Behind_Motor_FSM::Get_Pitch_Rate_Dps() const
{
    return pitch_rate_dps_;
}

/* 返回最近一次保存的 roll 角速度反馈，单位为度/秒。 */
float Class_Up_Stair_Behind_Motor_FSM::Get_Roll_Rate_Dps() const
{
    return roll_rate_dps_;
}

/*
 * 返回指定逻辑电机的方向系数。
 * 任务层先完成 pitch/roll 左右混控，再使用该系数修正左右电机的实际正
 * 方向；非法编号返回 0。
 */
int8_t Class_Up_Stair_Behind_Motor_FSM::Get_Motor_Direction(uint8_t id) const
{
    const uint8_t index = To_Index(id);
    return index <= 1U ? config_.motor_direction[index] : 0;
}

/*
 * 判断指定逻辑电机当前是否允许输出姿态力矩。
 * 禁用状态下即使编码器标志有效，也不允许输出；只有状态机已进入恢复或
 * 姿态保持状态，并且该侧反馈和机械角度均有效时才返回 true。
 */
bool Class_Up_Stair_Behind_Motor_FSM::Is_Motor_Controllable(uint8_t id) const
{
    const uint8_t index = To_Index(id);
    return index <= 1U && Get_State() != UP_STAIR_BEHIND_MOTOR_DISABLED &&
           motor_controllable_[index];
}

/*
 * 对任务层生成的原始力矩执行最终安全限制。
 *
 * raw_torque_nm 已经包含 pitch/roll 混控和方向修正。本函数继续检查电机
 * 编号、反馈状态、状态机状态和有限值，然后：
 * 1. 在机械下限附近禁止继续向负方向施力；
 * 2. 在机械上限附近禁止继续向正方向施力；
 * 3. 乘以恢复斜坡比例 output_scale_ 和该侧最终力矩校准增益。
 *
 * 远离机械边界时，允许“逃离限位”的反方向力矩通过。
 */
float Class_Up_Stair_Behind_Motor_FSM::Limit_Torque(
    uint8_t id, float raw_torque_nm) const
{
    const uint8_t index = To_Index(id);
    if (index > 1U || !Is_Motor_Controllable(id) ||
        (Get_State() != UP_STAIR_BEHIND_MOTOR_RECOVERING &&
         Get_State() != UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD) ||
        !std::isfinite(raw_torque_nm))
    {
        return 0.0f;
    }

    const float angle = To_Unwrapped_Angle(index, motor_angle_rad_[index]);
    const float lower = config_.angle_start_rad[index];
    const float upper = config_.angle_start_rad[index] <
                                config_.angle_end_rad[index]
                            ? config_.angle_end_rad[index]
                            : config_.angle_end_rad[index] + TWO_PI_RAD;
    const bool at_lower_margin = angle <= lower + LIMIT_MARGIN_RAD;
    const bool at_upper_margin = angle >= upper - LIMIT_MARGIN_RAD;
    if ((at_lower_margin && raw_torque_nm < 0.0f) ||
        (at_upper_margin && raw_torque_nm > 0.0f))
    {
        return 0.0f;
    }
    // 确认机械方向安全后，再对左、右终端力矩分别做校准。
    return raw_torque_nm * output_scale_ * config_.motor_torque_gain[index];
}

/* 返回构造时完成的机械配置安全校验结果。 */
bool Class_Up_Stair_Behind_Motor_FSM::Is_Config_Valid() const
{
    return config_valid_;
}
