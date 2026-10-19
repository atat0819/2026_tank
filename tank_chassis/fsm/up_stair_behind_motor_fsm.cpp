#include "up_stair_behind_motor_fsm.hpp"
#include "rear_torque_safety.hpp"

#include <math.h>

volatile float rear_estimated_torque_nm[2] = {0.0f, 0.0f};

/*
 * 后部上台阶电机状态机的实现。
 *
 * 本文件只负责检查 IMU、左右电机反馈和机械角度，维护后部状态，执行
 * 2000 ms 力矩软启动、Roll 比例补偿，并对最终力矩进行机械边界和反馈安全保护。
 * 本文件不执行姿态 PID，也不直接发送电机命令；任务层读取本文件提供的
 * 姿态反馈，完成 pitch 串级 PID 和左右电机方向修正后，再调用 Limit_Torque()。
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

constexpr float Class_Up_Stair_Behind_Motor_FSM::PITCH_ZERO_DEG;
constexpr float Class_Up_Stair_Behind_Motor_FSM::PITCH_TARGET_DEG;
constexpr float Class_Up_Stair_Behind_Motor_FSM::ROLL_TARGET_DEG;
constexpr float Class_Up_Stair_Behind_Motor_FSM::ROLL_KP;
constexpr float Class_Up_Stair_Behind_Motor_FSM::ROLL_MAX_TORQUE_NM;
constexpr float Class_Up_Stair_Behind_Motor_FSM::MIT_SPEED_LIMIT_RAD_S;
constexpr float Class_Up_Stair_Behind_Motor_FSM::ANGLE_START_RAD[2];
constexpr float Class_Up_Stair_Behind_Motor_FSM::ANGLE_END_RAD[2];
constexpr float Class_Up_Stair_Behind_Motor_FSM::ANGLE_LIMIT_TOLERANCE_RAD;
constexpr int8_t Class_Up_Stair_Behind_Motor_FSM::MOTOR_DIRECTION[2];
constexpr float Class_Up_Stair_Behind_Motor_FSM::MOTOR_TORQUE_GAIN[2];
constexpr float Class_Up_Stair_Behind_Motor_FSM::RETRACT_MARGIN_RAD;
constexpr float Class_Up_Stair_Behind_Motor_FSM::RETRACT_TARGET_RAD[2];
constexpr float Class_Up_Stair_Behind_Motor_FSM::RETRACT_POSITION_TOLERANCE_RAD;
constexpr uint32_t Class_Up_Stair_Behind_Motor_FSM::RETRACT_SPEED_RAMP_TIME_MS;
constexpr float Class_Up_Stair_Behind_Motor_FSM::RETRACT_FEEDFORWARD_TORQUE_NM[2];

/*
 * 构造默认配置。
 * 默认机械起止角相等，因此会被 Validate_Config() 判定为无效，保证完成
 * 实测机械标定之前后部电机保持安全禁用状态。
 */
Class_Up_Stair_Behind_Motor_FSM::Config::Config()
    : pitch_zero_deg(0.0f),
      pitch_target_deg(0.0f),
      roll_target_deg(0.0f),
      roll_kp(0.0f),
      roll_max_torque_nm(0.0f),
      angle_start_rad{0.0f, 0.0f},
      angle_end_rad{0.0f, 0.0f},
      motor_direction{1, 1},
      motor_torque_gain{1.0f, 1.0f},
      retract_target_rad{0.0f, 0.0f},
      retract_position_tolerance_rad(0.0f),
      retract_feedforward_torque_nm{0.0f, 0.0f}
{
}

Class_Up_Stair_Behind_Motor_FSM::Config
Class_Up_Stair_Behind_Motor_FSM::Config::WithAngleConstants()
{
    Config config;
    config.pitch_zero_deg = PITCH_ZERO_DEG;
    config.pitch_target_deg = PITCH_TARGET_DEG;
    config.roll_target_deg = ROLL_TARGET_DEG;
    config.roll_kp = ROLL_KP;
    config.roll_max_torque_nm = ROLL_MAX_TORQUE_NM;
    for (int i = 0; i < 2; ++i)
    {
        config.angle_start_rad[i] = ANGLE_START_RAD[i];
        config.angle_end_rad[i] = ANGLE_END_RAD[i];
        config.motor_direction[i] = MOTOR_DIRECTION[i];
        config.motor_torque_gain[i] = MOTOR_TORQUE_GAIN[i];
        config.retract_target_rad[i] = RETRACT_TARGET_RAD[i];
        config.retract_feedforward_torque_nm[i] = RETRACT_FEEDFORWARD_TORQUE_NM[i];
    }
    config.retract_position_tolerance_rad = RETRACT_POSITION_TOLERANCE_RAD;
    return config;
}

/*
 * 使用默认配置构造后部状态机。
 * 初始状态为 DISABLED，所有反馈和输出比例清零；随后检查默认配置是否
 * 通过安全校验。
 */
Class_Up_Stair_Behind_Motor_FSM::Class_Up_Stair_Behind_Motor_FSM()
    : retract_config_valid_(false),
      position_feedback_rad_{0.0f, 0.0f},
      retract_target_angle_rad_{0.0f, 0.0f},
      pending_resume_(false),
      config_(),
      config_valid_(false),
      motor_controllable_{false, false},
      motor_angle_rad_{0.0f, 0.0f},
      feedback_pitch_deg_(0.0f),
      pitch_rate_dps_(0.0f),
      feedback_roll_deg_(0.0f),
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
    retract_config_valid_ = Validate_Retract_Config();
}

/*
 * 使用指定机械配置构造后部状态机。
 * config 包含 IMU 零偏、左右电机机械安全区间和电机方向系数。配置有效性
 * 在构造时检查并锁存到 config_valid_，运行过程中不会自动重新标定。
 */
Class_Up_Stair_Behind_Motor_FSM::Class_Up_Stair_Behind_Motor_FSM(
    const Config &config)
    : retract_config_valid_(false),
      position_feedback_rad_{0.0f, 0.0f},
      retract_target_angle_rad_{0.0f, 0.0f},
      pending_resume_(false),
      config_(config),
      config_valid_(false),
      motor_controllable_{false, false},
      motor_angle_rad_{0.0f, 0.0f},
      feedback_pitch_deg_(0.0f),
      pitch_rate_dps_(0.0f),
      feedback_roll_deg_(0.0f),
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
    retract_config_valid_ = Validate_Retract_Config();
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
    pending_resume_ = false;
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
 * 起止角必须位于 [-π, +π]，可以构成普通区间或跨越 ±π 的区间，但不能相等；方向系数只能
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
            start < -PI_RAD || start > PI_RAD || end < -PI_RAD ||
            end > PI_RAD || start == end ||
            (config_.motor_direction[index] != 1 &&
             config_.motor_direction[index] != -1) ||
            !std::isfinite(config_.motor_torque_gain[index]) ||
            config_.motor_torque_gain[index] <= 0.0f)
        {
            return false;
        }
        if (start == PI_RAD && end == -PI_RAD)
        {
            return false;
        }
    }
    return std::isfinite(config_.pitch_zero_deg) && std::isfinite(config_.pitch_target_deg) &&
           std::isfinite(config_.roll_target_deg) &&
           std::isfinite(config_.roll_kp) && config_.roll_kp >= 0.0f &&
           std::isfinite(config_.roll_max_torque_nm) && config_.roll_max_torque_nm >= 0.0f;
}

/*
 * 将跨越 ±π 接缝的原始角度展开成连续角度。
 * 例如安全区间为 150°～-150° 时，原始 -170° 会转换为 190°，从而可以和连续
 * 区间 150°～210° 一起进行机械边界判断；非跨接缝区间保持原值。
 */
float Class_Up_Stair_Behind_Motor_FSM::To_Unwrapped_Retract_Target(
    uint8_t index) const
{
    float target = config_.retract_target_rad[index];
    if (config_.angle_start_rad[index] > config_.angle_end_rad[index] &&
        target < config_.angle_start_rad[index])
    {
        target += TWO_PI_RAD;
    }
    return target;
}

bool Class_Up_Stair_Behind_Motor_FSM::Validate_Retract_Config() const
{
    if (!config_valid_ ||
        !std::isfinite(config_.retract_position_tolerance_rad) ||
        config_.retract_position_tolerance_rad <= 0.0f)
    {
        return false;
    }

    for (uint8_t index = 0U; index < 2U; ++index)
    {
        const float raw_target = config_.retract_target_rad[index];
        if (!std::isfinite(config_.retract_feedforward_torque_nm[index]) ||
            !std::isfinite(raw_target) || raw_target < -PI_RAD ||
            raw_target > PI_RAD)
        {
            return false;
        }

        const float target = To_Unwrapped_Retract_Target(index);
        const float lower = config_.angle_start_rad[index];
        const float upper = config_.angle_start_rad[index] <
                                    config_.angle_end_rad[index]
                                ? config_.angle_end_rad[index]
                                : config_.angle_end_rad[index] + TWO_PI_RAD;
        if (!std::isfinite(target) || target < lower || target > upper)
        {
            return false;
        }
    }
    return true;
}

float Class_Up_Stair_Behind_Motor_FSM::To_Unwrapped_Angle(
    uint8_t index, float raw_angle_rad) const
{
    if (config_.angle_start_rad[index] > config_.angle_end_rad[index] &&
        raw_angle_rad < config_.angle_start_rad[index] - ANGLE_LIMIT_TOLERANCE_RAD)
    {
        return raw_angle_rad + TWO_PI_RAD;
    }
    return raw_angle_rad;
}

/*
 * 判断指定逻辑电机的编码器角度是否可用于控制。
 * 除了检查编号、配置和有限值外，还要求原始角度位于 [-π, +π]，并落在当前
 * 电机配置的普通安全区间或跨 ±π 安全区间内，包含端点外侧 0.5° 标定容差。
 */
bool Class_Up_Stair_Behind_Motor_FSM::Is_Angle_Valid(
    uint8_t id, float raw_angle_rad) const
{
    // 编码器原始角度必须在 [-π, +π] 内，并落在允许的普通区间或跨 ±π 区间内。
    const uint8_t index = To_Index(id);
    if (index > 1U || !config_valid_ || !std::isfinite(raw_angle_rad) ||
        raw_angle_rad < -PI_RAD || raw_angle_rad > PI_RAD)
    {
        return false;
    }

    if (config_.angle_start_rad[index] < config_.angle_end_rad[index])
    {
        return raw_angle_rad >= config_.angle_start_rad[index] - ANGLE_LIMIT_TOLERANCE_RAD &&
               raw_angle_rad <= config_.angle_end_rad[index] + ANGLE_LIMIT_TOLERANCE_RAD;
    }

    return raw_angle_rad >= config_.angle_start_rad[index] - ANGLE_LIMIT_TOLERANCE_RAD ||
           raw_angle_rad <= config_.angle_end_rad[index] + ANGLE_LIMIT_TOLERANCE_RAD;
}

/*
 * 关闭后部姿态控制。
 * 进入 DISABLED 后，Limit_Torque() 会返回 0；同时恢复规划器和反馈恢复检测
 * 状态清零；故障及未完成的恢复会保留，正常模式切换不新增软启动。
 */
void Class_Up_Stair_Behind_Motor_FSM::Disable()
{
    if (Get_State() != UP_STAIR_BEHIND_MOTOR_DISABLED && output_scale_ < 1.0f)
    {
        feedback_degraded_ = true;
    }
    // 任何控制链路、IMU 或配置故障都回到禁用态，并清零恢复规划器。
    Set_Status(UP_STAIR_BEHIND_MOTOR_DISABLED);
    output_scale_ = 0.0f;
    recovery_start_tick_ = 0U;
    recovery_last_tick_ = 0U;
    recovery_planner_.SetNowReal(0.0f);
    recovery_planner_.SetTarget(0.0f);
    recovery_planner_.TIM_Calculate_PeriodElapsedCallback(0.0f, 0.0f);
    pending_resume_ = false;
    motor_controllable_[0] = false;
    motor_controllable_[1] = false;
    feedback_valid_previous_[0] = false;
    feedback_valid_previous_[1] = false;
}

/*
 * 启动一次后部控制恢复过程。
 * 用于 IMU 或电机反馈故障恢复的场景。状态切换到 RECOVERING，
 * 输出比例从 0 开始，避免姿态力矩瞬间恢复。
 */
void Class_Up_Stair_Behind_Motor_FSM::Start_Recovery(uint32_t now_tick)
{
    // 后部重新上线时从零比例开始，避免姿态力矩突然恢复。
    Set_Status(UP_STAIR_BEHIND_MOTOR_RECOVERING);
    recovery_start_tick_ = now_tick;
    recovery_last_tick_ = now_tick;
    output_scale_ = 0.0f;
    pending_resume_ = false;
    recovery_planner_.SetNowReal(0.0f);
    recovery_planner_.SetTarget(0.0f);
    recovery_planner_.TIM_Calculate_PeriodElapsedCallback(0.0f, 0.0f);
}

/*
 * 更新恢复斜坡输出比例。
 * 使用当前 tick 与上次 tick 的差值计算本周期增量，而不是假设 Update() 固定
 * 周期调用。输出比例在 2000 ms 内由 0 增加到 1，达到目标后切换到
 * ATTITUDE_HOLD。
 */
void Class_Up_Stair_Behind_Motor_FSM::Update_Recovery_Scale(uint32_t now_tick)
{
    const uint32_t elapsed = now_tick - recovery_start_tick_;
    const uint32_t delta_tick = now_tick - recovery_last_tick_;
    recovery_last_tick_ = now_tick;

    // 根据真实经过时间设置本周期增量，SlopePlanning 输出约 2000 ms 内从 0 到 1。
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
        if (Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING)
        {
            Set_Status(UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
        }
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
 * - pitch_deg：IMU 俯仰角，单位为度；
 * - pitch_rate_dps：俯仰角速度，单位为度/秒；
 * - left/right_angle_rad：左右编码器原始角度，单位为弧度；
 * - now_tick：当前系统 tick，用于计算恢复斜坡时间。
 *
 * 函数先检查 IMU 数值并保存校准后的反馈，再分别判断左右电机是否可控，
 * 最后根据控制权限、反馈恢复沿和当前状态执行禁用、软启动或正常保持。
 * 这里不执行 PID，也不发送电机命令。
 */
void Class_Up_Stair_Behind_Motor_FSM::Start_Retracting(uint32_t now_tick)
{
    // 正常启用保留完整力矩比例；已有的故障恢复沿用当前比例，速度另行软启动。
    if (Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED)
    {
        output_scale_ = 1.0f;
    }
    Set_Status(UP_STAIR_BEHIND_MOTOR_RETRACTING);
    retract_speed_planner_ = Alg::Utility::SlopePlanning(0.0f, 0.0f);
    retract_speed_last_tick_ = now_tick;
    pending_resume_ = false;
    for (uint8_t index = 0U; index < 2U; ++index)
    {
        retract_target_angle_rad_[index] = To_Unwrapped_Retract_Target(index);
    }
}

bool Class_Up_Stair_Behind_Motor_FSM::Both_Retract_Targets_Reached() const
{
    for (uint8_t index = 0U; index < 2U; ++index)
    {
        const float target = To_Unwrapped_Retract_Target(index);
        if (!std::isfinite(position_feedback_rad_[index]) ||
            fabsf(position_feedback_rad_[index] -
                  target) >
                config_.retract_position_tolerance_rad)
        {
            return false;
        }
    }
    return true;
}

void Class_Up_Stair_Behind_Motor_FSM::Update(
    bool control_enabled,
    bool imu_valid,
    bool left_feedback_valid,
    bool right_feedback_valid,
    float pitch_deg,
    float pitch_rate_dps,
    float left_angle_rad,
    float right_angle_rad,
    uint32_t now_tick,
    bool retract_requested,
    float left_velocity_rad_s,
    float right_velocity_rad_s,
    float roll_deg)
{
    // 本周期未进入有效发送分支时，观察值保持为 0，避免沿用上一周期的力矩。
    rear_estimated_torque_nm[0] = 0.0f;
    rear_estimated_torque_nm[1] = 0.0f;
    motor_velocity_rad_s_[0] = left_velocity_rad_s;
    motor_velocity_rad_s_[1] = right_velocity_rad_s;
    const bool retract_state =
        Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTING ||
        Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD;
    const float calibrated_pitch_deg = pitch_deg - config_.pitch_zero_deg;
    const bool imu_ready =
        imu_valid && std::isfinite(pitch_deg) &&
        std::isfinite(pitch_rate_dps) && std::isfinite(calibrated_pitch_deg) &&
        std::isfinite(roll_deg);

    feedback_pitch_deg_ = imu_ready ? calibrated_pitch_deg : 0.0f;
    pitch_rate_dps_ = imu_ready ? pitch_rate_dps : 0.0f;
    feedback_roll_deg_ = imu_ready ? roll_deg : 0.0f;
    motor_angle_rad_[0] = left_angle_rad;
    motor_angle_rad_[1] = right_angle_rad;
    motor_controllable_[0] = config_valid_ && left_feedback_valid &&
                              std::isfinite(left_velocity_rad_s) &&
                              Is_Angle_Valid(1U, left_angle_rad);
    motor_controllable_[1] = config_valid_ && right_feedback_valid &&
                              std::isfinite(right_velocity_rad_s) &&
                              Is_Angle_Valid(2U, right_angle_rad);
    if (motor_controllable_[0])
    {
        position_feedback_rad_[0] = To_Unwrapped_Angle(0U, left_angle_rad);
    }
    if (motor_controllable_[1])
    {
        position_feedback_rad_[1] = To_Unwrapped_Angle(1U, right_angle_rad);
    }
    // 每个电机独立判断反馈和机械角度；无效侧由 Limit_Torque 强制为零。
    const bool feedback_recovered =
        (!feedback_valid_previous_[0] && motor_controllable_[0]) ||
        (!feedback_valid_previous_[1] && motor_controllable_[1]);
    feedback_valid_previous_[0] = motor_controllable_[0];
    feedback_valid_previous_[1] = motor_controllable_[1];

    const bool both_feedback_valid = motor_controllable_[0] &&
                                     motor_controllable_[1];
    if ((control_enabled || Get_State() != UP_STAIR_BEHIND_MOTOR_DISABLED) &&
        (!imu_ready || !both_feedback_valid))
    {
        feedback_degraded_ = true;
    }

    // IMU 无效优先于调平、收腿和位置保持，两台后腿统一禁用。
    if (!control_enabled || !config_valid_ || !imu_ready)
    {
        Disable();
        return;
    }

    if (retract_state && !both_feedback_valid)
    {
        Disable();
        return;
    }
    // 故障恢复优先于正常动作切换；首次启用的反馈有效沿不触发软启动。
    if (feedback_degraded_ && both_feedback_valid && feedback_recovered)
    {
        Start_Recovery(now_tick);
        feedback_degraded_ = false;
    }

    // 收腿是持续目标，不再依赖独立 V 事件；恢复中也可立即切入。
    if (retract_requested)
    {
        if (!retract_config_valid_ || !both_feedback_valid)
        {
            Disable();
            return;
        }
        if (!retract_state)
        {
            Start_Retracting(now_tick);
        }
        else if (Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTING)
        {
            if (Both_Retract_Targets_Reached())
            {
                Set_Status(UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD);
            }
        }
        // 按真实 tick 差值推进速度比例；反馈传 0，避免规划器的反馈优先逻辑跳过斜坡。
        const float speed_step = fminf(1.0f,
            static_cast<float>(now_tick - retract_speed_last_tick_) / RETRACT_SPEED_RAMP_TIME_MS);
        retract_speed_last_tick_ = now_tick;
        retract_speed_planner_.SetIncreaseValue(speed_step);
        retract_speed_planner_.TIM_Calculate_PeriodElapsedCallback(1.0f, 0.0f);
        if (output_scale_ < 1.0f)
        {
            Update_Recovery_Scale(now_tick);
        }
        return;
    }

    // 正常进入调平直接使用完整输出；切换动作不能跳过已有故障恢复。
    if (retract_state ||
        Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED)
    {
        if (Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED)
        {
            output_scale_ = 1.0f;
        }
        Set_Status(output_scale_ < 1.0f ? UP_STAIR_BEHIND_MOTOR_RECOVERING
                                      : UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);
        if (output_scale_ < 1.0f)
        {
            Update_Recovery_Scale(now_tick);
        }
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

/* IMU 工作偏角与传感器水平零偏分开标定。 */
float Class_Up_Stair_Behind_Motor_FSM::Get_Target_Pitch_Deg() const
{
    return config_.pitch_target_deg;
}

/* 返回去除 IMU pitch 零偏后的角度反馈，单位为度。 */
float Class_Up_Stair_Behind_Motor_FSM::Get_Feedback_Pitch_Deg() const
{
    return feedback_pitch_deg_;
}

float Class_Up_Stair_Behind_Motor_FSM::Get_Target_Pitch_Rad() const
{
    return Get_Target_Pitch_Deg() * (PI_RAD / 180.0f);
}

float Class_Up_Stair_Behind_Motor_FSM::Get_Feedback_Pitch_Rad() const
{
    return Get_Feedback_Pitch_Deg() * (PI_RAD / 180.0f);
}

/* 返回最近一次保存的 pitch 角速度反馈，单位为度/秒。 */
float Class_Up_Stair_Behind_Motor_FSM::Get_Pitch_Rate_Dps() const
{
    return pitch_rate_dps_;
}

bool Class_Up_Stair_Behind_Motor_FSM::Uses_Attitude_Control() const
{
    return Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING ||
           Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD;
}

float Class_Up_Stair_Behind_Motor_FSM::Get_Raw_Roll_Torque(uint8_t id) const
{
    const uint8_t index = To_Index(id);
    if (index > 1U || !Uses_Attitude_Control() ||
        !Is_Motor_Controllable(1U) || !Is_Motor_Controllable(2U))
    {
        return 0.0f;
    }
    const float roll_torque_nm = fmaxf(-config_.roll_max_torque_nm,
        fminf(config_.roll_max_torque_nm,
            config_.roll_kp * (feedback_roll_deg_ - config_.roll_target_deg)));
    // 用户确认左加右减；方向映射只做一次，保护由合并后的总前馈统一执行。
    return static_cast<float>(config_.motor_direction[index]) *
           (index == 0U ? roll_torque_nm : -roll_torque_nm);
}

bool Class_Up_Stair_Behind_Motor_FSM::Uses_Retract_Position_Control() const
{
    return Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTING ||
           Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD;
}

bool Class_Up_Stair_Behind_Motor_FSM::Uses_Position_Control() const
{
    return Uses_Retract_Position_Control();
}

float Class_Up_Stair_Behind_Motor_FSM::Get_Retract_Target_Angle(
    uint8_t id) const
{
    const uint8_t index = To_Index(id);
    return index <= 1U ? retract_target_angle_rad_[index] : 0.0f;
}

float Class_Up_Stair_Behind_Motor_FSM::Get_Position_Target_Angle(
    uint8_t id) const
{
    const uint8_t index = To_Index(id);
    if (index > 1U)
    {
        return 0.0f;
    }
    return retract_target_angle_rad_[index];
}

float Class_Up_Stair_Behind_Motor_FSM::Get_Position_Feedback(uint8_t id) const
{
    const uint8_t index = To_Index(id);
    return index <= 1U ? position_feedback_rad_[index] : 0.0f;
}

/*
 * 返回指定逻辑电机的方向系数。
 * 任务层先计算 pitch 力矩，再使用该系数修正左右电机的实际正负方向；
 * 非法编号返回 0。
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

// 先应用软启动/增益，再拦截端点向外的力矩；不减速、不生成回退力矩。
float Class_Up_Stair_Behind_Motor_FSM::Limit_Torque(
    uint8_t id, float raw_torque_nm) const
{
    const uint8_t index = To_Index(id);
    if (index > 1U || !Is_Motor_Controllable(id) ||
        (Get_State() != UP_STAIR_BEHIND_MOTOR_RECOVERING &&
         Get_State() != UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD &&
         Get_State() != UP_STAIR_BEHIND_MOTOR_RETRACTING &&
         Get_State() != UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD) ||
        !std::isfinite(raw_torque_nm))
    {
        return 0.0f;
    }

    const float torque = raw_torque_nm * output_scale_ * config_.motor_torque_gain[index];
    if (!std::isfinite(torque))
    {
        return 0.0f;
    }
    if (Has_Mechanical_Limit(index))
    {
        const float angle = To_Unwrapped_Angle(index, motor_angle_rad_[index]);
        const float lower = config_.angle_start_rad[index];
        const float upper = config_.angle_start_rad[index] <
                                    config_.angle_end_rad[index]
                                ? config_.angle_end_rad[index]
                                : config_.angle_end_rad[index] + TWO_PI_RAD;
        const bool at_lower_margin = angle <= lower + ANGLE_LIMIT_TOLERANCE_RAD;
        const bool at_upper_margin = angle >= upper - ANGLE_LIMIT_TOLERANCE_RAD;
        if ((at_lower_margin && torque < 0.0f) ||
            (at_upper_margin && torque > 0.0f))
        {
            return 0.0f;
        }
    }
    return torque;
}

float Class_Up_Stair_Behind_Motor_FSM::Get_Retract_Feedforward_Torque(uint8_t id) const
{
    const uint8_t index = To_Index(id);
    if (index > 1U || !Uses_Retract_Position_Control())
    {
        return 0.0f;
    }
    return StairTorqueSafety::ClampJ6248Torque(
        Limit_Torque(id, config_.retract_feedforward_torque_nm[index]));
}

float Class_Up_Stair_Behind_Motor_FSM::Limit_Velocity(
    uint8_t id, float target_velocity_rad_s, float kd, float attitude_feedforward_nm) const
{
    const uint8_t index = To_Index(id);
    if (index > 1U || !Is_Motor_Controllable(id) ||
        !std::isfinite(target_velocity_rad_s) ||
        !std::isfinite(kd) || kd <= 0.0f || kd > 5.0f)
    {
        if (index <= 1U) rear_estimated_torque_nm[index] = 0.0f;
        return 0.0f;
    }
    const float target = fmaxf(-MIT_SPEED_LIMIT_RAD_S,
                              fminf(MIT_SPEED_LIMIT_RAD_S, target_velocity_rad_s)) *
                         (Uses_Retract_Position_Control() ? retract_speed_planner_.GetOut() : 1.0f);
    const float velocity = motor_velocity_rad_s_[index];
    const float raw_feedforward = Uses_Retract_Position_Control()
                                      ? config_.retract_feedforward_torque_nm[index]
                                      : (Uses_Attitude_Control() ? attitude_feedforward_nm : 0.0f);
    const float feedforward = StairTorqueSafety::ClampJ6248Torque(
        Limit_Torque(id, raw_feedforward));
    const float torque = StairTorqueSafety::ClampJ6248Torque(
        Limit_Torque(id, kd * (target - velocity) + raw_feedforward));
    // MIT 会另外叠加 feedforward；只将剩余力矩换算为速度误差。
    const float limited_target = velocity + (torque - feedforward) / kd;
    const float commanded_target = fmaxf(-MIT_SPEED_LIMIT_RAD_S, fminf(MIT_SPEED_LIMIT_RAD_S, limited_target));
    rear_estimated_torque_nm[index] = kd * (commanded_target - velocity) + feedforward;
    return commanded_target;
}

bool Class_Up_Stair_Behind_Motor_FSM::Has_Mechanical_Limit(uint8_t index) const
{
    return config_.angle_start_rad[index] != -PI_RAD ||
           config_.angle_end_rad[index] != PI_RAD;
}

/* 返回构造时完成的机械配置安全校验结果。 */
bool Class_Up_Stair_Behind_Motor_FSM::Is_Config_Valid() const
{
    return config_valid_;
}
