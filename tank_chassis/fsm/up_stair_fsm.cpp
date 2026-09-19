#include "up_stair_fsm.hpp"
#include <math.h>

/*
 * 前部上台阶状态机的实现。
 *
 * 本文件只负责：
 * 1. 保存并检查两个前部电机的编码器反馈；
 * 2. 根据当前状态产生初始位置或上台阶目标位置；
 * 3. 判断两个电机是否已经到达目标；
 * 4. 在 DISABLED、回初始位、去目标位和保持状态之间切换。
 *
 * 本文件不直接发送电机命令，也不执行 PID。任务层会读取本文件
 * 提供的目标角度和反馈角度，再生成实际的电机控制命令。
 */

constexpr float Class_Up_Stair_FSM::LIMIT_START_RAD[2];
constexpr float Class_Up_Stair_FSM::LIMIT_END_RAD[2];
constexpr float Class_Up_Stair_FSM::HOME_ANGLE_RAD[2];
constexpr float Class_Up_Stair_FSM::TARGET_ANGLE_RAD[2];

namespace
{
// 一个完整圆周的弧度值，用于展开跨越 0 弧度的电机角度。
constexpr float TWO_PI_RAD = 2.0f * 3.14159265359f;

// 到位判定容差为 ±2°。
// 该值只用于判断“是否已经到位”，不会改变 target_angle_ 的目标值。
constexpr float POSITION_TOLERANCE_RAD = 2.0f * 3.14159265359f / 180.0f;
}

/*
 * 初始化前部上台阶状态机。
 *
 * 参数 action_sequence：
 *   当前已经产生的 B 动作序号。初始化时保存该值，避免把初始化之前
 *   的旧动作误认为是一次新的 B 动作。
 *
 * 初始化后的状态为 UP_STAIR_DISABLED。后续第一次 Update() 时，只有在
 * 机构被允许控制且至少有一个有效反馈的情况下，状态机才会尝试回到
 * 机械初始位置。
 */
void Class_Up_Stair_FSM::Init(uint32_t action_sequence)
{
    // 初始化父类中的状态数组和当前状态编号。
    Class_FSM::Init(UP_STAIR_STATUS_COUNT, UP_STAIR_DISABLED);

    // 记录当前动作序号，防止重启后重复执行旧的 B 动作。
    last_action_sequence_ = action_sequence;

    // 检查机械限位、初始角度和目标角度配置是否相互匹配。
    config_valid_ = Validate_Configuration();

    // 清空反馈缓存，并把控制目标恢复为机械初始角度。
    Reset_Feedback();
}

/*
 * 清空两个电机的反馈缓存。
 *
 * raw_angle_ 和 control_angle_ 都恢复为 0。目标角度不清零，而是恢复为
 * 两个电机各自的 HOME_ANGLE_RAD；这样即使状态机暂时禁用，外部重新读取
 * 目标时也能得到安全的初始位置目标。
 */
void Class_Up_Stair_FSM::Reset_Feedback()
{
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        // 原始编码器角度：下标 0 为电机 1，下标 1 为电机 2。
        raw_angle_[i] = 0.0f;

        // 连续控制角度；初始化时尚未收到有效反馈，因此清零。
        control_angle_[i] = 0.0f;

        // 初始目标需要经过 To_Control_Angle()，以兼容跨 0 弧度的电机。
        target_angle_[i] = To_Control_Angle(i, HOME_ANGLE_RAD[i]);
    }
}

/*
 * 判断指定电机的原始角度是否在机械安全范围内。
 *
 * id 使用 1、2 表示电机 1、2；内部数组下标使用 0、1，因此先进行
 * id - 1 的转换。
 *
 * 当 LIMIT_START_RAD 小于 LIMIT_END_RAD 时，安全区间是普通区间：
 *     start <= angle <= end
 *
 * 当 LIMIT_START_RAD 大于 LIMIT_END_RAD 时，说明安全区间跨过 0 弧度，
 * 例如电机 2 的安全区间是：
 *     [300°, 360°] 或 [0°, 70°]
 * 此时判断条件变为：
 *     angle >= start 或 angle <= end
 *
 * 如果 id 无效，或者起止限位相同，则返回 false。
 */
bool Class_Up_Stair_FSM::Is_Angle_Valid(uint8_t id, float angle) const
{
    // 对外接口的电机 ID 从 1 开始，只接受 1 和 2。
    if (id < 1U || id > 2U)
    {
        return false;
    }

    // 转换成内部数组下标。
    const uint8_t index = id - 1U;

    // 普通的不跨 0 弧度区间。
    if (LIMIT_START_RAD[index] < LIMIT_END_RAD[index])
    {
        return angle >= LIMIT_START_RAD[index] &&
               angle <= LIMIT_END_RAD[index];
    }

    // 起点大于终点，表示有效区间跨越 0 弧度。
    if (LIMIT_START_RAD[index] > LIMIT_END_RAD[index])
    {
        return angle >= LIMIT_START_RAD[index] ||
               angle <= LIMIT_END_RAD[index];
    }

    // 起点等于终点时无法表示一个有效的机械角度区间。
    return false;
}

/*
 * 将原始角度转换为用于 PID 的连续角度。
 *
 * 对不跨越 0 弧度的电机，原始角度直接返回。
 * 对跨越 0 弧度的电机，如果原始角度落在区间的低段，则加 360°：
 *
 *     原始 340° -> 控制角度 340°
 *     原始  60° -> 控制角度 420°
 *
 * 这样从 340°运动到 60°时，控制器看到的是从 340°连续运动到 420°，
 * 而不是突然从 340°跳到 60°。
 *
 * 注意：index 是内部数组下标，不是对外的电机 ID。
 */
float Class_Up_Stair_FSM::To_Control_Angle(uint8_t index, float raw_angle) const
{
    // 只有安全区间跨越 0 弧度，且原始角度处于低角度段时才展开。
    if (LIMIT_START_RAD[index] > LIMIT_END_RAD[index] &&
        raw_angle < LIMIT_START_RAD[index])
    {
        return raw_angle + TWO_PI_RAD;
    }

    // 普通区间或高角度段不需要转换。
    return raw_angle;
}

/*
 * 检查编译期机械配置是否有效。
 *
 * 对两个电机分别检查：
 * 1. 起始限位和结束限位不能相同；
 * 2. 机械初始角必须位于安全范围内；
 * 3. 上台阶目标角必须位于安全范围内。
 *
 * 该函数只检查配置，不检查当前编码器反馈。
 */
bool Class_Up_Stair_FSM::Validate_Configuration() const
{
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        // i + 1 是对外使用的电机 ID，i 是内部数组下标。
        if (LIMIT_START_RAD[i] == LIMIT_END_RAD[i] ||
            !Is_Angle_Valid(i + 1U, HOME_ANGLE_RAD[i]) ||
            !Is_Angle_Valid(i + 1U, TARGET_ANGLE_RAD[i]))
        {
            // 任意一个电机的配置不合法，整个前部状态机都视为无效。
            return false;
        }
    }

    return true;
}

/*
 * 根据当前状态刷新两个电机的目标角度。
 *
 * 只有 MOVING_TO_TARGET 和 TARGET_HOLD 状态使用 TARGET_ANGLE_RAD；
 * 其他状态都使用 HOME_ANGLE_RAD，包括：
 *
 *     DISABLED
 *     HOME_HOLD
 *     RETURNING_HOME
 *
 * 目标角度是固定值。POSITION_TOLERANCE_RAD 只参与到位判断，不会把目标
 * 角度改成一个 ±2° 的范围。
 */
void Class_Up_Stair_FSM::Refresh_Target()
{
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        // 目标位置由状态决定：去台阶/保持台阶时去目标位，否则回初始位。
        const float raw_target = (Get_Now_Status_Serial() == UP_STAIR_MOVING_TO_TARGET ||
                                  Get_Now_Status_Serial() == UP_STAIR_TARGET_HOLD)
                                      ? TARGET_ANGLE_RAD[i]
                                      : HOME_ANGLE_RAD[i];

        // 目标角也要转换成连续控制角度，特别是电机 2 的 60° -> 420°。
        target_angle_[i] = To_Control_Angle(i, raw_target);
    }
}

/*
 * 判断两个电机是否都到达机械初始位置。
 *
 * 到位条件包括：
 * 1. 左右两个反馈本次都有效；
 * 2. 两个控制角度与各自初始角度的差值都不超过 ±2°。
 *
 * 容差只用于状态转换判断，不会改变目标角度，也不会直接改变 PID 的
 * 输入目标。
 */
bool Class_Up_Stair_FSM::Both_Motors_At_Home(bool left_valid,
                                             bool right_valid) const
{
    // 任意一个反馈无效，都不能宣称两个电机已经同时到达初始位置。
    return left_valid && right_valid &&
           fabsf(control_angle_[0] - To_Control_Angle(0U, HOME_ANGLE_RAD[0])) <=
               POSITION_TOLERANCE_RAD &&
           fabsf(control_angle_[1] - To_Control_Angle(1U, HOME_ANGLE_RAD[1])) <=
               POSITION_TOLERANCE_RAD;
}

/*
 * 判断两个电机是否都到达上台阶目标位置。
 *
 * MOVING_TO_TARGET 状态下，Update() 会调用本函数检查最新反馈；如果本次
 * 检查通过，就把状态切换到 TARGET_HOLD。这里的“当前状态是移动状态”
 * 表示当前控制阶段，而不是要求电机瞬时速度一定非零。
 */
bool Class_Up_Stair_FSM::Both_Motors_At_Target(bool left_valid,
                                               bool right_valid) const
{
    // 必须两个反馈都有效，且两个电机都进入目标角 ±2° 范围。
    return left_valid && right_valid &&
           fabsf(control_angle_[0] - To_Control_Angle(0U, TARGET_ANGLE_RAD[0])) <=
               POSITION_TOLERANCE_RAD &&
           fabsf(control_angle_[1] - To_Control_Angle(1U, TARGET_ANGLE_RAD[1])) <=
               POSITION_TOLERANCE_RAD;
}

/*
 * 推进一次前部上台阶状态机。
 *
 * 参数说明：
 *   current_left_angle/right_angle：两个电机当前原始编码器角度，单位弧度。
 *   left_feedback_valid/right_feedback_valid：对应反馈链路是否有效。
 *   mechanism_enabled：上层是否允许前部位置控制。
 *   stair_command_enabled：当前是否允许执行上台阶动作；为 false 时回初始位。
 *   action_sequence：经过消抖的 B 动作序号，每变化一次表示一次新动作。
 *
 * 本函数每次调用的大致顺序：
 * 1. 检查反馈链路和机械角度是否有效；
 * 2. 在安全条件不满足时进入 DISABLED；
 * 3. 更新有效的角度缓存；
 * 4. 从 DISABLED 重新启用时先回 HOME；
 * 5. 没有上台阶命令时回 HOME；
 * 6. 根据新的 action_sequence 在 HOME 和 TARGET 之间切换；
 * 7. 检查是否到达 HOME 或 TARGET，并切换到对应 HOLD 状态。
 */
void Class_Up_Stair_FSM::Update(float current_left_angle,
                                float current_right_angle,
                                bool left_feedback_valid,
                                bool right_feedback_valid,
                                bool mechanism_enabled,
                                bool stair_command_enabled,
                                uint32_t action_sequence)
{
    // 反馈必须同时满足“链路有效”和“角度位于机械安全范围”。
    const bool left_valid = left_feedback_valid &&
                            Is_Angle_Valid(1U, current_left_angle);
    const bool right_valid = right_feedback_valid &&
                             Is_Angle_Valid(2U, current_right_angle);

    // 机构未授权、静态配置错误，或者左右反馈都失效时，关闭前部控制。
    // 此时同步动作序号，避免恢复后重复处理旧的 B 动作。
    if (!mechanism_enabled || !config_valid_ || (!left_valid && !right_valid))
    {
        Set_Status(UP_STAIR_DISABLED);
        last_action_sequence_ = action_sequence;
        Refresh_Target();
        return;
    }

    // 只更新本次有效的反馈。无效通道保留上一次有效值，但不能参与到位判断。
    if (left_valid)
    {
        raw_angle_[0] = current_left_angle;
        control_angle_[0] = To_Control_Angle(0U, raw_angle_[0]);
    }
    if (right_valid)
    {
        raw_angle_[1] = current_right_angle;
        control_angle_[1] = To_Control_Angle(1U, raw_angle_[1]);
    }

    // 从 DISABLED 重新启用时，先执行回初始位流程，不直接响应旧的上台阶动作。
    if (Get_Now_Status_Serial() == UP_STAIR_DISABLED)
    {
        Set_Status(UP_STAIR_RETURNING_HOME);
        Refresh_Target();
        last_action_sequence_ = action_sequence;

        // 如果重新启用时已经在初始位置，可以直接进入初始位置保持。
        if (Both_Motors_At_Home(left_valid, right_valid))
        {
            Set_Status(UP_STAIR_HOME_HOLD);
        }
        return;
    }

    // 没有上台阶命令时，目标始终是机械初始位置。
    if (!stair_command_enabled)
    {
        // 当前不在初始位置保持，或者反馈表明尚未到位，则进入回初始位状态。
        if (Get_Now_Status_Serial() != UP_STAIR_HOME_HOLD ||
            !Both_Motors_At_Home(left_valid, right_valid))
        {
            Set_Status(UP_STAIR_RETURNING_HOME);
            Refresh_Target();
        }

        // 回到初始位置后进入保持状态。
        if (Both_Motors_At_Home(left_valid, right_valid))
        {
            Set_Status(UP_STAIR_HOME_HOLD);
        }
        last_action_sequence_ = action_sequence;
        return;
    }

    // 每次经过消抖的 B 按键都会递增动作序号，状态机据此在初始位置
    // 和上台阶目标位置之间切换；任务只传递动作序号，不直接改状态。
    if (action_sequence != last_action_sequence_)
    {
        // 先保存序号，保证同一个动作只处理一次。
        last_action_sequence_ = action_sequence;

        // 在初始位或正在回初始位时，B 动作表示开始去上台阶目标位。
        if (Get_Now_Status_Serial() == UP_STAIR_HOME_HOLD ||
            Get_Now_Status_Serial() == UP_STAIR_RETURNING_HOME)
        {
            Set_Status(UP_STAIR_MOVING_TO_TARGET);
        }
        else
        {
            // 在去目标位或目标保持状态时，B 动作表示返回初始位。
            Set_Status(UP_STAIR_RETURNING_HOME);
        }

        // 状态改变后，重新设置对应的固定目标角度。
        Refresh_Target();
    }

    // 当前控制阶段是去目标位，并且两个电机都进入目标 ±2°，转入目标保持。
    // 这里的 MOVING_TO_TARGET 是控制阶段，不代表电机瞬时速度必须非零。
    if (Get_Now_Status_Serial() == UP_STAIR_MOVING_TO_TARGET &&
        Both_Motors_At_Target(left_valid, right_valid))
    {
        Set_Status(UP_STAIR_TARGET_HOLD);
    }
    // 当前处于回初始位或初始位保持阶段，并且两个电机都进入初始位置 ±2°，
    // 转入初始位置保持。
    else if ((Get_Now_Status_Serial() == UP_STAIR_RETURNING_HOME ||
              Get_Now_Status_Serial() == UP_STAIR_HOME_HOLD) &&
             Both_Motors_At_Home(left_valid, right_valid))
    {
        Set_Status(UP_STAIR_HOME_HOLD);
    }
}

/*
 * 获取指定电机的目标控制角度。
 *
 * 返回的是已经经过 To_Control_Angle() 展开的目标角度，例如电机 2 的
 * 原始目标 60°会返回 420°，方便位置 PID 进行连续角度控制。
 * 无效 ID 返回 0。
 */
float Class_Up_Stair_FSM::Get_Target_Angle(uint8_t id) const
{
    // 对外电机 ID 从 1 开始。
    if (id < 1U || id > 2U)
    {
        return 0.0f;
    }
    return target_angle_[id - 1U];
}

/*
 * 获取指定电机最近一次有效的原始编码器角度。
 *
 * 该值不做跨 0 弧度展开。例如电机 2 的实际角度为 60°时，这里返回 60°，
 * 而不是控制用的 420°。如果最近一次反馈无效，返回的仍可能是之前缓存的
 * 有效值；调用者应结合反馈有效标志判断该值当前是否可用。
 */
float Class_Up_Stair_FSM::Get_Current_Angle(uint8_t id) const
{
    // 对外电机 ID 从 1 开始。
    if (id < 1U || id > 2U)
    {
        return 0.0f;
    }
    return raw_angle_[id - 1U];
}

/*
 * 获取指定电机用于位置 PID 的连续反馈角度。
 *
 * 与 Get_Current_Angle() 的区别是：这里返回的是经过跨 0 弧度展开后的值。
 * 例如电机 2 原始反馈 60°，此函数返回 420°。
 */
float Class_Up_Stair_FSM::Get_Position_Feedback(uint8_t id) const
{
    // 对外电机 ID 从 1 开始。
    if (id < 1U || id > 2U)
    {
        return 0.0f;
    }
    return control_angle_[id - 1U];
}

/*
 * 获取位置误差：目标角度 - 连续控制反馈角度。
 *
 * 目标角度是固定目标值；±2°容差只用于 Both_Motors_At_Home() 和
 * Both_Motors_At_Target() 的到位判断，不会在这里对误差做死区处理。
 */
float Class_Up_Stair_FSM::Get_Position_Error(uint8_t id) const
{
    // 对外电机 ID 从 1 开始。
    if (id < 1U || id > 2U)
    {
        return 0.0f;
    }
    return target_angle_[id - 1U] - control_angle_[id - 1U];
}

/*
 * 获取当前状态机状态。
 *
 * 返回值对应 Enum_Up_Stair_Status 中的枚举值。
 */
uint8_t Class_Up_Stair_FSM::Get_State() const
{
    return Now_Status_Serial;
}

/*
 * 判断前部位置控制是否启用。
 *
 * 只有 DISABLED 状态返回 false；HOME_HOLD、MOVING_TO_TARGET、
 * TARGET_HOLD 和 RETURNING_HOME 都表示外部可以继续使用位置控制。
 */
bool Class_Up_Stair_FSM::Is_Enabled() const
{
    return Now_Status_Serial != UP_STAIR_DISABLED;
}
