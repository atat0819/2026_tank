#include "gimbal_fsm.hpp"
#include "../user/core/BSP/RemoteControl/DT7.hpp"

namespace
{
// 遥控摇杆死区及键鼠增量阈值；摇杆为归一化输入，鼠标阈值单位为像素/周期。
constexpr float INPUT_DEADBAND = 0.01f;
constexpr float MOUSE_ANGLE_DEADBAND = 2.0f;
constexpr float PI = 3.14159265358979323846f;
constexpr float FULL_CIRCLE_RAD = 2.0f * PI;

// 绝对值仅用于幅值/死区比较，调用方仍保留原始有符号输入决定方向。
float Absolute_Value(float value)
{
    return (value >= 0.0f) ? value : -value;
}

}

using Remote = BSP::REMOTE_CONTROL::RemoteController;

// Struct_Gimbal_FSM_Config 配置语义（角度 rad、角速度 rad/s）：
// angle_step：每次 FSM 更新将归一化摇杆值乘此值，累加为本周期角度增量；不是时间归一的速率。
// speed_scale：速度状态下每单位摇杆对应的目标角速度(rad/s)。
// mouse_angle_scale：每个有符号鼠标位移像素对应的目标角增量(rad/pixel)。
// mouse_speed_scale：意图是键鼠速度比例(rad/s/pixel)，但此 FSM 当前未读取此字段或 mouse_speed。
// min_angle/max_angle：限制参考角的边界；两轴都参考原始编码器(rad)。
// limit_angle：启用基于实际限位反馈的外向目标检查和最终力矩方向拦截。
// limit_upper_torque：仅控制上限正力矩拦截，不改变上下限目标保护及下限负力矩拦截。
// limit_target_angle：同时启用 limit_angle 时，额外把目标夹入 min_angle/max_angle；目标与反馈须同参考系。
// normalize_angle：视觉目标按最近圈数对齐到当前角；当 continuous_angle=0 时也将普通目标归一到 [-pi,pi)。
// continuous_angle：为 1 时 Apply_Angle_Rule 保留普通目标的累计圈数，不折返归一化。

void Class_Gimbal_FSM::Init(const Struct_Gimbal_FSM_Config &__config,
                            uint8_t __initial_status)
{
    // 基类清状态计时并设初始状态，随后复制上方约定的各轴配置并清空目标和标志。
    Class_FSM::Init(GIMBAL_STATUS_COUNT, __initial_status);

    config = __config;
    control_output = 0.0f;
    target_angle = 0.0f;
    target_speed = 0.0f;
    control_type = GIMBAL_CONTROL_STOP;
    angle_target_initialized = 0U;
    mode_changed_flag = 0U;
    source_initialized_ = 0U;
    last_is_keymouse_ = false;
    limit_feedback_angle_ = 0.0f;
    limit_reset_flag_ = 0U;

    if (__initial_status == GIMBAL_STATUS_ANGLE)
    {
        control_type = GIMBAL_CONTROL_ANGLE;
    }
    else if (__initial_status == GIMBAL_STATUS_SPEED)
    {
        control_type = GIMBAL_CONTROL_SPEED;
    }
}

// ===== 模式判定：FSM 内部根据 s1/s2 和键鼠状态自行决定 =====
// 键鼠来源优先检查视觉按住/数据有效；遥控器双下停止、双上可进视觉，其余有效运行组合为角度模式。
uint8_t Class_Gimbal_FSM::DetermineMode(const Struct_Gimbal_Input &input) const
{
    // ---- 键鼠模式：s1中 + s2中 ----
    if (input.is_keymouse)
    {
        if (input.vision_ready && input.vision_fresh && input.mouse_right_held)
        {
            return GIMBAL_MODE_VISION;
        }
        return GIMBAL_MODE_ANGLE;
    }

    // ---- 遥控器模式：双下 STOP，视觉就绪时进视觉，其余运行档位均为角度双环 ----
    else if (input.s1 == Remote::DOWN && input.s2 == Remote::DOWN)
    {
        return GIMBAL_MODE_STOP;
    }
    else if (input.s1 == Remote::DOWN && input.s2 == Remote::MIDDLE)
    {
        return GIMBAL_MODE_ANGLE;
    }
    else if (input.s1 == Remote::MIDDLE && input.s2 == Remote::DOWN)
    {
        return GIMBAL_MODE_SPEED;
    }

    else if (input.s1 == Remote::DOWN && input.s2 == Remote::UP)
    {
        return GIMBAL_MODE_ANGLE;
    }
    else if (input.s1 == Remote::UP && input.s2 == Remote::DOWN)
    {
        return GIMBAL_MODE_SPEED;
    }
    else if (input.s1 == Remote::MIDDLE && input.s2 == Remote::UP)
    {
        return GIMBAL_MODE_ANGLE;
    }
    else if (input.s1 == Remote::UP && input.s2 == Remote::MIDDLE)
    {
        return GIMBAL_MODE_ANGLE;
    }
    else if (input.s1 == Remote::UP && input.s2 == Remote::UP)
    {
        if (input.vision_ready && input.vision_fresh)
        {
            return GIMBAL_MODE_VISION;
        }
        return GIMBAL_MODE_ANGLE;
    }
    else
    {
        return GIMBAL_MODE_STOP;
        // 非法拨杆值（上电未连接时为 0）→ 强制 STOP，防止云台误动
        return GIMBAL_MODE_STOP;
    }
}

void Class_Gimbal_FSM::Update(const Struct_Gimbal_Input &input, float current_angle)
{
    // current_angle 为本轴当前 IMU 角(rad)：Yaw 是连续角，Pitch 是姿态角。
    // 本函数生成目标，不计算 PID/力矩；外层任务读取目标后运行控制器。
    // 1. 确定模式
    uint8_t mode_command = DetermineMode(input);
    last_mode_command_ = mode_command;

    const bool source_changed =
        (source_initialized_ == 0U) || (input.is_keymouse != last_is_keymouse_);
    source_initialized_ = 1U;
    last_is_keymouse_ = input.is_keymouse;

    // 2. 从摇杆初始化输入；视觉状态以 vision_angle 替代目标角输入。
    // Struct_Gimbal_Input.mouse_speed 和 config.mouse_speed_scale 在当前实现中未用于速度计算。
    float angle_input = input.joystick_speed;  // 遥控器默认用摇杆
    float speed_input = input.joystick_speed;

    if (mode_command == GIMBAL_MODE_VISION)
    {
        angle_input = input.vision_angle;
    }
    // 键鼠或遥控器的视觉数据失效后回到普通角度模式。

    // 3. 把模式命令映射为内部状态；角度/速度/视觉控制类型随后由状态分支更新。
    uint8_t next_status = GIMBAL_STATUS_STOP;

    switch (mode_command)
    {
    case GIMBAL_MODE_ANGLE:
        next_status = GIMBAL_STATUS_ANGLE;
        break;
    case GIMBAL_MODE_SPEED:
        next_status = GIMBAL_STATUS_SPEED;
        break;
    case GIMBAL_MODE_VISION:
        next_status = GIMBAL_STATUS_VISION;
        break;
    case GIMBAL_MODE_STOP:
    default:
        next_status = GIMBAL_STATUS_STOP;
        break;
    }

    // 模式切换和输入源切换都重建目标，避免目标从旧模式/旧控制器接续造成跳变。
    if (Get_Now_Status_Serial() != next_status)
    {
        Set_Status(next_status);
        mode_changed_flag = 1U;

        switch (next_status)
        {
        case GIMBAL_STATUS_ANGLE:
            Enter_Angle_State(current_angle);
            break;
        case GIMBAL_STATUS_SPEED:
            Enter_Speed_State();
            break;
        case GIMBAL_STATUS_VISION:
            Enter_Vision_State(current_angle);
            break;
        case GIMBAL_STATUS_STOP:
        default:
            Enter_Stop_State();
            break;
        }
    }
    else if (source_changed)
    {
        // 输入源切换时，即使控制类型相同也重新锚定，防止继承旧目标
        switch (next_status)
        {
        case GIMBAL_STATUS_ANGLE:
            Enter_Angle_State(current_angle);
            break;
        case GIMBAL_STATUS_SPEED:
            Enter_Speed_State();
            break;
        case GIMBAL_STATUS_VISION:
            Enter_Vision_State(current_angle);
            break;
        case GIMBAL_STATUS_STOP:
        default:
            Enter_Stop_State();
            break;
        }
        mode_changed_flag = 1U;
    }

    switch (Get_Now_Status_Serial())
    {
    case GIMBAL_STATUS_ANGLE:
        // 普通遥控/键鼠角度模式：离散累加角目标；摇杆受 INPUT_DEADBAND 判断。
        control_type = GIMBAL_CONTROL_ANGLE;
        if (angle_target_initialized == 0U)
        {
            target_angle = Apply_Angle_Rule(current_angle);
            angle_target_initialized = 1U;
        }
        // 键鼠按累计鼠标位移乘 mouse_angle_scale 更新目标，绝对位移不超过 2 像素时忽略。
        if (source_changed == false && input.is_keymouse &&
            Absolute_Value(input.mouse_angle_delta) > MOUSE_ANGLE_DEADBAND)
        {
            target_angle += input.mouse_angle_delta * config.mouse_angle_scale;
            target_angle = Apply_Angle_Rule(target_angle);
        }
        // 遥控摇杆幅值超过 0.01 时，每次 FSM 调用累加 joystick * angle_step。
        else if (source_changed == false && !input.is_keymouse &&
                 Absolute_Value(angle_input) > INPUT_DEADBAND)
        {
            target_angle += angle_input * config.angle_step;
            target_angle = Apply_Angle_Rule(target_angle);
        }
        control_output = target_angle;
        target_speed = 0.0f;
        break;

    case GIMBAL_STATUS_SPEED:
        // 保留速度控制接口：摇杆幅值超过 0.01 才输出摇杆 * speed_scale(rad/s)。
        control_type = GIMBAL_CONTROL_SPEED;
        if (Absolute_Value(speed_input) > INPUT_DEADBAND)
        {
            target_speed = speed_input * config.speed_scale;
        }
        else
        {
            target_speed = 0.0f;
        }
        control_output = target_speed;
        break;

    case GIMBAL_STATUS_VISION:
        // 视觉角直接设为角度目标；进入状态的首个切换周期先锚定，后续周期才跟踪视觉更新。
        control_type = GIMBAL_CONTROL_ANGLE;
        if (mode_changed_flag == 0U)
        {
            float desired_angle = Apply_Angle_Rule(angle_input);

            // 无斜坡规划：视觉目标角直接作为目标角。
            // yaw 轴仍需要 unwrap：把有界的视觉角 [-180,180] 展开到与连续反馈角
            // 相同的"圈数"参考系，否则 (有界目标 - 连续反馈) 会算出数百度的假误差
            if (config.normalize_angle != 0U)
            {
                float diff_t = desired_angle - current_angle;
                while (diff_t > PI)  { desired_angle -= FULL_CIRCLE_RAD; diff_t -= FULL_CIRCLE_RAD; }
                while (diff_t < -PI) { desired_angle += FULL_CIRCLE_RAD; diff_t += FULL_CIRCLE_RAD; }
            }

            target_angle = desired_angle;
        }
        control_output = target_angle;
        target_speed = 0.0f;
        break;

    case GIMBAL_STATUS_STOP:
    default:
        control_type = GIMBAL_CONTROL_STOP;
        control_output = 0.0f;
        target_speed = 0.0f;
        break;
    }
}

void Class_Gimbal_FSM::Set_Target_Angle(float angle)
{
    // 外部显式写目标(rad)；仍按当前配置进行归一化及可选目标上下限夹紧。
    target_angle = Apply_Angle_Rule(angle);
    angle_target_initialized = 1U;
}

void Class_Gimbal_FSM::Set_Target_Speed(float speed)
{
    // 外部显式写速度目标(rad/s)；此 setter 不改变当前模式或 control_output。
    target_speed = speed;
}

void Class_Gimbal_FSM::ReAnchor(float new_angle)
{
    // 用当前选定反馈角(rad)覆盖目标并请求控制器复位，常用于故障/输入源恢复。
    target_angle = Apply_Angle_Rule(new_angle);
    control_output = target_angle;
    target_speed = 0.0f;
    angle_target_initialized = 1U;
    mode_changed_flag = 1U;
}

bool Class_Gimbal_FSM::Would_Push_Outward(float signed_command) const
{
    // 正命令增大反馈角；等于边界也拦截，反向制动和回退始终允许。
    return config.limit_angle != 0U &&
           ((limit_feedback_angle_ >= config.max_angle && signed_command > 0.0f) ||
            (limit_feedback_angle_ <= config.min_angle && signed_command < 0.0f));
}

void Class_Gimbal_FSM::Update_Angle_Limit(float limit_feedback_angle, float current_control_angle)
{
    // 两个参数单位均为 rad。两轴均传原始编码器限位角 + 当前选定的闭环反馈角。
    limit_feedback_angle_ = limit_feedback_angle;
    if (control_type == GIMBAL_CONTROL_ANGLE &&
        Would_Push_Outward(target_angle - current_control_angle))
    {
        // 遥控/键鼠/视觉目标向外时收回到当前控制角；是否额外夹紧由配置决定。
        target_angle = Apply_Angle_Rule(current_control_angle);
        control_output = target_angle;
        limit_reset_flag_ = 1U;
    }
}

float Class_Gimbal_FSM::Limit_Torque(float torque_nm)
{
    // torque_nm 是 PID 和前馈相加、经任务幅值限幅后的最终命令(N*m)。
    if (Would_Push_Outward(torque_nm) &&
        (torque_nm < 0.0f || config.limit_upper_torque != 0U))
    {
        // 上限正力矩按配置拦截，下限负力矩始终拦截；向内力矩允许输出。
        limit_reset_flag_ = 1U;
        return 0.0f;
    }
    return torque_nm;
}

uint8_t Class_Gimbal_FSM::Take_Limit_Reset_Flag()
{
    const uint8_t flag = limit_reset_flag_;
    limit_reset_flag_ = 0U;
    return flag;
}

float Class_Gimbal_FSM::Get_Control_Output() const
{
    return control_output;
}

uint8_t Class_Gimbal_FSM::Get_Control_Type() const
{
    return control_type;
}

float Class_Gimbal_FSM::Get_Target_Angle() const
{
    return target_angle;
}

float Class_Gimbal_FSM::Get_Target_Speed() const
{
    return target_speed;
}

uint8_t Class_Gimbal_FSM::Take_Mode_Changed_Flag()
{
    // 一次性读取并清零标志；任务据此在新目标参与 PID 前复位状态。
    uint8_t tmp = mode_changed_flag;
    mode_changed_flag = 0U;
    return tmp;
}

void Class_Gimbal_FSM::Enter_Stop_State()
{
    // STOP 将控制类型和速度/输出清零；目标角保留为下次模式切换重新锚定的历史值。
    control_type = GIMBAL_CONTROL_STOP;
    control_output = 0.0f;
    target_speed = 0.0f;
}

void Class_Gimbal_FSM::Enter_Angle_State(float current_angle)
{
    // 进入普通角度状态时先将目标锚定至当前角，避免启用位置环时追逐旧目标。
    target_angle = Apply_Angle_Rule(current_angle);
    target_speed = 0.0f;
    control_output = target_angle;
    control_type = GIMBAL_CONTROL_ANGLE;
    angle_target_initialized = 1U;
}

void Class_Gimbal_FSM::Enter_Speed_State()
{
    // 进入速度状态从零速度目标开始，避免沿用上个状态的速度命令。
    target_speed = 0.0f;
    control_output = 0.0f;
    control_type = GIMBAL_CONTROL_SPEED;
}

void Class_Gimbal_FSM::Enter_Vision_State(float current_angle)
{
    // 进入视觉状态时先锚定 IMU 当前角；下一次 Update 再应用视觉角目标。
    target_angle = Apply_Angle_Rule(current_angle);
    target_speed = 0.0f;
    control_output = target_angle;
    control_type = GIMBAL_CONTROL_ANGLE;
    angle_target_initialized = 1U;
}

float Class_Gimbal_FSM::Apply_Angle_Rule(float angle) const
{
    // 所有角度单位为 rad。按开关做归一化和目标夹紧；编码器反馈限位由 Update_Angle_Limit 单独处理。
    float result = angle;

    if (config.normalize_angle != 0U && config.continuous_angle == 0U)
    {
        // 仅对非连续参考角归一到 [-pi, pi)；continuous_angle=1 时保留累计圈数。
        while (result >= PI)
        {
            result -= FULL_CIRCLE_RAD;
        }
        while (result < -PI)
        {
            result += FULL_CIRCLE_RAD;
        }
    }

    if (config.limit_angle != 0U && config.limit_target_angle != 0U)
    {
        // 仅当两个限位开关均打开时对目标夹紧；两轴关闭此项，避免混用反馈参考系。
        if (result > config.max_angle)
        {
            result = config.max_angle;
        }
        else if (result < config.min_angle)
        {
            result = config.min_angle;
        }
    }

    return result;
}
