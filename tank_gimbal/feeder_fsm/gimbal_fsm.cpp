#include "gimbal_fsm.hpp"
#include "../user/core/BSP/RemoteControl/DT7.hpp"

namespace
{
constexpr float INPUT_DEADBAND = 0.01f;
constexpr float MOUSE_ANGLE_DEADBAND = 2.0f; // pixels
constexpr float PI = 3.14159265358979323846f;
constexpr float FULL_CIRCLE_RAD = 2.0f * PI;

float Absolute_Value(float value)
{
    return (value >= 0.0f) ? value : -value;
}

// 原始编码器角度为度：+60 度拦正方向，-60 度拦负方向。
bool YawWouldPushOutward(float encoder_deg, float signed_command)
{
    return (encoder_deg >= 60.0f && signed_command > 0.0f) ||
           (encoder_deg <= -60.0f && signed_command < 0.0f);
}
}

using Remote = BSP::REMOTE_CONTROL::RemoteController;

void Class_Gimbal_FSM::Init(const Struct_Gimbal_FSM_Config &__config,
                            uint8_t __initial_status)
{
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
    pitch_came_from_double_down_ = false;
    pitch_start_locked_ = false;
    pitch_neutral_seen_ = false;
    pitch_unlock_is_keymouse_ = false;
    yaw_encoder_deg_ = 0.0f;
    yaw_limit_reset_flag_ = 0U;

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
        return GIMBAL_MODE_ANGLE;
    }

    else if (input.s1 == Remote::DOWN && input.s2 == Remote::UP)
    {
        return GIMBAL_MODE_ANGLE;
    }
    else if (input.s1 == Remote::UP && input.s2 == Remote::DOWN)
    {
        return GIMBAL_MODE_ANGLE;
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
    // 1. 确定模式
    uint8_t mode_command = DetermineMode(input);
    last_mode_command_ = mode_command;

    const bool source_changed =
        (source_initialized_ == 0U) || (input.is_keymouse != last_is_keymouse_);
    source_initialized_ = 1U;
    last_is_keymouse_ = input.is_keymouse;

    // 2. 计算 angle_input 和 speed_input
    float angle_input = input.joystick_speed;  // 遥控器默认用摇杆
    float speed_input = input.joystick_speed;

    if (mode_command == GIMBAL_MODE_VISION)
    {
        angle_input = input.vision_angle;
    }
    // 键鼠或遥控器的视觉数据失效后回到普通角度模式。

    // 3. 状态转移（与原 Update 完全一致）
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
        control_type = GIMBAL_CONTROL_ANGLE;
        if (angle_target_initialized == 0U)
        {
            target_angle = Apply_Angle_Rule(current_angle);
            angle_target_initialized = 1U;
        }
        if (source_changed == false && input.is_keymouse &&
            Absolute_Value(input.mouse_angle_delta) > MOUSE_ANGLE_DEADBAND)
        {
            target_angle += input.mouse_angle_delta * config.mouse_angle_scale;
            target_angle = Apply_Angle_Rule(target_angle);
        }
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
    target_angle = Apply_Angle_Rule(angle);
    angle_target_initialized = 1U;
}

void Class_Gimbal_FSM::Set_Target_Speed(float speed)
{
    target_speed = speed;
}

void Class_Gimbal_FSM::ReAnchor(float new_angle)
{
    target_angle = Apply_Angle_Rule(new_angle);
    control_output = target_angle;
    target_speed = 0.0f;
    angle_target_initialized = 1U;
    mode_changed_flag = 1U;
}

void Class_Gimbal_FSM::Update_Yaw_Limit(float encoder_deg, float current_imu_angle)
{
    yaw_encoder_deg_ = encoder_deg;
    if (control_type == GIMBAL_CONTROL_ANGLE &&
        YawWouldPushOutward(encoder_deg, target_angle - current_imu_angle))
    {
        // 目标仍朝机械边界外时收回到当前 IMU 角，防止遥控器/键鼠累积目标后难以反向。
        // 视觉目标也在每次 Update 后通过此处检查；不改变控制反馈来源。
        target_angle = Apply_Angle_Rule(current_imu_angle);
        control_output = target_angle;
        yaw_limit_reset_flag_ = 1U;
    }
}

float Class_Gimbal_FSM::Limit_Yaw_Torque(float torque_nm)
{
    if (YawWouldPushOutward(yaw_encoder_deg_, torque_nm))
    {
        // 所有 PID 与前馈相加后再拦截，向内的制动/回退力矩仍允许输出。
        yaw_limit_reset_flag_ = 1U;
        return 0.0f;
    }
    return torque_nm;
}

uint8_t Class_Gimbal_FSM::Take_Yaw_Limit_Reset_Flag()
{
    const uint8_t flag = yaw_limit_reset_flag_;
    yaw_limit_reset_flag_ = 0U;
    return flag;
}

Class_Gimbal_FSM::PitchStartDecision Class_Gimbal_FSM::Update_Pitch_Start_Gate(
    bool is_double_down, bool is_keymouse, float pitch_stick, float mouse_delta_y)
{
    if (is_double_down)
    {
        pitch_came_from_double_down_ = true;
        pitch_start_locked_ = false;
        pitch_neutral_seen_ = false;
        pitch_unlock_is_keymouse_ = is_keymouse;
        return PitchStartDecision::Normal;
    }

    if (control_type != GIMBAL_CONTROL_STOP && pitch_came_from_double_down_)
    {
        pitch_came_from_double_down_ = false;
        pitch_start_locked_ = true;
        pitch_unlock_is_keymouse_ = is_keymouse;
    }
    if (!pitch_start_locked_)
    {
        return PitchStartDecision::Normal;
    }
    if (pitch_unlock_is_keymouse_ != is_keymouse)
    {
        pitch_unlock_is_keymouse_ = is_keymouse;
        pitch_neutral_seen_ = false;
    }
    // 当前输入先进入死区，再等待首次上移；切档时已上移不能直接解锁。
    const float unlock_input = is_keymouse ? mouse_delta_y : pitch_stick;
    const float deadband = is_keymouse ? MOUSE_ANGLE_DEADBAND : INPUT_DEADBAND;
    if (unlock_input >= -deadband && unlock_input <= deadband)
    {
        pitch_neutral_seen_ = true;
    }
    if (control_type != GIMBAL_CONTROL_STOP && pitch_neutral_seen_ &&
        unlock_input > deadband)
    {
        pitch_start_locked_ = false;
        return PitchStartDecision::ReanchorAndRelease;
    }
    return PitchStartDecision::HoldZero;
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
    uint8_t tmp = mode_changed_flag;
    mode_changed_flag = 0U;
    return tmp;
}

void Class_Gimbal_FSM::Enter_Stop_State()
{
    control_type = GIMBAL_CONTROL_STOP;
    control_output = 0.0f;
    target_speed = 0.0f;
}

void Class_Gimbal_FSM::Enter_Angle_State(float current_angle)
{
    target_angle = Apply_Angle_Rule(current_angle);
    target_speed = 0.0f;
    control_output = target_angle;
    control_type = GIMBAL_CONTROL_ANGLE;
    angle_target_initialized = 1U;
}

void Class_Gimbal_FSM::Enter_Speed_State()
{
    target_speed = 0.0f;
    control_output = 0.0f;
    control_type = GIMBAL_CONTROL_SPEED;
}

void Class_Gimbal_FSM::Enter_Vision_State(float current_angle)
{
    target_angle = Apply_Angle_Rule(current_angle);
    target_speed = 0.0f;
    control_output = target_angle;
    control_type = GIMBAL_CONTROL_ANGLE;
    angle_target_initialized = 1U;
}

float Class_Gimbal_FSM::Apply_Angle_Rule(float angle) const
{
    float result = angle;

    if (config.normalize_angle != 0U && config.continuous_angle == 0U)
    {
        while (result >= PI)
        {
            result -= FULL_CIRCLE_RAD;
        }
        while (result < -PI)
        {
            result += FULL_CIRCLE_RAD;
        }
    }

    if (config.limit_angle != 0U)
    {
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
