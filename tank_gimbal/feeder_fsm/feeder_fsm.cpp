/**
 * @file   feeder_fsm.cpp
 * @brief  拨弹轮有限状态机：解释原始输入、选择 STOP/单发/连发/反转状态，输出目标给任务层。
 *
 * 输入角度和状态中的位置目标均为电机轴多圈角度(deg)，转速为 RPM；
 * 此 FSM 不生成 DJI 电流值，实际电流由 RtosTask/gimbal_task.cpp 的 PID 层计算。
 *
 * 状态流转:
 *   STOP ──┬──(连发+扳机)────→ CONTINUOUS_SHOT ──(松扳机)──→ STOP
 *          ├──(反转pending)──→ MANUAL_REVERSE ──(到位/超时)→ STOP
 *          └──(单发边缘)────→ SINGLE_SHOT ──(到位/超时)→ COOLDOWN → STOP
 *
 * 扳机信号:
 *   遥控器单发: 滚轮越过阈值产生沿，事件暂存后由 STOP 状态消费。
 *   键鼠开火: InputDispatcher 的鼠标触发输入；视觉端 fire 指令不参与开火。
 */

#include "feeder_fsm.hpp"
#include "../user/core/BSP/RemoteControl/DT7.hpp"

using Remote = BSP::REMOTE_CONTROL::RemoteController;

// 拨弹常量定义于 feeder_fsm.hpp；说明这里的量纲与状态机使用方式：
// SINGLE_SHOT_ANGLE=-3060 deg：60 deg 弹位 * 51 减速比，负号定义发射推进方向。
// MANUAL_REVERSE_ANGLE=+204 deg：4 deg 退弹行程 * 51，正方向与发射相反。
// FINISH_THRESHOLD=50 deg、FINISH_SPEED=300 RPM：正常完成需角差和转速两项同时满足。
// STALL_SPEED=50 RPM 连续 10 tick(当前 5 ms/tick)：误差仍超窗时判卡止并转入冷却。
// PHASE_CORRECTION_LIMIT=100 deg：仅接受绝对残差 <=100 deg 的下一发相位微调。
// FORWARD_SPEED=500 RPM；单发及退弹各超时 200 tick，约 1 s。
// COOLDOWN=1000 tick，约 5 s 的绝对上限；扳机松开后满足抱角条件可提前退出。
// 遥控抱角最短/最长为 30/60 tick(约 150/300 ms)，停稳阈值为角差 50 deg、转速 40 RPM。

// ============================================================================
// 初始化
// ============================================================================
void Class_Feeder_FSM::Init()
{
    // 清空基础 FSM 各状态计时；本对象回到 STOP，不沿用此前扳机、目标锁定和校正值。
    Class_FSM::Init(FEEDER_STATUS_COUNT, FEEDER_STOP);
    control_output               = 0.0f;
    raw_angle                    = 0.0f;
    accumulated_angle            = 0.0f;
    single_shot_target_angle     = 0.0f;
    manual_reverse_target_angle  = 0.0f;
    single_shot_phase_correction = 0.0f;
    last_trigger_pressed         = 0;
    current_mode                 = FEEDER_MODE_STOP;
    current_trigger_pressed      = 0;
    single_shot_pending          = 0;
    manual_reverse_pending       = 0;
    control_type                 = FEEDER_CONTROL_STOP;
    angle_initialized            = 0;
    single_shot_target_locked    = 0;
    manual_reverse_target_locked = 0;
    single_shot_stall_ticks      = 0;
}

// ============================================================================
// 多圈角度更新 — 电机驱动层已完成跨圈累计
// ============================================================================
void Class_Feeder_FSM::Update_Accumulated_Angle(float feeder_current_angle)
{
    // 驱动层已提供跨圈累计角；这里仅复制新样本，不在 FSM 内再次累加角度增量。
    raw_angle         = feeder_current_angle;
    accumulated_angle = feeder_current_angle;
    angle_initialized = 1;
}

// ROLLBACK_MARKER_SINGLE_SHOT_PHASE_CORRECTION_BEGIN
float Class_Feeder_FSM::Calculate_Single_Shot_Target() const
{
    // 单发目标 = 当前轴角 + 一发固定行程 - 上一发残差校正；方向由 SINGLE_SHOT_ANGLE 的符号决定。
    return accumulated_angle + SINGLE_SHOT_ANGLE - single_shot_phase_correction;
}
// ROLLBACK_MARKER_SINGLE_SHOT_PHASE_CORRECTION_END

// ============================================================================
// 到位判定：这里仅返回布尔结果；Update 的状态分支决定是否进入冷却或 STOP。
// ============================================================================
bool Class_Feeder_FSM::Is_Single_Shot_Finished(float current_speed)
{
    const float single_shot_error = fabs(single_shot_target_angle - accumulated_angle);

    // 正常完成需同时满足：角度误差不超过阈值(deg)，且转速绝对值不超过完成阈值(RPM)。
    if (single_shot_error <= SINGLE_SHOT_FINISH_THRESHOLD &&
        fabs(current_speed) <= SINGLE_SHOT_FINISH_SPEED_RPM)
    {
        single_shot_stall_ticks = 0;
        return true;
    }

    // 卡止早退：误差仍大于到位窗，但速度持续不超过低速阈值达到配置周期数，就不再等超时。
    // 实测会出现转子停在误差阈值边缘(如53° > 50°)的情况, 双条件永远不满足,
    // 只能干等1s超时; 这里直接判定到位, 进冷却后记录残差给相位修正
    if (single_shot_error > SINGLE_SHOT_FINISH_THRESHOLD &&
        fabs(current_speed) <= SINGLE_SHOT_STALL_SPEED_RPM)
    {
        if (++single_shot_stall_ticks >= SINGLE_SHOT_STALL_TICKS)
        {
            single_shot_stall_ticks = 0;
            return true;
        }
    }
    else
    {
        single_shot_stall_ticks = 0;
    }

    return false;
}

bool Class_Feeder_FSM::Is_Manual_Reverse_Finished(float current_speed) const
{
    // 反转结束同样要求目标误差进入单发位置窗且转速降至正常完成阈值以内。
    return fabs(manual_reverse_target_angle - accumulated_angle) <= SINGLE_SHOT_FINISH_THRESHOLD &&
           fabs(current_speed) <= SINGLE_SHOT_FINISH_SPEED_RPM;
}

// ============================================================================
// 模式与触发判定：FSM 内部根据 s1/s2 和键鼠状态自行决定
// 开火触发仅认遥控器滚轮 / 键鼠按键，视觉端 fire 指令设计上丢弃
// ============================================================================
static void DetermineFeederModeAndTrigger(const Struct_Feeder_Input &input,
                                          uint8_t &feeder_mode,
                                          uint8_t &trigger_pressed)
{
    // 拨杆编码需为 1~3；异常值 fail-safe 地输出 STOP 和无扳机。
    // 非法拨杆值（上电未连接时为 0）→ 强制 STOP，防止拨弹轮误动
    if (input.s1 < 1 || input.s1 > 3 || input.s2 < 1 || input.s2 > 3)
    {
        feeder_mode     = FEEDER_MODE_STOP;
        trigger_pressed = FEEDER_TRIGGER_NONE;
        return;
    }

    // ---- 键鼠模式：R 控制摩擦轮许可，G 在单发/连发之间选择，开火沿由上游传入 ----
    if (input.is_keymouse)
    {
        if (input.friction_on)
        {
            feeder_mode = input.is_single_shot ? FEEDER_MODE_SINGLE : FEEDER_MODE_CONTINUOUS;
        }
        else
        {
            feeder_mode = FEEDER_MODE_STOP;
        }

        trigger_pressed = input.fire_triggered ? FEEDER_TRIGGER_FORWARD : FEEDER_TRIGGER_NONE;
        return;
    }

    // ---- 遥控器模式：拨杆确定是否允许单发；滚轮决定正向发射或反向退弹 ----

    if (input.s1 == Remote::DOWN && input.s2 == Remote::DOWN)
    {
        feeder_mode = FEEDER_MODE_STOP;
    }
    else if (input.s1 == Remote::DOWN && input.s2 == Remote::MIDDLE)
    {
        feeder_mode = FEEDER_MODE_STOP;
    }
    else if (input.s1 == Remote::DOWN && input.s2 == Remote::UP)
    {
        feeder_mode = FEEDER_MODE_SINGLE;
    }
    else if (input.s1 == Remote::MIDDLE && input.s2 == Remote::DOWN)
    {
        feeder_mode = FEEDER_MODE_STOP;
    }
    else if (input.s1 == Remote::MIDDLE && input.s2 == Remote::MIDDLE)
    {
        feeder_mode = FEEDER_MODE_STOP;
    }
    else if (input.s1 == Remote::MIDDLE && input.s2 == Remote::UP)
    {
        feeder_mode = FEEDER_MODE_STOP;
    }
    else if (input.s1 == Remote::UP && input.s2 == Remote::DOWN)
    {
        feeder_mode = FEEDER_MODE_SINGLE;
    }
    else if (input.s1 == Remote::UP && input.s2 == Remote::MIDDLE)
    {
        feeder_mode = FEEDER_MODE_SINGLE;
    }
    else if (input.s1 == Remote::UP && input.s2 == Remote::UP)
    {
        feeder_mode = FEEDER_MODE_SINGLE;
    }
    else
    {
        feeder_mode = FEEDER_MODE_STOP;
    }

    // 滚轮值约为 -1~+1；单发正/反向触发阈值为 +/-0.95，回中复位采用独立的滞回阈值。
    // armed 标志使一次持续滚动只触发一次；离开该方向的复位区后才重新允许触发。
    // 连发正向触发是持续电平；反向滚轮不启动连发。
    static uint8_t fwd_armed = 1;
    static uint8_t rev_armed = 1;
    trigger_pressed = FEEDER_TRIGGER_NONE;

    if (feeder_mode == FEEDER_MODE_CONTINUOUS)
    {
        trigger_pressed = (input.scroll_value > 0.95f) ? FEEDER_TRIGGER_FORWARD : FEEDER_TRIGGER_NONE;
    }
    else if (feeder_mode == FEEDER_MODE_SINGLE)
    {
        if (fwd_armed && input.scroll_value > 0.95f)
        {
            trigger_pressed = FEEDER_TRIGGER_FORWARD;
            fwd_armed = 0;
        }
        if (fabs(input.scroll_value) < 0.20f)
        {
            fwd_armed = 1;
        }

        if (rev_armed && input.scroll_value < -0.95f)
        {
            trigger_pressed = FEEDER_TRIGGER_REVERSE;
            rev_armed = 0;
        }
        if (input.scroll_value > -0.80f)
        {
            rev_armed = 1;
        }
    }
}

// ============================================================================
// 核心: 状态机 Update — 每 5ms 由 gimbal_task 调用
// ============================================================================
void Class_Feeder_FSM::Update(const Struct_Feeder_Input &input,
                              float   feeder_current_angle,
                              float   current_speed,
                              float   current_iq)
{
    // current_iq 当前保留在接口中供诊断/扩展使用；此 FSM 的转移使用角度、转速、输入和单发许可。
    // 1. 更新驱动层给出的多圈轴角(deg)，作为本周期位置反馈和新目标的起点。
    Update_Accumulated_Angle(feeder_current_angle);

    // 2. 按键鼠/拨杆规则生成模式和当前扳机电平；视觉 fire 字段没有接入本 FSM。
    uint8_t feeder_mode = FEEDER_MODE_STOP;
    uint8_t trigger_pressed = FEEDER_TRIGGER_NONE;
    DetermineFeederModeAndTrigger(input, feeder_mode, trigger_pressed);

    // 3. 保存本周期模式/触发快照，下面的边沿和状态转换都使用同一份输入。
    current_mode            = feeder_mode;
    current_trigger_pressed = trigger_pressed;

    // 4. 只把 NONE -> 正/反方向定义为上升沿；长按保持一个状态，不重复创建边沿事件。
    const uint8_t forward_rising_edge =
        (trigger_pressed == FEEDER_TRIGGER_FORWARD) &&
        (last_trigger_pressed == FEEDER_TRIGGER_NONE);
    const uint8_t reverse_rising_edge =
        (trigger_pressed == FEEDER_TRIGGER_REVERSE) &&
        (last_trigger_pressed == FEEDER_TRIGGER_NONE);
    last_trigger_pressed = trigger_pressed;

    // 单发正沿只在待命状态排队，且当前热量判断允许时才记为待处理的一发。
    if (current_mode == FEEDER_MODE_SINGLE &&
        Get_Now_Status_Serial() == FEEDER_STOP &&
        forward_rising_edge != 0U &&
        input.single_shot_allowed)
    {
        single_shot_pending = 1;
    }

    if (current_mode != FEEDER_MODE_STOP && reverse_rising_edge != 0U)
    {
        manual_reverse_pending = 1;
    }

    // 5. STOP 输入优先级最高：清掉目标锁和相位修正，并取消状态机当前运动请求。
    if (current_mode == FEEDER_MODE_STOP)
    {
        control_type  = FEEDER_CONTROL_STOP;
        control_output = 0.0f;
        single_shot_target_locked    = 0;
        manual_reverse_target_locked = 0;
        single_shot_phase_correction = 0.0f;
        Set_Status(FEEDER_STOP);
    }
    else
    {
        switch (Get_Now_Status_Serial())
    {
    case FEEDER_STOP:
        // STOP 同时是事件消费点：连发电平、待处理退弹、待处理单发按顺序仲裁。
        control_type  = FEEDER_CONTROL_STOP;
        control_output = 0.0f;
        single_shot_target_locked    = 0;
        manual_reverse_target_locked = 0;

        if (current_mode == FEEDER_MODE_CONTINUOUS &&
            current_trigger_pressed == FEEDER_TRIGGER_FORWARD)
        {
            single_shot_phase_correction = 0.0f;
            Set_Status(FEEDER_CONTINUOUS_SHOT);
        }
        else if (current_mode != FEEDER_MODE_STOP && manual_reverse_pending != 0U)
        {
            manual_reverse_pending = 0;
            single_shot_phase_correction = 0.0f;
            manual_reverse_target_angle = accumulated_angle + MANUAL_REVERSE_ANGLE;
            manual_reverse_target_locked = 1;
            Set_Status(FEEDER_MANUAL_REVERSE);
        }
        // 挂起单发再次检查热量许可，防止事件排队期间许可已经失效。
        else if (single_shot_pending != 0U && input.single_shot_allowed)
        {
            single_shot_pending = 0;
            single_shot_target_angle = Calculate_Single_Shot_Target();
            single_shot_target_locked = 1;
            Set_Status(FEEDER_SINGLE_SHOT);
        }
        else if (current_mode == FEEDER_MODE_SINGLE &&
                 current_trigger_pressed == FEEDER_TRIGGER_FORWARD &&
                 input.single_shot_allowed)
        {
            single_shot_target_angle = Calculate_Single_Shot_Target();
            single_shot_target_locked = 1;
            Set_Status(FEEDER_SINGLE_SHOT);
        }
        break;

    case FEEDER_SINGLE_SHOT:
        // 首次进入时锁定目标，后续周期不随反馈漂移而重新计算本发行程。
        if (single_shot_target_locked == 0U)
        {
            single_shot_target_angle = Calculate_Single_Shot_Target();
            single_shot_target_locked = 1;
        }
        control_type  = FEEDER_CONTROL_ANGLE;
        control_output = single_shot_target_angle;

        // 到位条件成立或状态累计时间到配置上限，进入冷却/抱角状态。
        if (Is_Single_Shot_Finished(current_speed) ||
            Status[FEEDER_SINGLE_SHOT].Count_Time >= SINGLE_SHOT_TIMEOUT_COUNT)
        {
            Set_Status(FEEDER_SINGLE_COOLDOWN);
        }
        break;

    case FEEDER_CONTINUOUS_SHOT:
        // 输出恒定转速目标；扳机松开或拨杆离开连发组合后回 STOP。
        control_type  = FEEDER_CONTROL_SPEED;
        control_output = FORWARD_SPEED;
        single_shot_target_locked = 0;
        single_shot_phase_correction = 0.0f;

        if (current_mode != FEEDER_MODE_CONTINUOUS ||
            current_trigger_pressed != FEEDER_TRIGGER_FORWARD)
        {
            Set_Status(FEEDER_STOP);
        }
        break;

    case FEEDER_MANUAL_REVERSE:
        // 反转目标在进入时锁住，输出角度目标给位置环；不会由反转速度直接输出电流。
        if (manual_reverse_target_locked == 0U)
        {
            manual_reverse_target_angle = accumulated_angle + MANUAL_REVERSE_ANGLE;
            manual_reverse_target_locked = 1;
        }
        control_type  = FEEDER_CONTROL_ANGLE;
        control_output = manual_reverse_target_angle;

        if (Is_Manual_Reverse_Finished(current_speed) ||
            Status[FEEDER_MANUAL_REVERSE].Count_Time >= MANUAL_REVERSE_TIMEOUT_COUNT)
        {
            Set_Status(FEEDER_STOP);
        }
        break;

    case FEEDER_SINGLE_COOLDOWN:
        // ROLLBACK_MARKER_SINGLE_COOLDOWN_HOLD_BEGIN
        // 单发到位后继续保持同一位置目标，由任务的位置环主动制动拨弹轮惯性。
        control_type  = FEEDER_CONTROL_ANGLE;
        control_output = single_shot_target_angle;

        {
            // signed_error 保留实际角相对目标的方向，且用于下一发相位补偿；cooldown_error 只看大小。
            const float cooldown_signed_error = accumulated_angle - single_shot_target_angle;
            const float cooldown_error = fabs(single_shot_target_angle - accumulated_angle);
            const float cooldown_speed = fabs(current_speed);
            const uint8_t remote_hold_min_done =
                (Status[FEEDER_SINGLE_COOLDOWN].Count_Time >= SINGLE_SHOT_REMOTE_HOLD_TICKS);
            const uint8_t remote_hold_settled =
                (cooldown_error <= SINGLE_SHOT_SETTLE_THRESHOLD &&
                 cooldown_speed <= SINGLE_SHOT_SETTLE_SPEED_RPM);
            const uint8_t remote_hold_timeout =
                (Status[FEEDER_SINGLE_COOLDOWN].Count_Time >= SINGLE_SHOT_REMOTE_HOLD_TIMEOUT_TICKS);

            // 两种离开条件：固定冷却总时长到期；或扳机已松、最短抱角时间结束且停稳/超时。
            if (Status[FEEDER_SINGLE_COOLDOWN].Count_Time >= SINGLE_SHOT_COOLDOWN_TICKS ||
                (current_trigger_pressed == FEEDER_TRIGGER_NONE &&
                 remote_hold_min_done &&
                 (remote_hold_settled || remote_hold_timeout)))
            {
                // ROLLBACK_MARKER_SINGLE_SHOT_PHASE_CORRECTION_BEGIN
                // 残差只在配置小窗内传给下一发；异常大误差不自动追补，避免制造大跳变。
                if (fabs(cooldown_signed_error) <= SINGLE_SHOT_PHASE_CORRECTION_LIMIT)
                {
                    single_shot_phase_correction = cooldown_signed_error;
                }
                else
                {
                    single_shot_phase_correction = 0.0f;
                }
                // ROLLBACK_MARKER_SINGLE_SHOT_PHASE_CORRECTION_END
                Set_Status(FEEDER_STOP);
            }
        }
        // ROLLBACK_MARKER_SINGLE_COOLDOWN_HOLD_END
        break;

    default:
        control_type  = FEEDER_CONTROL_STOP;
        control_output = 0.0f;
        single_shot_target_locked    = 0;
        manual_reverse_target_locked = 0;
        single_shot_phase_correction = 0.0f;
        Set_Status(FEEDER_STOP);
        break;
    }
    } // else: current_mode != FEEDER_MODE_STOP
}

// ============================================================================
// 供 gimbal_task 查询本次 FSM 决策；control_output 是角度(deg)或转速(RPM)，由 control_type 区分。
// ============================================================================
float Class_Feeder_FSM::Get_Control_Output()
{
    // 返回值本身没有固定物理量纲；必须同时检查 Get_Control_Type()。
    return control_output;
}

uint8_t Class_Feeder_FSM::Get_Control_Type()
{
    return control_type;
}

float Class_Feeder_FSM::Get_Accumulated_Angle()
{
    // 返回驱动层输入的电机多圈轴角(deg)，不是 0~360 度单圈角。
    return accumulated_angle;
}

float Class_Feeder_FSM::Get_Single_Shot_Target_Angle()
{
    return single_shot_target_angle;
}

float Class_Feeder_FSM::Get_Manual_Reverse_Target_Angle()
{
    return manual_reverse_target_angle;
}
