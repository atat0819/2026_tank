#include "up_stair.hpp"
#include "../fsm/stair_mode_policy.hpp"
#include "../fsm/up_stair_fsm.hpp"
#include "../fsm/up_stair_behind_motor_fsm.hpp"
#include "../fsm/rear_torque_safety.hpp"
#include "../fsm/motor_recovery_fsm.hpp"
#include "can_send_task.hpp"
#include "remote_task.hpp"
#include "FreeRTOS.h"
#include "../user/core/Alg/Filter/Filter.hpp"

#include <math.h>

/*
 * 上台阶机构控制任务：负责前两台 J4340、后两台 J6248 的闭环计算和 MIT 命令发送。
 *
 * 职责分工：
 *   - can_send_task 发布 StairCommand，决定 NORMAL/IMU 模式和前后机构的动作目标。
 *   - 前腿 FSM 维护初始位/目标位及跨 ±pi 的连续编码器角度。
 *   - 后腿 FSM 维护调平/收腿、反馈有效性、软启动和机械端点单向力矩保护。
 *   - 本文件读取这些目标、计算主控角度外环，再向电机发送 MIT 速度目标。
 *
 * 四台电机的控制结构：
 *   目标角(rad) - 反馈角(rad) -> 主控角度 PID -> 轴速度目标(rad/s)
 *   -> MIT 速度内环。启用时 Kp=0；后腿 Kd 按 IMU/收腿模式分别设置。
 *   IMU 角速度阻尼写入力矩前馈，收腿前馈由后 FSM 提供；两者均参与总力矩保护。
 *   此时电机内部请求力矩可按 Kd * (目标轴速度 - 实际轴速度) + 前馈力矩估算。
 *   PID 构造参数里的 Kd 是角度外环微分系数；MIT 命令里的 Kd 是轴速度阻尼增益。
 *
 * 单位边界：
 *   - 编码器位置、收腿/回退目标：rad；外环输出和电机轴速度：rad/s。
 *   - HI14 快照和滤波变量保留原始坐标下的 deg、deg/s，
 *     通过 Get_Target_Pitch_Rad()/Get_Feedback_Pitch_Rad() 提供弧度给控制器。
 *   - HAL_GetTick() 时间戳用于毫秒超时/规划；osDelay(1U) 是让出一个 RTOS tick。
 *
 * 键鼠命令由协调层解释，本任务只执行快照：
 *   V：IMU 模式，前腿回初始位，后腿调平。
 *   第一次 B：前腿去目标位，后腿继续调平；第二次 B：前腿回初始位、后腿收腿。
 *   C：NORMAL 模式，前腿保持/返回初始位，后腿输出零力矩。
 *   双下、控制链路失效或非法档位：四台机构电机都输出零力矩。
 */

// 对象内部通道 1=前左、2=前右；实际收发 CAN ID 分别为 0x05、0x06，使用 FDCAN2。
// 后续 ctrl_Mit()/On()/getAngleRad() 的第一个参数始终传逻辑通道 1/2。
BSP::Motor::DM::J4340<2> front_4340(
    0x00, {0x05, 0x06}, {0x05, 0x06}, HAL::FDCAN::FdcanDeviceId::HAL_Fdcan2);
// 对象内部通道 1=后左、2=后右；实际收发 CAN ID 为 0x07、0x08，使用 FDCAN3。
// 后腿通过连杆支撑车尾，调平以公共 IMU Pitch 为反馈，收腿以各自编码器为反馈。
BSP::Motor::DM::J6248<2> rear_6248(
    0x00, {0x07, 0x08}, {0x07, 0x08}, HAL::FDCAN::FdcanDeviceId::HAL_Fdcan3);

// 下列 PID 构造参数顺序：Kp、Ki、Kd、输出绝对值上限、积分上限、积分分离阈值。
// 当前角度环的 Ki、Kd 都为 0，因此输出是有限幅的比例项，Kp 的量纲为 1/s。
// 输出限幅约束的是目标速度；MIT 内环的真实硬限流/限矩需在驱动器端设置。
// 前左 4340 的角度外环：弧度误差转换为 rad/s 目标，速度内环由 MIT Kd 完成。
ALG::PID::PID front_4340_left_pid = {
    120.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f,
};
// 前右角度外环保留独立增益。
ALG::PID::PID front_4340_right_pid = {
    120.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f,
};

// 前两台 4340 共用的 MIT 速度阻尼增益。
float FRONT_MIT_KD = 3.4f;

// 后 6248 的 MIT 阻尼按模式独立调节；各模式左右共用，保护估算与发送一致，范围 0 < Kd <= 5。
float REAR_IMU_MIT_KD = 3.5f;     // IMU 调平、姿态保持。
float REAR_RETRACT_MIT_KD = 3.6f; // 收腿移动、收腿位置保持及 IMU 两端位置保持。
// IMU Pitch 角速度的力矩阻尼，N·m/(rad/s)；左右共用，0=关闭，正值越大阻尼越强。
float REAR_IMU_PITCH_DAMPING = 1.0f; // 初始试调值，独立于 MIT Kd。

// 后 6248：主控只计算角度外环，电机 MIT Kd 完成轴速度内环。
// Pitch 外环输入统一为 rad，输出为 rad/s；当前输出上限为 5.4 rad/s。
// 它根据车身 Pitch 误差生成公共调平速度，再由左右方向系数映射到两台电机。
ALG::PID::PID rear_6248_pitch_pid = {
    55.0f, 0.0f, 0.0f, 7.4f, 0.0f, 0.0f,
};
// 编码器角度外环输入 rad，输出电机轴目标速度 rad/s。
// 左右各有一个，仅用于收腿位置控制；两侧参数/历史状态彼此独立。
// 收腿最终角直接交给位置环；这里的 3 rad/s 限制外环输出的目标轴速度。
ALG::PID::PID rear_6248_left_retract_pid = {
    16.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f,
};
ALG::PID::PID rear_6248_right_retract_pid = {
    18.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f,
};

// 便于调试观察的中间速度：包含方向修正，限位接管时还会被位置速度/交接比例改写。
// 真正写进 MIT 帧的速度还要经过发送处的 Limit_Velocity()。
float left_pitch_velocity = 0.0f;  // 后左电机轴调平速度，rad/s
float right_pitch_velocity = 0.0f; // 后右电机轴调平速度，rad/s


namespace
{
// 后腿 IMU 一阶低通固定权重；角度用于外环，角速度用于前馈阻尼，MIT 内环使用电机轴速度。
// 每收到新帧才执行 y = ratio*x + (1-ratio)*y；ratio 越小，平滑越强、响应越慢。
// 滤波后的 Pitch 角速度用于 MIT 力矩前馈阻尼，不进入主控角度 PID。
constexpr float REAR_PITCH_FILTER_RATIO = 0.20f; // Pitch 角度权重；越大滤波越轻、响应越快。
LPFFilter rear_6248_pitch_filter(REAR_PITCH_FILTER_RATIO);
LPFFilter rear_6248_pitch_rate_filter(0.25f);
LPFFilter rear_6248_roll_filter(0.09f);

// 对电机反馈角度做有限值保护，避免 NaN 进入 MIT 位置字段。
float SafeMotorAngle(float angle)
{
    return std::isfinite(angle) ? angle : 0.0f;
}

// 清除本任务所有主控角度 PID 的历史状态；电机内部速度反馈由 MIT 驱动器处理。
void ResetAllStairPid()
{
    // 前左右位置、后公共 Pitch、后左右位置，共五个角度外环。
    // 启动/安全停车时统一清空，避免重新允许输出后沿用停车前的历史误差。
    front_4340_left_pid.reset();
    front_4340_right_pid.reset();
    rear_6248_pitch_pid.reset();
    rear_6248_left_retract_pid.reset();
    rear_6248_right_retract_pid.reset();
}

// 向前后四个机构电机发送零力矩命令，且保留合法的位置字段。
void SendAllStairMotorsZero(float front_left_angle, float front_right_angle,
                            float rear_left_angle, float rear_right_angle)
{
    // ctrl_Mit 参数依次为：逻辑通道、位置(rad)、速度(rad/s)、Kp、Kd、前馈力矩(Nm)。
    // 必须同时清零 Kp、Kd 和前馈力矩；只把速度目标设为 0 且保留 Kd 会形成速度制动。
    // 这里 Kp=0，位置字段不会产生位置保持力矩；本函数也不发送电机 Off 命令。
    front_4340.ctrl_Mit(1, SafeMotorAngle(front_left_angle), 0.0f, 0.0f,
                        0.0f, 0.0f);
    front_4340.ctrl_Mit(2, SafeMotorAngle(front_right_angle), 0.0f, 0.0f,
                        0.0f, 0.0f);
    rear_6248.ctrl_Mit(1, SafeMotorAngle(rear_left_angle), 0.0f, 0.0f,
                       0.0f, 0.0f);
    rear_6248.ctrl_Mit(2, SafeMotorAngle(rear_right_angle), 0.0f, 0.0f,
                       0.0f, 0.0f);
}
}  // namespace



float front_left_target_velocity = 0.0f; // 前左位置环输出的目标轴速度，rad/s，供调试观察
Class_Up_Stair_FSM up_stair_fsm;          // 前 4340 上台阶状态机
Class_Up_Stair_Behind_Motor_FSM up_stair_behind_motor_fsm(Class_Up_Stair_Behind_Motor_FSM::Config::WithAngleConstants()); // 后 6248 状态机
MotorRecoveryFSM front_recovery_fsm[2];  // 前左右电机通信恢复状态机
MotorRecoveryFSM rear_recovery_fsm[2];   // 后左右电机通信恢复状态机
volatile bool dm_motor_control_ready = false; // CAN 回调和电机对象是否已准备好

// FreeRTOS 任务入口。状态机与 PID 在本任务中更新，电机反馈由 CAN 接收路径维护。
// 每轮采样 -> 安全门 -> FSM/滤波 -> 使能恢复 -> 前腿输出 -> 后腿输出 -> 让出 CPU。
extern "C" void up_stair_task(void *argument)
{
    (void)argument;
    // 等待 CAN 接收回调和电机对象准备完成，避免上电瞬间访问未初始化反馈。
    while (!dm_motor_control_ready)
    {
        osDelay(1U);
    }

    // 前后状态机独立运行，前部启动后默认回初始位置。
    // 后腿 FSM 已由全局构造装入头文件的角度/方向配置，是否输出还要经过 IMU 模式门控。
    up_stair_fsm.Init();
    const uint32_t init_tick = HAL_GetTick();
    // 初始化四个电机通信恢复状态机的时间基准。
    front_recovery_fsm[0].Init(init_tick);
    front_recovery_fsm[1].Init(init_tick);
    rear_recovery_fsm[0].Init(init_tick);
    rear_recovery_fsm[1].Init(init_tick);
    ResetAllStairPid();
    ControlInputSource last_input_source = ControlInputSource::NONE;
    // 滤波历史跨循环保存；首次启用/恢复时需要用当前有效帧重建，而非从 0 缓慢爬升。
    bool rear_imu_filter_initialized = false;
    uint32_t rear_imu_filter_tick = 0U;

    for (;;)
    {
        // 编码器反馈是原始周期角(rad)，跨 ±pi 的连续角展开由对应 FSM 完成。
        // 后腿轴速度(rad/s)用于 MIT 采样力矩估算和端点方向保护；前腿内环直接用驱动器反馈。
        const float front_left_angle = front_4340.getAngleRad(1);
        const float front_right_angle = front_4340.getAngleRad(2);
        const float rear_left_angle = rear_6248.getAngleRad(1);
        const float rear_right_angle = rear_6248.getAngleRad(2);
        const float rear_left_velocity = rear_6248.getVelocityRads(1);
        const float rear_right_velocity = rear_6248.getVelocityRads(2);
        // isConnected 的第一个参数是对象内通道，第二个参数是连接状态提示使用的电机 ID。
        // 在线仅表示反馈及时；是否允许闭环还要检查模式、机械角度以及 IMU 有效性。
        const bool front_left_online = front_4340.isConnected(1, 5);
        const bool front_right_online = front_4340.isConnected(2, 6);
        const bool rear_left_online = rear_6248.isConnected(1, 7);
        const bool rear_right_online = rear_6248.isConnected(2, 8);

        uint32_t now_tick;
        StairCommand command;
        // 模式、前后目标与时间在同一临界区采样，避免读到 B 切换过程中的前后半组目标。
        taskENTER_CRITICAL();
        command = stair_command;
        now_tick = HAL_GetTick();
        taskEXIT_CRITICAL();
        const ControlInputSnapshot input = GetControlInputSnapshot(now_tick);
        const bool source_changed = input.source != last_input_source;
        last_input_source = input.source;
        // 本地遥控/云台链路切源的首周期暂按无效输入处理，触发安全门清除旧动作。
        // 防止上一输入源的模式/动作在新输入源接管时继续执行。

        ImuControlSnapshot imu_snapshot;
        GetImuControlSnapshot(imu_snapshot);

        const bool control_link_online =
            input.source != ControlInputSource::NONE && !source_changed;
        const bool keyboard_online = input.keyboard_online && !source_changed;
        const StairModePolicy policy = EvaluateStairModePolicy(
            input.s1, input.s2, control_link_online, keyboard_online);
        const bool rear_control_enabled =
            policy.rear_attitude_enabled && command.mode == StairMode::IMU;
        // NORMAL 下前腿仍可按 FSM 保持初始位，后腿所有调平/收腿输出都受此 IMU 模式门控。

        // 双下主动停车、遥控链路掉线或档位非法时，策略会置 zero_all_torque，
        // 本分支让四个机构电机输出零力矩。
        // 双下属于主动停车；链路掉线或档位非法是异常停车，只有异常状态跳过本周期后续逻辑。
        if (policy.zero_all_torque)
        {
            // 锁存 NORMAL 并使停车前的输入采样失效，恢复后必须重新按 V。
            // generation 递增，使 can_send_task 拒绝发布在安全停车之前采到的旧命令。
            taskENTER_CRITICAL();
            stair_command = StairCommand{};
            ++stair_command_generation;
            taskEXIT_CRITICAL();
            rear_imu_filter_initialized = false;
            // 安全门生效期间不发送 On；重新预备四个恢复状态机，确保恢复后的
            // 首个正常周期分别补发一次使能帧。
            front_recovery_fsm[0].Force_Enable_On_Next_Check();
            front_recovery_fsm[1].Force_Enable_On_Next_Check();
            rear_recovery_fsm[0].Force_Enable_On_Next_Check();
            rear_recovery_fsm[1].Force_Enable_On_Next_Check();

            // 禁用前部状态机，避免安全分支继续执行位置控制。
            up_stair_fsm.Update(
                front_left_angle,
                front_right_angle,
                false,
                false,
                false,
                false,
                false,
                now_tick);

            // 禁用后部姿态状态机，清除 IMU 和左右反馈的控制权限。
            up_stair_behind_motor_fsm.Update(
                false,
                false,
                false,
                false,
                0.0f,
                0.0f,
                rear_left_angle,
                rear_right_angle,
                now_tick,
                false);
            ResetAllStairPid();
            SendAllStairMotorsZero(front_left_angle, front_right_angle,
                                   rear_left_angle, rear_right_angle);

            // 仅链路掉线或档位非法时命中：延时后跳过本周期；双下不会命中，仍会走到循环末尾延时。
            if (policy.control_fault)
            {
                osDelay(1U);
                continue;
            }
        }
        else
        {
        // 控制链路在线、档位有效且当前不是双下时，运行前后状态机、恢复逻辑和 PID 控制。
        // 前 4340 的编码器必须同时满足在线和机械角度有效，才允许位置闭环。
        const bool front_left_feedback_valid =
            front_left_online && up_stair_fsm.Is_Angle_Valid(1U, front_left_angle);
        const bool front_right_feedback_valid =
            front_right_online && up_stair_fsm.Is_Angle_Valid(2U, front_right_angle);

        // 前部独立执行协调层给出的初始位/目标位请求。
        // 任一侧反馈失效会使前 FSM 整组禁用；恢复时先返回初始位，避免重放旧 B 动作。
        up_stair_fsm.Update(
            front_left_angle,
            front_right_angle,
            front_left_feedback_valid,
            front_right_feedback_valid,
            policy.front_hold_enabled,
            policy.front_stair_command_enabled,
            command.front_target_requested,
            now_tick);

        // 后 6248 使用公共 IMU 超时门限，避免快照层和控制层配置不一致。
        // tick 差值单位 ms；只有快照标志有效、未超时、角度/角速度均有限才可进入控制。
        const bool imu_valid = imu_snapshot.valid &&
                               (now_tick - imu_snapshot.tick < IMU_CONTROL_TIMEOUT_MS) &&
                               std::isfinite(imu_snapshot.pitch_deg) &&
                               std::isfinite(imu_snapshot.pitch_rate_dps) &&
                               std::isfinite(imu_snapshot.roll_deg);
        float filtered_pitch_deg = 0.0f;
        float filtered_pitch_rate_dps = 0.0f;
        float filtered_roll_deg = 0.0f;
        if (imu_valid && rear_control_enabled)
        {
            const bool imu_new_frame = !rear_imu_filter_initialized || imu_snapshot.tick != rear_imu_filter_tick;
            if (!rear_imu_filter_initialized)
            {
                // LPFFilter 无 reset 接口；权重 1 将三路历史直接置为当前测量。
                // 重新有效时先建立正确初值，再恢复正常固定权重，避免旧历史导致姿态跳变。
                rear_6248_pitch_filter.setRatio(1.0f);
                rear_6248_pitch_rate_filter.setRatio(1.0f);
                rear_6248_roll_filter.setRatio(1.0f);
                rear_6248_pitch_filter.filter(imu_snapshot.pitch_deg);
                rear_6248_pitch_rate_filter.filter(imu_snapshot.pitch_rate_dps);
                rear_6248_roll_filter.filter(imu_snapshot.roll_deg);
                rear_6248_pitch_filter.setRatio(REAR_PITCH_FILTER_RATIO);
                rear_6248_pitch_rate_filter.setRatio(0.25f);
                rear_6248_roll_filter.setRatio(0.09f);
                rear_imu_filter_initialized = true;
            }
            else if (imu_snapshot.tick != rear_imu_filter_tick)
            {
                // 每帧仅更新一次，使用固定权重。
                // 任务循环通常快于 IMU 新帧到达；同一时间戳不反复滤波，以免改变实际滤波强度。
                rear_6248_pitch_filter.filter(imu_snapshot.pitch_deg);
                rear_6248_pitch_rate_filter.filter(imu_snapshot.pitch_rate_dps);
                rear_6248_roll_filter.filter(imu_snapshot.roll_deg);
            }
            rear_imu_filter_tick = imu_snapshot.tick;
            filtered_pitch_deg = rear_6248_pitch_filter.getOutput();
            filtered_pitch_rate_dps = rear_6248_pitch_rate_filter.getOutput();
            filtered_roll_deg = rear_6248_roll_filter.getOutput();
            // VOFA: 原始/滤波 Pitch(deg)、原始/滤波角速度(deg/s)、采样时间(s)、预留。
            if (imu_new_frame)
            {
                vofa_send(imu_snapshot.pitch_deg, filtered_pitch_deg,
                          imu_snapshot.pitch_rate_dps, filtered_pitch_rate_dps,
                          static_cast<float>(imu_snapshot.tick) * 0.001f, 0.0f);
            }
        }
        else
        {
            // IMU 失效或后腿模式关闭时不保留滤波历史；后续 FSM/发送分支负责关闭后腿输出。
            rear_imu_filter_initialized = false;
        }

        // 后 FSM 接口仍接收 deg、deg/s 的 IMU 量，以及 rad、rad/s 的电机量。
        // 它校正水平零偏并管理恢复斜坡/收腿目标；Pitch 外环通过 Rad 接口取弧度。
        // 任一后腿不可控时，后面的整组安全分支会将两台电机的速度、Kd、前馈力矩清零。
        up_stair_behind_motor_fsm.Update(
            rear_control_enabled,
            imu_valid,
            rear_left_online,
            rear_right_online,
            filtered_pitch_deg,
            filtered_pitch_rate_dps,
            rear_left_angle,
            rear_right_angle,
            now_tick,
            command.rear_retract_requested,
            rear_left_velocity,
            rear_right_velocity,
            filtered_roll_deg);

        // 前左反馈从离线恢复，或离线后需要首次/每隔 100 ms 重试使能时发送 On。
        // 对应逻辑通道 1、CAN ID 0x05；安全门恢复后也会重新请求一次使能。
        // 使能重试和闭环输出权限分开处理，On 请求本身不会绕过反馈/角度/模式检查。
        if (front_recovery_fsm[0].Should_Enable(
                front_left_online,
                now_tick))
            front_4340.On(1, BSP::Motor::DM::Model::MIT);

        // 前右反馈从离线恢复，或离线后需要首次/每隔 100 ms 重试使能时发送 On。
        // 对应逻辑通道 2、CAN ID 0x06；安全门恢复后也会重新请求一次使能。
        if (front_recovery_fsm[1].Should_Enable(
                front_right_online,
                now_tick))
            front_4340.On(2, BSP::Motor::DM::Model::MIT);

        // 后左反馈从离线恢复，或离线后需要首次/每隔 100 ms 重试使能时发送 On。
        // 对应逻辑通道 1、CAN ID 0x07；安全门恢复后也会重新请求一次使能。
        if (rear_recovery_fsm[0].Should_Enable(
                rear_left_online,
                now_tick))
            rear_6248.On(1, BSP::Motor::DM::Model::MIT);

        // 后右反馈从离线恢复，或离线后需要首次/每隔 100 ms 重试使能时发送 On。
        // 对应逻辑通道 2、CAN ID 0x08；安全门恢复后也会重新请求一次使能。
        if (rear_recovery_fsm[1].Should_Enable(
                rear_right_online,
                now_tick))
            rear_6248.On(2, BSP::Motor::DM::Model::MIT);

        // 前左角度外环输出乘速度启动比例后送入 MIT，最终目标角保持不变。
        // FSM 的目标角和反馈角都已展开成连续 rad，跨过 ±pi 不会出现整圈误差跳变。
        // 外环限制轴速度到 ±10 rad/s；启动比例与后腿收腿一致，按真实时间从 0 增至 1。
        if (front_left_feedback_valid && up_stair_fsm.Is_Enabled())
        {
            front_left_target_velocity = front_4340_left_pid.UpDate(
                up_stair_fsm.Get_Target_Angle(1),
                up_stair_fsm.Get_Position_Feedback(1)) * up_stair_fsm.Get_Speed_Scale();
            // 电机端位置 Kp=0，因此当前位置字段不闭环；实际内环力矩由 Kd 的轴速度误差产生。
            front_4340.ctrl_Mit(1, up_stair_fsm.Get_Current_Angle(1), front_left_target_velocity,
                                0.0f, FRONT_MIT_KD, 0.0f);
        }
        else
        {
            // 前左反馈无效、角度越界或前部 FSM 未启用时，清除 PID 状态并发送零力矩。
            front_4340_left_pid.reset();
            front_4340.ctrl_Mit(1, SafeMotorAngle(front_left_angle), 0.0f,
                                0.0f, 0.0f, 0.0f);
        }

        // 前右同样使用主控角度外环和电机 MIT 速度内环。
        // 右前展开后的初始角和目标角在同一坐标中，直接由 target-feedback 得到转动方向。
        if (front_right_feedback_valid && up_stair_fsm.Is_Enabled())
        {
            const float target_velocity = front_4340_right_pid.UpDate(
                up_stair_fsm.Get_Target_Angle(2),
                up_stair_fsm.Get_Position_Feedback(2)) * up_stair_fsm.Get_Speed_Scale();
            front_4340.ctrl_Mit(2, up_stair_fsm.Get_Current_Angle(2), target_velocity,
                                0.0f, FRONT_MIT_KD, 0.0f);
        }
        else
        {
            // 前右反馈无效、角度越界或前部 FSM 未启用时，清除 PID 状态并发送零力矩。
            front_4340_right_pid.reset();
            front_4340.ctrl_Mit(2, SafeMotorAngle(front_right_angle), 0.0f,
                                0.0f, 0.0f, 0.0f);
        }



        const bool rear_both_controllable =
            up_stair_behind_motor_fsm.Is_Motor_Controllable(1U) &&
            up_stair_behind_motor_fsm.Is_Motor_Controllable(2U);
        // 任一后腿反馈/角度无效时，整组退出速度内环，必须同时将 Kd 清零。
        // 后 FSM 的 RECOVERING 也是可控状态；“可控”不表示恢复比例已经到 1。
        if (up_stair_behind_motor_fsm.Get_State() ==
                UP_STAIR_BEHIND_MOTOR_DISABLED ||
            !rear_both_controllable)
        {
            rear_6248_pitch_pid.reset();
            rear_6248_left_retract_pid.reset();
            rear_6248_right_retract_pid.reset();
            rear_6248.ctrl_Mit(1, SafeMotorAngle(rear_left_angle), 0.0f,
                               0.0f, 0.0f, 0.0f);
            rear_6248.ctrl_Mit(2, SafeMotorAngle(rear_right_angle), 0.0f,
                               0.0f, 0.0f, 0.0f);
        }
        else if (up_stair_behind_motor_fsm.Uses_Position_Control())
        {
            // 此分支用于收腿移动/收腿保持；发送前统一执行端点单向力矩保护。
            rear_6248_pitch_pid.reset();
            // 编码器外环输出已是 rad/s，不再叠加 Pitch 的左右方向系数。
            // 目标是 FSM 给出的最终收腿位置(rad)，反馈是同一编码器坐标中的连续位置(rad)。
            // 位置误差已经决定了电机正负方向，左右镜像关系体现在各自的目标角中。
            const float left_target_velocity = rear_6248_left_retract_pid.UpDate(
                up_stair_behind_motor_fsm.Get_Position_Target_Angle(1U),
                up_stair_behind_motor_fsm.Get_Position_Feedback(1U));
            const float right_target_velocity = rear_6248_right_retract_pid.UpDate(
                up_stair_behind_motor_fsm.Get_Position_Target_Angle(2U),
                up_stair_behind_motor_fsm.Get_Position_Feedback(2U));
            // Limit_Velocity 估算速度环+收腿前馈总力矩 -> 软启动/增益/机械保护 -> ±30 Nm 采样限幅
            // -> 扣除实际发送的前馈后换回 rad/s 目标。估算和发送共用 REAR_RETRACT_MIT_KD。
            // 真实帧间力矩由驱动器速度反馈决定，±30 Nm 采样估算约束不替代驱动器硬限流/限矩。
            rear_6248.ctrl_Mit(
                1, SafeMotorAngle(rear_left_angle),
                up_stair_behind_motor_fsm.Limit_Velocity(1U, left_target_velocity, REAR_RETRACT_MIT_KD),
                0.0f, REAR_RETRACT_MIT_KD, up_stair_behind_motor_fsm.Get_Retract_Feedforward_Torque(1U));
            rear_6248.ctrl_Mit(
                2, SafeMotorAngle(rear_right_angle),
                up_stair_behind_motor_fsm.Limit_Velocity(2U, right_target_velocity, REAR_RETRACT_MIT_KD),
                0.0f, REAR_RETRACT_MIT_KD, up_stair_behind_motor_fsm.Get_Retract_Feedforward_Torque(2U));
        }
        else if (up_stair_behind_motor_fsm.Uses_Attitude_Control())
        {
            // V 进入 IMU 后，RECOVERING 和 ATTITUDE_HOLD 都进入本分支。
            // 目标来自固定工作 Pitch 配置；按 V 不会把当前姿态自动捕获成新的工作目标。
            // 两个 Rad 接口使用同一 HI14 原始坐标，PID 输出直接是 rad/s。
            const float pitch_target_velocity_rad_s = rear_6248_pitch_pid.UpDate(
                up_stair_behind_motor_fsm.Get_Target_Pitch_Rad(),
                up_stair_behind_motor_fsm.Get_Feedback_Pitch_Rad());
            // 两侧共用一个车身 Pitch 误差，经各自方向系数形成镜像安装下的电机轴速度请求。
            left_pitch_velocity =
                static_cast<float>(up_stair_behind_motor_fsm.Get_Motor_Direction(1)) *
                pitch_target_velocity_rad_s;
            right_pitch_velocity =
                static_cast<float>(up_stair_behind_motor_fsm.Get_Motor_Direction(2)) *
                pitch_target_velocity_rad_s;

            // 车身 Pitch 增大时取正角速度；阻尼始终反向，deg/s 在此转为 rad/s。
            const float pitch_damping_torque_nm = -REAR_IMU_PITCH_DAMPING *
                up_stair_behind_motor_fsm.Get_Pitch_Rate_Dps() * (3.14159265358979323846f / 180.0f);
            // FSM 返回已做左右方向映射的原始 Roll 补偿；与 Pitch 阻尼合并后统一保护。
            const float left_attitude_kd = up_stair_behind_motor_fsm.Uses_Limit_Hold(1U)
                ? REAR_RETRACT_MIT_KD : REAR_IMU_MIT_KD;
            const float right_attitude_kd = up_stair_behind_motor_fsm.Uses_Limit_Hold(2U)
                ? REAR_RETRACT_MIT_KD : REAR_IMU_MIT_KD;
            const float left_attitude_feedforward_nm =
                up_stair_behind_motor_fsm.Uses_Limit_Hold(1U) ? 0.0f :
                static_cast<float>(up_stair_behind_motor_fsm.Get_Motor_Direction(1)) *
                pitch_damping_torque_nm + up_stair_behind_motor_fsm.Get_Raw_Roll_Torque(1U);
            const float right_attitude_feedforward_nm =
                up_stair_behind_motor_fsm.Uses_Limit_Hold(2U) ? 0.0f :
                static_cast<float>(up_stair_behind_motor_fsm.Get_Motor_Direction(2)) *
                pitch_damping_torque_nm + up_stair_behind_motor_fsm.Get_Raw_Roll_Torque(2U);
            const float left_feedforward_nm = StairTorqueSafety::ClampJ6248Torque(
                up_stair_behind_motor_fsm.Limit_Torque(1U, left_attitude_feedforward_nm));
            const float right_feedforward_nm = StairTorqueSafety::ClampJ6248Torque(
                up_stair_behind_motor_fsm.Limit_Torque(2U, right_attitude_feedforward_nm));

            // 两端保持的独立位置 P 在 Limit_Velocity 中计算；收腿 PID 不参与。
            rear_6248_left_retract_pid.reset();
            rear_6248_right_retract_pid.reset();
            // Limit_Velocity 合并速度环、Pitch 阻尼和 Roll 补偿执行保护，再扣除实际发送的前馈。
            // 下限禁止负力矩，上限禁止正力矩；端点内侧 0.5° 按触限处理。
            // 拦截按速度项+前馈的采样总力矩执行，不能只判断目标速度的符号。
            rear_6248.ctrl_Mit(
                1, SafeMotorAngle(rear_left_angle),
                up_stair_behind_motor_fsm.Limit_Velocity(1U, left_pitch_velocity, left_attitude_kd, left_attitude_feedforward_nm),
                0.0f, left_attitude_kd, left_feedforward_nm);
            rear_6248.ctrl_Mit(
                2, SafeMotorAngle(rear_right_angle),
                up_stair_behind_motor_fsm.Limit_Velocity(2U, right_pitch_velocity, right_attitude_kd, right_attitude_feedforward_nm),
                0.0f, right_attitude_kd, right_feedforward_nm);
        }
        else
        {
            // 兜底：后 FSM 未请求任何有效控制模式时，复位主控 PID 并关闭 MIT 速度反馈增益。
            rear_6248_pitch_pid.reset();
            rear_6248_left_retract_pid.reset();
            rear_6248_right_retract_pid.reset();
            rear_6248.ctrl_Mit(1, SafeMotorAngle(rear_left_angle), 0.0f,
                               0.0f, 0.0f, 0.0f);
            rear_6248.ctrl_Mit(2, SafeMotorAngle(rear_right_angle), 0.0f,
                               0.0f, 0.0f, 0.0f);
        }

        }

        osDelay(1U); // 让出一个 RTOS tick；实际循环时间还包含本轮计算/发送的开销。
        // 规划/超时使用真实 tick 差值，本循环并非硬件定时器严格固定周期。
    }
}
