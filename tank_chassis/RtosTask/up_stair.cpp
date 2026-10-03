#include "up_stair.hpp"
#include "../fsm/stair_mode_policy.hpp"
#include "../fsm/up_stair_fsm.hpp"
#include "../fsm/up_stair_behind_motor_fsm.hpp"
#include "../fsm/motor_recovery_fsm.hpp"
#include "../fsm/rear_torque_safety.hpp"
#include "can_send_task.hpp"
#include "imu_task.hpp"
#include "FreeRTOS.h"
#include "../user/core/HAL/UART/uart_hal.hpp"

#include <math.h>
#include <string.h>

// 前部两台 J4340：负责卡住台阶、返回初始位置以及正常姿态保持。
BSP::Motor::DM::J4340<2> front_4340(
    0x00, {0x05, 0x06}, {0x05, 0x06}, HAL::FDCAN::FdcanDeviceId::HAL_Fdcan1);
// 后部两台 J6248：通过连杆支撑车尾，并执行 pitch 水平控制。
BSP::Motor::DM::J6248<2> rear_6248(
    0x00, {0x07, 0x08}, {0x07, 0x08}, HAL::FDCAN::FdcanDeviceId::HAL_Fdcan3);

// 前左 4340 的位置环和速度环 PID，数组下标 0=位置环、1=速度环。
ALG::PID::PID front_4340_left_pid[2] = {
    {15.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f},
    {3.0f, 0.0f, 0.0f, 27.0f, 0.0f, 0.0f},
};
// 前右 4340 的位置环和速度环 PID，左右使用独立参数。
ALG::PID::PID front_4340_right_pid[2] = {
    {8.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f},
    {0.8f, 0.0f, 0.0f, 27.0f, 0.0f, 0.0f},
};

// 后 6248 不使用积分项，采用“pitch 角度环 → 角速度环 → 力矩”的串级控制。
// 具体增益需要结合整车负载、连杆方向和实车响应继续标定。
// 后部 pitch 轴的角度环和角速度环 PID。
ALG::PID::PID rear_6248_pitch_pid[2] = {
    {1.5f, 0.0f, 0.0f, 30.0f, 0.0f, 0.0f},
    {0.8f, 0.0f, 0.0f, 40.0f, 0.0f, 0.0f},
};
ALG::PID::PID rear_6248_left_retract_pid[2] = {
    {5.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f},
    {1.0f, 0.02f, 0.0f, 8.0f, 1.0f, 1.0f},
};
ALG::PID::PID rear_6248_right_retract_pid[2] = {
    {5.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f},
    {1.0f, 0.02f, 0.0f, 8.0f, 1.0f, 1.0f},
};

 float left_pitch_torque =0.0f;  // 后左 6248 的目标力矩
 float right_pitch_torque =0.0f; // 后右 6248 的目标力矩


namespace
{
// VOFA+：UART10，115200，8N1，JustFloat；x1~x9 对应通道 0~8。
// 返回 true 表示成功启动 DMA；仅由上台阶任务调用，共用一个静态发送缓冲区。
bool vofa_send(float x1, float x2, float x3, float x4, float x5,
               float x6, float x7, float x8, float x9)
{
    HAL::UART::IUartDevice &uart =
        HAL::UART::get_uart_bus_instance().get_uart10();
    static float vofa_frame[10]; // DMA 使用静态缓冲区：9 个 float + 帧尾。
    // DMA 忙时不能覆盖缓冲区，直接跳过本帧。
    if (uart.get_handle()->gState != HAL_UART_STATE_READY)
    {
        return false;
    }

    vofa_frame[0] = x1;
    vofa_frame[1] = x2;
    vofa_frame[2] = x3;
    vofa_frame[3] = x4;
    vofa_frame[4] = x5;
    vofa_frame[5] = x6;
    vofa_frame[6] = x7;
    vofa_frame[7] = x8;
    vofa_frame[8] = x9;
    const uint32_t vofa_tail = 0x7F800000U; // 小端帧尾：00 00 80 7F。
    memcpy(&vofa_frame[9], &vofa_tail, sizeof(vofa_tail));
    HAL::UART::Data tx_data{reinterpret_cast<uint8_t *>(vofa_frame),
                           static_cast<uint16_t>(sizeof(vofa_frame))};
    return uart.transmit_dma(tx_data);
}

// 对电机反馈角度做有限值保护，避免 NaN 进入 MIT 位置字段。
float SafeMotorAngle(float angle)
{
    return std::isfinite(angle) ? angle : 0.0f;
}

// 清除前后所有位置、速度和姿态 PID 的内部历史状态。
void ResetAllStairPid()
{
    // 进入双下、失联或其他禁用状态时清除四组 PID 的历史误差和积分量。
    front_4340_left_pid[0].reset();
    front_4340_left_pid[1].reset();
    front_4340_right_pid[0].reset();
    front_4340_right_pid[1].reset();
    rear_6248_pitch_pid[0].reset();
    rear_6248_pitch_pid[1].reset();
    rear_6248_left_retract_pid[0].reset();
    rear_6248_left_retract_pid[1].reset();
    rear_6248_right_retract_pid[0].reset();
    rear_6248_right_retract_pid[1].reset();
}

// 向前后四个机构电机发送当前位置加零力矩命令。
void SendAllStairMotorsZero(float front_left_angle, float front_right_angle,
                            float rear_left_angle, float rear_right_angle)
{
    // 双下或链路故障时，四个机构电机统一发送零力矩，但仍带当前安全角度。
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

float front_left_target_velocity = 0.0f; // 前左位置环输出的目标角速度
float front_left_target_torque = 0.0f;   // 前左速度环输出的目标力矩
Class_Up_Stair_FSM up_stair_fsm;          // 前 4340 上台阶状态机
Class_Up_Stair_Behind_Motor_FSM up_stair_behind_motor_fsm(Class_Up_Stair_Behind_Motor_FSM::Config::WithAngleConstants()); // 后 6248 状态机
MotorRecoveryFSM front_recovery_fsm[2];  // 前左右电机通信恢复状态机
MotorRecoveryFSM rear_recovery_fsm[2];   // 后左右电机通信恢复状态机
volatile bool dm_motor_control_ready = false; // CAN 回调和电机对象是否已准备好

extern "C" void up_stair_task(void *argument)
{
    (void)argument;
    // 等待 CAN 接收回调和电机对象准备完成，避免上电瞬间访问未初始化反馈。
    while (!dm_motor_control_ready)
    {
        osDelay(1U);
    }

    // 初始化前部上台阶状态机，并同步当前已经产生的 B 动作序号。
    up_stair_fsm.Init(stair_action_sequence);
    const uint32_t init_tick = HAL_GetTick();
    // 初始化四个电机通信恢复状态机的时间基准。
    front_recovery_fsm[0].Init(init_tick);
    front_recovery_fsm[1].Init(init_tick);
    rear_recovery_fsm[0].Init(init_tick);
    rear_recovery_fsm[1].Init(init_tick);
    ResetAllStairPid();
    ControlInputSource last_input_source = ControlInputSource::NONE;
    uint32_t vofa_last_send_tick = init_tick;
    uint32_t vofa_last_sample_tick = init_tick;

    for (;;)
    {
        // 读取四个机构电机的角度、速度和在线状态，后续状态机只使用这些原始数据。
        const float front_left_angle = front_4340.getAngleRad(1);
        const float front_right_angle = front_4340.getAngleRad(2);
        const float front_left_velocity = front_4340.getVelocityRads(1);
        const float front_right_velocity = front_4340.getVelocityRads(2);
        const float rear_left_angle = rear_6248.getAngleRad(1);
        const float rear_right_angle = rear_6248.getAngleRad(2);
        const float rear_left_velocity = rear_6248.getVelocityRads(1);
        const float rear_right_velocity = rear_6248.getVelocityRads(2);
        const bool front_left_online = front_4340.isConnected(1, 5);
        const bool front_right_online = front_4340.isConnected(2, 6);
        const bool rear_left_online = rear_6248.isConnected(1, 7);
        const bool rear_right_online = rear_6248.isConnected(2, 8);

        uint32_t now_tick;
        uint32_t rear_action_sequence;
        // 动作序号与时间在同一临界区采样；输入源快照由公共函数完成。
        taskENTER_CRITICAL();
        rear_action_sequence = rear_retract_action_sequence;
        now_tick = HAL_GetTick();
        taskEXIT_CRITICAL();
        const ControlInputSnapshot input = GetControlInputSnapshot(now_tick);
        const bool source_changed = input.source != last_input_source;
        last_input_source = input.source;

        ImuControlSnapshot imu_snapshot;
        GetImuControlSnapshot(imu_snapshot);

        // 在控制权限判断前发送，停车或遥控离线时仍可观察有效 IMU 数据。
        // 一帧 40 字节约需 3.47 ms；至少间隔 5 ms，只上报有效的新快照。
        if ((now_tick - vofa_last_send_tick >= 5U) &&
            imu_snapshot.tick != vofa_last_sample_tick &&
            imu_snapshot.valid && (now_tick - imu_snapshot.tick < 20U) &&
            std::isfinite(imu_snapshot.roll_deg) &&
            std::isfinite(imu_snapshot.roll_rate_dps) &&
            std::isfinite(imu_snapshot.accel_x_mps2) &&
            std::isfinite(imu_snapshot.accel_y_mps2) &&
            std::isfinite(imu_snapshot.accel_z_mps2) &&
            std::isfinite(imu_snapshot.yaw_rate_dps))
        {
            if (vofa_send(
                    imu_snapshot.pitch_deg,       // 通道 0：Pitch，度
                    imu_snapshot.pitch_rate_dps,  // 通道 1：陀螺 Y，度/秒，已减零偏
                    imu_snapshot.roll_deg,        // 通道 2：Roll，度
                    imu_snapshot.roll_rate_dps,   // 通道 3：陀螺 X，度/秒，已减零偏
                    imu_snapshot.accel_x_mps2,    // 通道 4：加速度 X，米/秒²
                    imu_snapshot.accel_y_mps2,    // 通道 5：加速度 Y，米/秒²
                    imu_snapshot.accel_z_mps2,    // 通道 6：加速度 Z，米/秒²
                    imu_snapshot.yaw_rate_dps,    // 通道 7：陀螺 Z，度/秒，已减零偏
                    static_cast<float>(imu_snapshot.tick - init_tick))) // 通道 8：读取完成时间，毫秒
            {
                vofa_last_send_tick = now_tick;
                vofa_last_sample_tick = imu_snapshot.tick;
            }
        }

        const bool control_link_online =
            input.source != ControlInputSource::NONE && !source_changed;
        const bool keyboard_online = input.keyboard_online && !source_changed;
        const StairModePolicy policy = EvaluateStairModePolicy(
            input.s1, input.s2, control_link_online, keyboard_online);

        // 双下主动停车、遥控链路掉线或档位非法时，策略会置 zero_all_torque，
        // 本分支让四个机构电机输出零力矩。
        // 双下属于主动停车；链路掉线或档位非法是异常停车，只有异常状态跳过本周期后续逻辑。
        if (policy.zero_all_torque)
        {
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
                stair_action_sequence);

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
                false,
                rear_action_sequence);
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

        // 更新前部 4340 状态机：根据反馈、控制权限和 B 动作序号决定目标位置。
        up_stair_fsm.Update(
            front_left_angle,
            front_right_angle,
            front_left_feedback_valid,
            front_right_feedback_valid,
            policy.front_hold_enabled,
            policy.front_stair_command_enabled,
            stair_action_sequence);

        // 后 6248 只接受完整且不超过 20 ms 的 IMU 快照。
        const bool imu_valid = imu_snapshot.valid &&
                               (now_tick - imu_snapshot.tick < 20U);

        // 更新后部 6248 状态机：校验 IMU/编码器，并管理姿态力矩软启动和限幅。
        up_stair_behind_motor_fsm.Update(
            policy.rear_attitude_enabled,
            imu_valid,
            rear_left_online,
            rear_right_online,
            imu_snapshot.pitch_deg,
            imu_snapshot.pitch_rate_dps,
            rear_left_angle,
            rear_right_angle,
            now_tick,
            policy.rear_retract_command_enabled,
            rear_action_sequence);

        // 前左反馈从离线恢复，或离线后需要首次/每隔 100 ms 重试使能时发送 On。
        // 对应逻辑通道 1、CAN ID 1；安全门恢复后也会重新请求一次使能。
        if (front_recovery_fsm[0].Should_Enable(
                front_left_online,
                now_tick))
            front_4340.On(1, BSP::Motor::DM::Model::MIT);

        // 前右反馈从离线恢复，或离线后需要首次/每隔 100 ms 重试使能时发送 On。
        // 对应逻辑通道 2、CAN ID 2；安全门恢复后也会重新请求一次使能。
        if (front_recovery_fsm[1].Should_Enable(
                front_right_online,
                now_tick))
            front_4340.On(2, BSP::Motor::DM::Model::MIT);

        // 后左反馈从离线恢复，或离线后需要首次/每隔 100 ms 重试使能时发送 On。
        // 对应逻辑通道 1、CAN ID 3；安全门恢复后也会重新请求一次使能。
        if (rear_recovery_fsm[0].Should_Enable(
                rear_left_online,
                now_tick))
            rear_6248.On(1, BSP::Motor::DM::Model::MIT);

        // 后右反馈从离线恢复，或离线后需要首次/每隔 100 ms 重试使能时发送 On。
        // 对应逻辑通道 2、CAN ID 4；安全门恢复后也会重新请求一次使能。
        if (rear_recovery_fsm[1].Should_Enable(
                rear_right_online,
                now_tick))
            rear_6248.On(2, BSP::Motor::DM::Model::MIT);

        // 前左反馈在线、角度有效且前部 FSM 已启用时，才运行位置环和速度环输出控制力矩。
        if (front_left_feedback_valid && up_stair_fsm.Is_Enabled())
        {
            front_left_target_velocity = front_4340_left_pid[0].UpDate(
                up_stair_fsm.Get_Target_Angle(1),
                up_stair_fsm.Get_Position_Feedback(1));
            front_left_target_torque = front_4340_left_pid[1].UpDate(
                front_left_target_velocity, front_left_velocity);
            front_4340.ctrl_Mit(1, up_stair_fsm.Get_Current_Angle(1), 0.0f,
                                0.0f, 0.0f, front_left_target_torque);
        }
        else
        {
            // 前左反馈无效、角度越界或前部 FSM 未启用时，清除 PID 状态并发送零力矩。
            front_4340_left_pid[0].reset();
            front_4340_left_pid[1].reset();
            front_4340.ctrl_Mit(1, SafeMotorAngle(front_left_angle), 0.0f,
                                0.0f, 0.0f, 0.0f);
        }

        // 前右反馈在线、角度有效且前部 FSM 已启用时，使用右侧独立 PID 执行位置和速度闭环。
        if (front_right_feedback_valid && up_stair_fsm.Is_Enabled())
        {
            const float target_velocity = front_4340_right_pid[0].UpDate(
                up_stair_fsm.Get_Target_Angle(2),
                up_stair_fsm.Get_Position_Feedback(2));
            const float target_torque = front_4340_right_pid[1].UpDate(
                target_velocity, front_right_velocity);
            front_4340.ctrl_Mit(2, up_stair_fsm.Get_Current_Angle(2), 0.0f,
                                0.0f, 0.0f, target_torque);
        }
        else
        {
            // 前右反馈无效、角度越界或前部 FSM 未启用时，清除 PID 状态并发送零力矩。
            front_4340_right_pid[0].reset();
            front_4340_right_pid[1].reset();
            front_4340.ctrl_Mit(2, SafeMotorAngle(front_right_angle), 0.0f,
                                0.0f, 0.0f, 0.0f);
        }

        const bool rear_both_controllable =
            up_stair_behind_motor_fsm.Is_Motor_Controllable(1U) &&
            up_stair_behind_motor_fsm.Is_Motor_Controllable(2U);
        // 后部 FSM 被禁用，或左右任一电机反馈/机械角度无效时，整组后腿复位并输出零力矩。
        if (up_stair_behind_motor_fsm.Get_State() ==
                UP_STAIR_BEHIND_MOTOR_DISABLED ||
            !rear_both_controllable)
        {
            // 后部状态机禁用时，清空姿态 PID 并明确发送零力矩。
            rear_6248_pitch_pid[0].reset();
            rear_6248_pitch_pid[1].reset();
            rear_6248_left_retract_pid[0].reset();
            rear_6248_left_retract_pid[1].reset();
            rear_6248_right_retract_pid[0].reset();
            rear_6248_right_retract_pid[1].reset();
            rear_6248.ctrl_Mit(1, SafeMotorAngle(rear_left_angle), 0.0f,
                               0.0f, 0.0f, 0.0f);
            rear_6248.ctrl_Mit(2, SafeMotorAngle(rear_right_angle), 0.0f,
                               0.0f, 0.0f, 0.0f);
        }
        // 后部未被禁用且两侧均可控，并处于收腿/收腿保持/基础角度状态时，按各自编码器目标运行位置环。
        else if (up_stair_behind_motor_fsm.Uses_Position_Control())
        {
            // 后部编码器位置控制：收腿与 IMU 失效基础角度模式共用现有串级 PID。
            rear_6248_pitch_pid[0].reset();
            rear_6248_pitch_pid[1].reset();

            const float left_target_velocity =
                rear_6248_left_retract_pid[0].UpDate(
                    up_stair_behind_motor_fsm.Get_Position_Target_Angle(1U),
                    up_stair_behind_motor_fsm.Get_Position_Feedback(1U));
            const float left_retract_torque =
                rear_6248_left_retract_pid[1].UpDate(
                    left_target_velocity, rear_left_velocity);

            const float right_target_velocity =
                rear_6248_right_retract_pid[0].UpDate(
                    up_stair_behind_motor_fsm.Get_Position_Target_Angle(2U),
                    up_stair_behind_motor_fsm.Get_Position_Feedback(2U));
            const float right_retract_torque =
                rear_6248_right_retract_pid[1].UpDate(
                    right_target_velocity, rear_right_velocity);

            rear_6248.ctrl_Mit(
                1, SafeMotorAngle(rear_left_angle), 0.0f, 0.0f, 0.0f,
                StairTorqueSafety::ClampJ6248Torque(
                    up_stair_behind_motor_fsm.Limit_Torque(
                        1U, left_retract_torque)));
            rear_6248.ctrl_Mit(
                2, SafeMotorAngle(rear_right_angle), 0.0f, 0.0f, 0.0f,
                StairTorqueSafety::ClampJ6248Torque(
                    up_stair_behind_motor_fsm.Limit_Torque(
                        2U, 0.0f)));
        }
        // 前两种分支均未命中，且后部 FSM 正处于恢复或姿态保持状态时，执行 Pitch 姿态串级控制。
        else if (up_stair_behind_motor_fsm.Uses_Attitude_Control())
        {
            rear_6248_left_retract_pid[0].reset();
            rear_6248_left_retract_pid[1].reset();
            rear_6248_right_retract_pid[0].reset();
            rear_6248_right_retract_pid[1].reset();

            const float pitch_target_rate = rear_6248_pitch_pid[0].UpDate(
                up_stair_behind_motor_fsm.Get_Target_Pitch_Deg(),
                up_stair_behind_motor_fsm.Get_Feedback_Pitch_Deg());
						
            const float pitch_torque = rear_6248_pitch_pid[1].UpDate(
                pitch_target_rate, 
						up_stair_behind_motor_fsm.Get_Pitch_Rate_Dps());
						
              left_pitch_torque =
                static_cast<float>(up_stair_behind_motor_fsm.Get_Motor_Direction(1)) *
                pitch_torque;
								
              right_pitch_torque =
                static_cast<float>(up_stair_behind_motor_fsm.Get_Motor_Direction(2)) *
                pitch_torque;
								
            // pitch 对左右同向；两侧各自按电机方向修正。后部 FSM 先执行反馈、机械角度、
            // 恢复比例和左右独立增益保护，再做 J6248 ±40 Nm 应用层最终硬限幅。
            rear_6248.ctrl_Mit(
                1, SafeMotorAngle(rear_left_angle), 0.0f, 0.0f, 0.0f,
                StairTorqueSafety::ClampJ6248Torque(
                    up_stair_behind_motor_fsm.Limit_Torque(1, left_pitch_torque)));
          rear_6248.ctrl_Mit(
              2, SafeMotorAngle(rear_right_angle), 0.0f, 0.0f, 0.0f,
              StairTorqueSafety::ClampJ6248Torque(
                  up_stair_behind_motor_fsm.Limit_Torque(2, 0.0f)));
        }

        else
        {
            // 后部 FSM 未请求位置控制或姿态控制时，清除两类 PID 并保持后腿零力矩。
            rear_6248_pitch_pid[0].reset();
            rear_6248_pitch_pid[1].reset();
            rear_6248_left_retract_pid[0].reset();
            rear_6248_left_retract_pid[1].reset();
            rear_6248_right_retract_pid[0].reset();
            rear_6248_right_retract_pid[1].reset();
            rear_6248.ctrl_Mit(1, SafeMotorAngle(rear_left_angle), 0.0f,
                               0.0f, 0.0f, 0.0f);
            rear_6248.ctrl_Mit(2, SafeMotorAngle(rear_right_angle), 0.0f,
                               0.0f, 0.0f, 0.0f);
        }

        }

        osDelay(1U);
    }
}
