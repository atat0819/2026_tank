#include "gantry_task.hpp"
#include "watchdog_heartbeat.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "remote_task.hpp"

#include "../fsm/gantry_fsm.hpp"
#include "../fsm/motor_recovery_fsm.hpp"
#include "../user/core/BSP/switch/switch.hpp"
#include "../user/core/Alg/PID/pid.hpp"
#include "can_send_task.hpp"
#include "up_stair.hpp"
#include <algorithm>
#include <cmath>

// FDCAN3：3508 物理 ID 5、6，反馈 ID 0x205、0x206。
// 0x1FF 命令帧内槽位 1、2 对应物理 ID 5、6；setCAN() 传槽位 1、2。
// 最后一个参数 19.2 为电机减速比（转子转数/输出轴转数），须与实物匹配；累计角度/速度已换算为输出轴单位。
const uint8_t gantry_3508_motor_idxs[2] = {5, 6};
BSP::Motor::Dji::GM3508<2> gantry_3508_motor(
    0x200, gantry_3508_motor_idxs, 0x1FF, HAL::FDCAN::FdcanDeviceId::HAL_Fdcan3, 19.2);

// FDCAN3：DM4310 控制/反馈 ID 均为 9；对象内部仅有逻辑通道 1。
BSP::Motor::DM::J4310<1> gantry_dm4310_motor(
    0x00, {9}, {9}, HAL::FDCAN::FdcanDeviceId::HAL_Fdcan3);

volatile GantryWatch gantry_watch{};

namespace
{
Class_Gantry_FSM gantry_fsm;
// 硬件配置：依次为第一次抬升、前伸、第二次抬升、后收、下降到底开关；仅在接线改变时修改端口。
// MicroSwitch 默认低电平按下，按下/松开均需稳定 20 ms；构造函数可用第三参数单独指定消抖 ms。
BSP::Switch::MicroSwitch limits[5] = {
    {GPIOE, GPIO_PIN_13}, {GPIOE, GPIO_PIN_9}, {GPIOA, GPIO_PIN_2},
    {GPIOA, GPIO_PIN_0}, {GPIOD, GPIO_PIN_10}
};
// PID 六参数顺序：Kp、Ki、Kd、输出限幅、积分状态限幅、积分分离阈值；各轴分别整定。
// Kp：当前误差的比例增益；增大增强修正，过大可能振荡。
// Ki：误差累加量的增益；消除持续偏差，过大易积累过冲；Ki=0 才是关闭积分输出。
// Kd：本次与上次误差差值的增益；抑制快速变化，也会放大反馈噪声；初期可置 0。
// 输出限幅：PID 输出的正负绝对值上限；速度环单位为 C620 原始电流指令，角度环单位为 rad/s。
// 积分状态限幅：限制 sum(error)，不是限制 Ki*sum(error)；填正值启用，0 表示不限制积分状态。
// 积分分离阈值：正值时仅 |error| 小于该值才累加，超出时保留已有积分；0 表示始终允许累加。
// 本 PID 不乘/除 dt：I=sum(error)，D=error-last_error；循环末尾 osDelay(2)，实际周期还包含计算/发送时间。
// 改任务周期后须重新核对 Ki/Kd；现有全部 Kp/Ki/Kd 为 0，尚无闭环调节能力，不是已整定参数。
// 速度环误差为输出轴 rad/s，输出为原始电流指令；标称 409.6 对应 0.5 A，16384 对应 20 A。
// 积分状态按每次 rad/s 误差累加；分离阈值为 rad/s。正目标/正反馈均沿电机自身正转方向，不因反装把增益取负。
// 速度环输出与重力补偿相加后还会统一限幅 ±16384；这里的 PID 限幅并不单独限制最终含补偿电流。
ALG::PID::PID lift_move_pid[2] = {
    {0.0f /* Kp */, 0.0f /* Ki */, 0 /* Kd */, 16384.0f /* 电流输出限幅 */, 0.0f /* 积分状态限幅 */, 0 /* 分离阈值 */}, // ID5：第一次/第二次抬升、下降
    {0.0f /* Kp */, 0.0f /* Ki */, 0 /* Kd */, 16384.0f /* 电流输出限幅 */, 0.0f /* 积分状态限幅 */, 0 /* 分离阈值 */}  // ID6：同上，独立参数
};

// 位置保持角度外环：误差 rad -> 目标速度 rad/s；六参数顺序同上。
// 输出限幅直接使用各轴速度上限，让 PID 在限幅时处理积分。
// Kp 单位 1/s；Ki/Kd 使用上面的离散累加/差分定义；积分状态按每次 rad 误差累加，分离阈值单位 rad。
// 第五项 100/19 或 10 是积分状态上限，不是最大保持角度；当前 Ki=0，因此该积分状态不产生输出。
// 原有 100/19 积分上限是历史调参值，并非从电机实例自动换算，需按当前输出轴反馈实测整定。
// 保持目标取状态切换时实际累计角度：ID5/6 用于前伸/后收保持，ID9 用于升降及完成后的保持。
ALG::PID::PID hold_angle_pid[3] = {
    {0.0f /* Kp */, 0.0f /* Ki */, 0 /* Kd */, 10.0f /* 速度输出限幅 */, 100.0f / 19.0f /* 积分状态限幅 */, 0 /* 分离阈值 */}, // ID5
    {0.0f /* Kp */, 0.0f /* Ki */, 0 /* Kd */, 10.0f /* 速度输出限幅 */, 100.0f / 19.0f /* 积分状态限幅 */, 0 /* 分离阈值 */}, // ID6
    {0.0f /* Kp */, 0.0f /* Ki */, 0 /* Kd */, 10.0f /* 速度输出限幅 */, 10 /* 积分状态限幅 */, 0 /* 分离阈值 */} // DM ID9
};

// 位置保持速度内环：单位/六参数含义与 lift_move_pid 相同；只用于 ID5/6 前伸、后收时保持高度。
// 先调稳速度内环，再整定保持角度外环；运动与保持使用不同实例，切换时未使用的 PID 会 reset。
ALG::PID::PID lift_hold_pid[2] = {
    {0.0f /* Kp */, 0.0f /* Ki */, 0 /* Kd */, 16384.0f /* 电流输出限幅 */, 0.0f /* 积分状态限幅 */, 0 /* 分离阈值 */}, // ID5 保持速度环
    {0.0f /* Kp */, 0.0f /* Ki */, 0 /* Kd */, 16384.0f /* 电流输出限幅 */, 0.0f /* 积分状态限幅 */, 0 /* 分离阈值 */}  // ID6 保持速度环
};

bool Gantry_ConfigValid()
{
    using namespace GantryTuning;
    const float gravity_cmd[] = {LIFT_ID5_GRAVITY_EMPTY_CMD, LIFT_ID6_GRAVITY_EMPTY_CMD,
                                 LIFT_ID5_GRAVITY_LOADED_CMD, LIFT_ID6_GRAVITY_LOADED_CMD};
    for (float value : gravity_cmd)
        if (!std::isfinite(value) || std::abs(value) > 16384.0f)
            return false;
    if (!Class_Gantry_FSM::ConfigValid())
        return false;
    if (!std::isfinite(LOWER_UNLOAD_PROGRESS) || LOWER_UNLOAD_PROGRESS <= 0 ||
        LOWER_UNLOAD_PROGRESS >= 1 || LOWER_UNLOAD_BLEND_MS == 0)
        return false;
    if (DM_EXTEND_SPEED_RAD_S > 30.0f || DM_RETRACT_SPEED_RAD_S > 30.0f ||
        !std::isfinite(DM_MOVE_KD) || DM_MOVE_KD <= 0 || DM_MOVE_KD > 5 ||
        !std::isfinite(DM_HOLD_KD) || DM_HOLD_KD <= 0 || DM_HOLD_KD > 5 ||
        !std::isfinite(DM_TORQUE_LIMIT_NM) || DM_TORQUE_LIMIT_NM <= 0 ||
        DM_TORQUE_LIMIT_NM > 3)
        return false;
    return true;
}
}

extern "C" void gantry_task(void *argument)
{
    (void)argument;
    using namespace GantryTuning;
    // 初始化：校验标定和控制参数。
    const bool configured = Gantry_ConfigValid();
    gantry_watch.configured = configured;
    while (!dm_motor_control_ready) { Watchdog_RecordHeartbeat(WATCHDOG_TASK_GANTRY); osDelay(2); }
    MotorRecoveryFSM dm_recovery_fsm;
    for(;;)
    {
        // 读取限位、遥控输入和三台电机反馈快照。
        float velocity[3];
        float lift_current_a[2];
        float angle_deg[3];
        bool limit_pressed[5];
        for (unsigned i = 0; i < 5; ++i)
        {
            limits[i].update();
            gantry_watch.limit_raw[i] = limits[i].getRawLevel() == GPIO_PIN_SET;
            limit_pressed[i] = limits[i].getPress();
            gantry_watch.limit[i] = limit_pressed[i];
        }
        // 遥控在线查询可能触发蜂鸣器请求，放在临界区外。
        const ControlInputSnapshot input = GetControlInputSnapshot();
        taskENTER_CRITICAL();
        const uint32_t now_ms = HAL_GetTick();
        const bool dm_control_allowed = !EvaluateStairModePolicy(
            input.s1, input.s2, input.source != ControlInputSource::NONE && !g_watchdog_rearm_required,
            input.keyboard_online).zero_all_torque;
        const bool control_enabled = configured && input.source == ControlInputSource::LOCAL &&
            input.keyboard_online && input.s1 == 3U && input.s2 == 3U && !g_watchdog_rearm_required;
        const bool start_key_pressed = (input.keyboard & BSP::REMOTE_CONTROL::RemoteController::KEY_Z) != 0;
        bool feedback_ok = true;
        bool lift_feedback_ok = true;
        for (unsigned i = 0; i < 2; ++i)
        {
            angle_deg[i] = gantry_3508_motor.getAddAngleDeg(i + 1);
            velocity[i] = gantry_3508_motor.getVelocityRads(i + 1);
            // Parse() 在驱动内更新时间戳，isConnected() 判断反馈超时。
            const bool online = gantry_3508_motor.isConnected(i + 1, i + 5);
            gantry_watch.online[i] = online;
            gantry_watch.angle_deg[i] = angle_deg[i];
            gantry_watch.velocity_rad_s[i] = velocity[i];
            lift_current_a[i] = gantry_3508_motor.getCurrent(i + 1);
            gantry_watch.lift_current_a[i] = lift_current_a[i];
            lift_feedback_ok = lift_feedback_ok && online;
            feedback_ok = feedback_ok && online && std::isfinite(velocity[i]);
        }
        angle_deg[2] = gantry_dm4310_motor.getAddAngleDeg(1);
        velocity[2] = gantry_dm4310_motor.getVelocityRads(1);
        const uint8_t dm_status = gantry_dm4310_motor.getDriveStatus(1);
        const bool dm_online = dm_status != BSP::Motor::DM::UNKNOWN_DRIVE_STATUS;
        gantry_watch.online[2] = dm_online;
        gantry_watch.angle_deg[2] = angle_deg[2];
        gantry_watch.velocity_rad_s[2] = velocity[2];
        const bool dm_ready = dm_status == 1U;
        feedback_ok = feedback_ok && dm_ready && std::isfinite(velocity[2]);
        gantry_watch.control_enabled = control_enabled;
        gantry_watch.feedback_ok = feedback_ok;
        gantry_watch.start_key_pressed = start_key_pressed;
        taskEXIT_CRITICAL();

        // 更新动作状态机及调试状态。
        gantry_fsm.Update(
            control_enabled,
            feedback_ok,
            lift_feedback_ok, // 启动原点只依赖两台 3508，不等待 DM 使能
            start_key_pressed,
            start_key_pressed,
            limit_pressed[0], // PE13：第一次抬升
            limit_pressed[1], // PE9：前伸
            limit_pressed[2], // PA2：第二次抬升
            limit_pressed[3], // PA0：后收
            limit_pressed[4], // PD10：下降到底
            angle_deg[0],     // 3508 ID5, deg
            angle_deg[1],     // 3508 ID6, deg
            angle_deg[2],     // DM ID9, deg
            lift_current_a[0], // 3508 ID5 实际转矩电流，A
            lift_current_a[1], // 3508 ID6 实际转矩电流，A
            velocity[0],      // 3508 ID5 输出轴速度，rad/s
            velocity[1],      // 3508 ID6 输出轴速度，rad/s
            now_ms);
        gantry_watch.state = gantry_fsm.GetState();
        gantry_watch.fault = gantry_fsm.GetFault();
        gantry_watch.progress = gantry_fsm.GetProgress();
        gantry_watch.scale = gantry_fsm.GetScale();
        gantry_watch.lift_sync_error_deg = gantry_fsm.GetLiftSyncErrorDeg();

        // 计算保持速度、升降 PID 和 DM 阻尼力矩，发送 CAN。
        for (unsigned i = 0; i < 2; ++i)
        {
            float current = 0;
            gantry_watch.target_velocity_rad_s[i] = 0;
            if (!gantry_fsm.IsMoving(i) && !gantry_fsm.IsKeeping(i)) // 停机或到底后两台同时零电流
            {
                lift_move_pid[i].reset();
                lift_hold_pid[i].reset();
                hold_angle_pid[i].reset();
            }
            else if (gantry_fsm.IsMoving(i))  // 3508 升降电机在运动状态下使用升降 PID，保持 PID 清零
            {
                const float target = gantry_fsm.GetTargetSpeed(i);
                gantry_watch.target_velocity_rad_s[i] = target;
                lift_hold_pid[i].reset();
                hold_angle_pid[i].reset();
                current = lift_move_pid[i].UpDate(target, velocity[i]);
            }
            else if (gantry_fsm.IsKeeping(i)) // 3508 位置保持双环，升降 PID 清零
            {
                // 累计角度和标定保留 deg，保持外环的角度误差转换为 rad。
                const float hold_error_rad =
                    (gantry_fsm.GetHoldAngle(i) - angle_deg[i]) * (3.14159265358979323846f / 180.0f);
                const float target = hold_angle_pid[i].UpDate(hold_error_rad, 0);
                gantry_watch.target_velocity_rad_s[i] = target;
                lift_move_pid[i].reset();
                current = lift_hold_pid[i].UpDate(target, velocity[i]);
            }
            if (gantry_fsm.IsMoving(i) || gantry_fsm.IsKeeping(i))
            {
                const float empty_cmd[2] = {LIFT_ID5_GRAVITY_EMPTY_CMD, LIFT_ID6_GRAVITY_EMPTY_CMD};
                const float loaded_cmd[2] = {LIFT_ID5_GRAVITY_LOADED_CMD, LIFT_ID6_GRAVITY_LOADED_CMD};
                const GantryState state = gantry_fsm.GetState();
                float gravity_cmd = 0;
                if (state == GantryState::LiftFirst || state == GantryState::Extend)
                {
                    gravity_cmd = empty_cmd[i]; // 第一次抬升和前伸：空载
                }
                else if (state == GantryState::LiftSecond || state == GantryState::Retract)
                {
                    gravity_cmd = loaded_cmd[i]; // 第二次抬升和后收：带载
                }
                else if (state == GantryState::Lower)
                {
                    // 下降卸载后，系数在 200 ms 内由 1 逐渐变为 0。
                    const float loaded_factor = gantry_fsm.GetGravityLoadFactor();
                    gravity_cmd = empty_cmd[i] + (loaded_cmd[i] - empty_cmd[i]) * loaded_factor;
                }
                current += gravity_cmd;
                current = std::clamp(current, -16384.0f, 16384.0f);
            }
            const int16_t command = static_cast<int16_t>(current);
            gantry_watch.lift_command[i] = command;
            gantry_3508_motor.setCAN(command, i + 1);
        }
        gantry_3508_motor.sendCAN();
        gantry_watch.target_velocity_rad_s[2] = 0;
        gantry_watch.dm_torque_command_nm = 0;
        if (dm_control_allowed && dm_ready && gantry_fsm.IsActive())
        {
            float target = 0;
            float kd = 0;
            if (gantry_fsm.IsMoving(2)) // DM 前伸、后收
            {
                hold_angle_pid[2].reset();
                target = gantry_fsm.GetTargetSpeed(2);
                kd = DM_MOVE_KD;
            }
            else if (gantry_fsm.IsKeeping(2)) // DM 位置保持
            {
                const float hold_error_rad =
                    (gantry_fsm.GetHoldAngle(2) - angle_deg[2]) * (3.14159265358979323846f / 180.0f);
                target = hold_angle_pid[2].UpDate(hold_error_rad, 0);
                kd = DM_HOLD_KD;
            }
            // MIT 的 Kp=0，使用速度阻尼；累计角度保持在外部计算，避免多圈位置折返。
            // 先限制 Kd*(目标速度-实际速度) 的总力矩，再换回发送的速度。
            const float torque = std::clamp(kd * (target - velocity[2]),
                                           -DM_TORQUE_LIMIT_NM, DM_TORQUE_LIMIT_NM);
            gantry_watch.target_velocity_rad_s[2] = target;
            gantry_watch.dm_torque_command_nm = torque;
            gantry_dm4310_motor.ctrl_Mit(1, 0, velocity[2] + torque / kd, 0, kd, 0);
        }
        else  // 非活动状态下清零输出并复位 PID
        {
            hold_angle_pid[2].reset();
            if (dm_control_allowed)
                gantry_dm4310_motor.ctrl_Mit(1, 0, 0, 0, 0, 0);
        }
/******************************************************************************** */
        // 驱动启停只看遥控安全条件和反馈状态；机构动作由 gantry_fsm 决定。
        const auto dm_request = dm_recovery_fsm.Update(dm_control_allowed, dm_status, now_ms);
        if (dm_request == MotorRecoveryFSM::Request::Off)
            gantry_dm4310_motor.Off(1, BSP::Motor::DM::MIT);
        else if (dm_request == MotorRecoveryFSM::Request::On)
            gantry_dm4310_motor.On(1, BSP::Motor::DM::MIT);
/************************************************************************* */
//发送vofa数据
const float vofa_data[] = {
    // 三台电机累计角度
    gantry_watch.angle_deg[0],                    // CH0：ID5 输出轴角度，deg
    gantry_watch.angle_deg[1],                    // CH1：ID6 输出轴角度，deg
    gantry_watch.angle_deg[2],                    // CH2：DM 角度，deg

    // 目标速度与实际速度
    gantry_watch.target_velocity_rad_s[0],        // CH3：ID5 目标速度，rad/s
    gantry_watch.velocity_rad_s[0],               // CH4：ID5 实际速度，rad/s
    gantry_watch.target_velocity_rad_s[1],        // CH5：ID6 目标速度，rad/s
    gantry_watch.velocity_rad_s[1],               // CH6：ID6 实际速度，rad/s
    gantry_watch.target_velocity_rad_s[2],        // CH7：DM 目标速度，rad/s
    gantry_watch.velocity_rad_s[2],               // CH8：DM 实际速度，rad/s

    // 升降电机输出
    static_cast<float>(gantry_watch.lift_command[0]), // CH9：ID5 电流指令
    static_cast<float>(gantry_watch.lift_command[1]), // CH10：ID6 电流指令
    gantry_watch.lift_current_a[0],              // CH11：ID5 实测电流，A
    gantry_watch.lift_current_a[1],              // CH12：ID6 实测电流，A

    gantry_watch.lift_sync_error_deg,            // CH13：同步角差，deg
    static_cast<float>(gantry_watch.state),      // CH14：状态机阶段
    static_cast<float>(gantry_watch.fault)       // CH15：故障编号
};

//vofa_sendN(vofa_data, 15);
/**************************************************************************** */

        Watchdog_RecordHeartbeat(WATCHDOG_TASK_GANTRY);
        osDelay(2); // 每轮延时 2 个系统 tick；当前 1 ms/tick 时为 2 ms，实际周期还含执行耗时；修改后重核 PID Ki/Kd
    }
}
