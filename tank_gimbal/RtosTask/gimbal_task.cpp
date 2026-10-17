#include "gimbal_task.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "main.h"
#include "remote_control_task.hpp"
#include "../communication_between_boards/input_dispatcher.hpp"
#include "../communication_between_boards/refree_receive_uart_485.hpp"
#include "../user/core/Alg/fire_control/single_shot_heat_limiter.hpp"
#include "../user/core/Alg/fire_control/friction_shot_detector.hpp"

// GM3508 逻辑槽 1~4：拨弹轮、左摩擦轮、右摩擦轮、下摩擦轮；接收反馈偏移为 1~4。
const uint8_t chassis_motor_idxs[4] = {1, 2, 3, 4};
// GM3508<4> 电机组：反馈 ID 基址 0x200，发送电流命令帧 ID 0x200。
BSP::Motor::Dji::GM3508<4> friction_motor(0x200, chassis_motor_idxs, 0x200);

using Remote = BSP::REMOTE_CONTROL::RemoteController;

DJI3508_State_t dji3508_state[4]; // 下标 0~3 对应拨弹轮、左摩擦轮、右摩擦轮、下摩擦轮

// 1 号电机是拨弹轮，2、3、4 号电机分别是左、右、下方摩擦轮。
float feeder_target_angle = 0.0f; // 拨弹位置环目标，电机轴多圈角度(deg)
float feeder_current_angle = 0.0f; // 拨弹轮当前多圈角度(deg)
float feeder_speed = 0.0f; // 拨弹电机实际转速(RPM)
float feeder_iq = 0.0f; // 拨弹电机电流反馈(A)；传入 FSM，但当前 FSM 不使用该量
float feeder_target_speed = 0.0f; // 拨弹位置环输出的目标转速(RPM)
float feeder_out = 0.0f; // 发给 DJI 3508 的电流控制原始命令值

float friction_current_speed_left = 0.0f; // 左摩擦轮实际转速(RPM)
float friction_current_speed_right = 0.0f;
float friction_current_speed_bottom = 0.0f; // 当前速度

float left_out = 0.0f; // 左摩擦轮电流控制输出，发送前转换为 int16_t 原始命令值
float right_out = 0.0f;
float bottom_out = 0.0f; // 下方摩擦轮控制输出

// feeder_mode, friction_mode, trigger_pressed 已迁移至 FSM 内部判定

void DJI3508_feedback();
static float Brake_Friction_To_Stop(float speed);

Class_Feeder_FSM feeder_fsm;
Class_Friction_FSM friction_fsm;
// 热量限制器使用默认参数；UpdateJudgeData 首参当前固定 false，因此采用本地离线估算路径。
// SingleShotHeatConfig 默认值：单发增加热量 100、热量上限 20000、离线冷却率 2000 heat/s、初始估算热量 0。
ALG::FireControl::SingleShotHeatLimiter single_shot_heat_limiter;
// 仅在摩擦轮运行期间检测电流边沿，用于无裁判热量数据时补记发射事件。
// 检测阈值来自 friction_shot_detector.hpp：左摩擦轮电流达到 8 A 触发，降到 5.5 A 重新武装，启用前 250 ms 忽略启动电流。
ALG::FireControl::FrictionShotDetector friction_shot_detector;

// 以下 PID 六参数依次为 Kp、Ki、Kd、输出幅值限幅、积分状态限幅、积分分离误差阈值。
// 拨弹位置环：角度误差(deg) -> 目标转速(RPM)；速度环：转速误差(RPM) -> 电流原始命令值。
ALG::PID::PID feeder_angle_pid(3.6f, 0.0f, 1.3f, 10000.0f, 1000.0f, 100.0f);
ALG::PID::PID feeder_speed_pid(3.6f, 0.0f, 0.6f, 16384.0f, 1000.0f, 100.0f);
// 连发位置外环：角度误差(deg)转换为转速目标(RPM)。
ALG::PID::PID feeder_angle_pid_speed(2.5f, 0.00f, 0.0f, 5000.0f, 1000.0f, 100.0f);
// 声明保留；当前强停和 STOP 分支没有调用此 PID。
ALG::PID::PID feeder_stop_pid(0.0f, 0.00f, 0.0f, 20000.0f, 1000.0f, 100.0f);


// 连发速度内环；三只摩擦轮使用相同增益，但各自 PID 状态独立。
ALG::PID::PID feeder_speed_pid_speed(1.0f, 0.00f, 0.0f, 5000.0f, 1000.0f, 100.0f);
// 摩擦轮 PID 的输入是速度误差(RPM)，输出是 DJI 电流原始命令；PID 末参数意义同上。
ALG::PID::PID left_friction_pid(37.0f, 0.13f, 0.0f, 16384.0f, 1000.0f, 100.0f);
ALG::PID::PID right_friction_pid(37.0f, 0.13f, 0.0f, 16384.0f, 1000.0f, 100.0f);
ALG::PID::PID bottom_friction_pid(37.0f, 0.13f, 0.0f, 16384.0f, 1000.0f, 100.0f);

// 关闭摩擦轮后，转速绝对值小于 80 RPM 时不再施加制动命令。
static constexpr float FRICTION_STOP_DEADBAND_RPM = 80.0f;
// 反向制动原始电流命令 = -转速(RPM) * 6，6 的单位是命令值/RPM；之后限幅至 +/-6000，不是 N*m。
static constexpr float FRICTION_BRAKE_KP = 6.0f;
static constexpr float FRICTION_BRAKE_MAX_OUTPUT = 6000.0f;
// 速度 PID 输出暂时与目标转速方向相反时，将该反向制动量限制在 +/-2500。
static constexpr float FRICTION_REVERSE_BRAKE_LIMIT = 2500.0f;
// ROLLBACK_MARKER_FEEDER_STATIC_FF_BEGIN
static constexpr float FEEDER_STATIC_FF_OUTPUT = 3200.0f; // 单发起步固定电流前馈的原始命令幅值
static constexpr float FEEDER_FF_MIN_ERROR_DEG = 275.0f; // 角度误差需严格大于 275 度才加前馈
static constexpr float FEEDER_FF_MAX_SPEED_RPM = 500.0f; // 当前转速绝对值需严格小于 500 RPM
static constexpr float FEEDER_SINGLE_OUTPUT_LIMIT = 16384.0f; // 前馈叠加后的拨弹电流命令总限幅
// ROLLBACK_MARKER_FEEDER_STATIC_FF_END

float leijia = 0;

extern "C" void gimbal_task(void *argument)
{
    // 拨弹轮和摩擦轮任务：读取反馈 -> 更新 FSM -> 计算 PID -> 发送 CAN 电流。
    feeder_fsm.Init();
    friction_fsm.Init();
    single_shot_heat_limiter.Init();


    for(;;)
    {
    // 每次循环读取反馈并更新两套 FSM；末尾延时 5 个 tick，tick=1 kHz 时约 5 ms。
/************************************************************************************************ */

/************************************************************************************** */
uint8_t force_stop = 0;

 // 读取 4 个 3508 电机的角度、速度、电流和温度反馈。
 DJI3508_feedback();
/******************************************************************************* */
friction_fsm.TIM_Calculate_PeriodElapsedCallback();
feeder_fsm.TIM_Calculate_PeriodElapsedCallback();
/************************************************************************************ */
// 先更新两个 FSM 的周期计时器，再用共享 InputDispatcher 的结果判断键鼠/遥控器来源。
bool is_keymouse = (input_dispatcher.GetSource() == InputSource::KeyMouse);

// 毫秒时间戳用于热量冷却估算和摩擦轮开火检测；FSM 状态计时另用周期计数。
const uint32_t now_tick_ms = HAL_GetTick();
const auto &refree = Communication::GimbalRefree485::instance();
// 首参数当前恒为离线；其余裁判热量/冷却数据用于记录和诊断，不改变该在线标志。
single_shot_heat_limiter.UpdateJudgeData(
    refree.isRefereeOnline(),// 固定传 false：限制器走裁判离线时的本地热量/冷却估算逻辑。
    refree.isDataReceived(),
     refree.getCoolingValue(),
    refree.getHeatLimit(),
    refree.getHeat42mm(),
    now_tick_ms);
const bool heat_limit_bypassed = input_dispatcher.IsHeatLimitBypassed();
const uint32_t referee_data_age_ms = refree.getDataAgeMs(now_tick_ms);

Struct_Feeder_Input feeder_input = {};
feeder_input.s1             = RemoteData.s1;
feeder_input.s2             = RemoteData.s2;
feeder_input.friction_on    = input_dispatcher.IsFrictionOn();
feeder_input.is_single_shot = input_dispatcher.IsSingleShot();
feeder_input.fire_triggered = input_dispatcher.IsFireTriggered();
feeder_input.scroll_value   = remoteController.get_scroll_();
feeder_input.is_keymouse    = is_keymouse;
// 热量限制器判定当前是否允许接受下一发；旁路只改变热量判断，不生成扳机事件。
feeder_input.single_shot_allowed =
    single_shot_heat_limiter.CanStartShot(heat_limit_bypassed);

Struct_Friction_Input friction_input = {};
friction_input.s1          = RemoteData.s1;
friction_input.s2          = RemoteData.s2;
friction_input.friction_on = input_dispatcher.IsFrictionOn();
friction_input.is_keymouse = is_keymouse;

force_stop = 0;
if (!remoteController.isConnected())
{
    // 遥控器失联：不信任拨杆和滚轮数据，最高优先级强制停止。
    force_stop = 1;
}
else if (!is_keymouse)
{
    // 遥控器模式下指定拨杆组合禁用拨弹和摩擦轮；其余允许组合交由 FSM 判定具体工作模式。
    if ((RemoteData.s1 == Remote::DOWN   && RemoteData.s2 == Remote::DOWN)   ||
        (RemoteData.s1 == Remote::DOWN   && RemoteData.s2 == Remote::MIDDLE) ||
        (RemoteData.s1 == Remote::MIDDLE && RemoteData.s2 == Remote::DOWN)   ||
        (RemoteData.s1 == Remote::MIDDLE && RemoteData.s2 == Remote::UP)     ||
        (RemoteData.s1 == Remote::MIDDLE && RemoteData.s2 == Remote::MIDDLE))
    {
        force_stop = 1;
    }
}
else
{
    // 键鼠模式下 R 关闭时外部强制停机；R 开启后，G 选择单发/连发。
    if (!input_dispatcher.IsFrictionOn())
    {
        force_stop = 1;
    }
}

// 触发检测和视觉开火已迁移至 feeder_fsm 内部，gimbal_task 不再处理
/************************************************************************************** */
    feeder_current_angle = dji3508_state[0].multi_angle;
    feeder_speed = dji3508_state[0].velocity_rpm;
    feeder_iq = dji3508_state[0].current_a;
    
    friction_current_speed_left = dji3508_state[1].velocity_rpm; // 左侧摩擦轮当前速度
    friction_current_speed_right = dji3508_state[2].velocity_rpm;
    friction_current_speed_bottom = dji3508_state[3].velocity_rpm; // 下方摩擦轮当前速度

// FSM 输入包含模式、滚轮/鼠标触发和热量许可；角度为电机多圈 deg，速度为 RPM，电流当前未参与 FSM 判定。
feeder_fsm.Update(feeder_input, feeder_current_angle, feeder_speed, feeder_iq);
leijia = feeder_fsm.Get_Accumulated_Angle();
/********************************************************************************** */

    // 用于检测本周期是否刚进入连发速度控制，以当前拨弹轮位置重新起算目标。
    static uint8_t prev_control_type = FEEDER_CONTROL_STOP;
    uint8_t current_control_type = feeder_fsm.Get_Control_Type();

    // 外部 force_stop 优先；否则按 FSM 控制类型选择停止、位置双环或连发速度控制。
    if (force_stop)
{
    // 强制停止：复位拨弹控制状态，并把拨弹输出清零；摩擦轮随后走独立的主动制动计算。
    feeder_angle_pid.reset();
    feeder_speed_pid.reset();
    feeder_angle_pid_speed.reset();
    feeder_speed_pid_speed.reset();
    feeder_out = 0.0f;
    left_out = 0.0f;
    right_out = 0.0f;
    bottom_out = 0.0f;
    bottom_friction_pid.reset();
}
else if (current_control_type == FEEDER_CONTROL_ANGLE)
{
    // 单发/退弹/冷却均由位置双环跟踪 FSM 目标；误差单位 deg，当前角度为多圈 deg。
    const float feeder_angle_error =
        feeder_fsm.Get_Control_Output() - feeder_fsm.Get_Accumulated_Angle();
    // 绝对值仅用于判断误差大小，实际方向仍由 feeder_angle_error 保留。
    const float feeder_abs_angle_error =
        (feeder_angle_error >= 0.0f) ? feeder_angle_error : -feeder_angle_error;
    // 转速绝对值用于判断当前是否处于低速阶段。
    const float feeder_abs_speed =
        (feeder_speed >= 0.0f) ? feeder_speed : -feeder_speed;
    const uint8_t feeder_status = feeder_fsm.Get_Now_Status_Serial();

    // 位置外环：目标角度与当前累积角度 -> 目标转速。
    feeder_target_speed = feeder_angle_pid.UpDate(
        feeder_fsm.Get_Control_Output(),
        feeder_fsm.Get_Accumulated_Angle()); // 位置环输入目标角度和当前累积角度，输出目标速度

    // 速度内环：目标转速与当前转速 -> 最终电机输出。
    feeder_out = feeder_speed_pid.UpDate(feeder_target_speed, feeder_speed);

    // ROLLBACK_MARKER_FEEDER_STATIC_FF_BEGIN
    // 单发起步时克服静摩擦和减速箱阻力，误差大且转速低时叠加固定前馈。
    // 接近目标或已经高速运行时不加前馈，减少过冲。
    if ((feeder_status == FEEDER_SINGLE_SHOT ||
         feeder_status == FEEDER_SINGLE_COOLDOWN) &&
        feeder_abs_angle_error > FEEDER_FF_MIN_ERROR_DEG &&
        feeder_abs_speed < FEEDER_FF_MAX_SPEED_RPM)
    {
        // 只有 PID 输出和目标转速同向时才叠加前馈，避免反向助推。
        if (feeder_target_speed > 0.0f && feeder_out > 0.0f)
        {
            feeder_out += FEEDER_STATIC_FF_OUTPUT;
        }
        else if (feeder_target_speed < 0.0f && feeder_out < 0.0f)
        {
            feeder_out -= FEEDER_STATIC_FF_OUTPUT;
        }

        // 前馈叠加后再次限幅，保护电机控制输出不超限。
        if (feeder_out > FEEDER_SINGLE_OUTPUT_LIMIT)
        {
            feeder_out = FEEDER_SINGLE_OUTPUT_LIMIT;
        }
        else if (feeder_out < -FEEDER_SINGLE_OUTPUT_LIMIT)
        {
            feeder_out = -FEEDER_SINGLE_OUTPUT_LIMIT;
        }
    }
    // ROLLBACK_MARKER_FEEDER_STATIC_FF_END
}

else if (current_control_type == FEEDER_CONTROL_SPEED)
{
    // 连发采用速度目标与位置外环组合，按频率换算出每周期拨弹角增量。
    // 刚切换到速度模式时，将目标角度同步为当前累积角度，防止从单发切换连发时
    // feeder_target_angle 还停留在旧值，导致大位置误差 → 先反转再正转
    if (prev_control_type != FEEDER_CONTROL_SPEED)
    {
        feeder_target_angle = feeder_fsm.Get_Accumulated_Angle();
    }

    // 3 Hz 是此处当前写死的推进频率；负号按拨弹轮电机安装方向逐周期推进位置目标。
    feeder_target_angle -= hz_to_rotor_angle_per_frame(3.0f);

    feeder_target_speed = feeder_angle_pid_speed.UpDate(
        feeder_target_angle,
        feeder_fsm.Get_Accumulated_Angle()
    );

    feeder_out = feeder_speed_pid_speed.UpDate(feeder_target_speed, feeder_speed);
}
else
{
    feeder_angle_pid.reset();
    feeder_speed_pid.reset();
    feeder_angle_pid_speed.reset();
    feeder_speed_pid_speed.reset();
    feeder_out = 0.0f;
}

    prev_control_type = current_control_type;

/********************************************************************************** */
// FSM 返回三路摩擦轮的速度目标(RPM)：STOP 为 0；STARTING 和 READY 使用相同目标转速。
// friction_fsm.hpp 当前 TARGET_SPEED=-4150，输出取其相反数即 +4150 RPM。
// READY 需三路实测速度均 >=4150 RPM 且 STARTING 达 200 tick；速度跌破阈值会回 STARTING。
friction_fsm.Update(friction_input,
                    friction_current_speed_left,
                    friction_current_speed_right,
                    friction_current_speed_bottom);

float left_friction_target = friction_fsm.Get_Left_Control_Output();
float right_friction_target = friction_fsm.Get_Right_Control_Output();
float bottom_friction_target = friction_fsm.Get_Bottom_Control_Output();

// 电流开火检测只在外部未强停且至少一个摩擦轮目标非零时运行。
const bool friction_detection_enabled =
    !force_stop &&
    (left_friction_target != 0.0f ||
     right_friction_target != 0.0f ||
     bottom_friction_target != 0.0f);
// 无裁判数据时用左摩擦轮电流(A)的开火事件补记本地热量；消费标志后只处理一次。
friction_shot_detector.Update(dji3508_state[1].current_a,
                              friction_detection_enabled,
                              now_tick_ms);
const bool friction_shot_confirmed =
    friction_shot_detector.ConsumeShotConfirmed();
if (!single_shot_heat_limiter.IsRefereeOnline() && friction_shot_confirmed)
{
    single_shot_heat_limiter.NotifyShotFired(heat_limit_bypassed, now_tick_ms);
}

// 任一外部强停或三路目标全为零时，复位速度 PID，并按各轮实际 RPM 单独施加反向刹车。
if (force_stop ||
    (left_friction_target == 0.0f &&
     right_friction_target == 0.0f &&
     bottom_friction_target == 0.0f))
{
    left_friction_pid.reset();
    right_friction_pid.reset();
    bottom_friction_pid.reset();
    left_out = Brake_Friction_To_Stop(friction_current_speed_left);
    right_out = Brake_Friction_To_Stop(friction_current_speed_right);
    bottom_out = Brake_Friction_To_Stop(friction_current_speed_bottom);
}
else
{
    left_out = left_friction_pid.UpDate(
        left_friction_target,
        friction_current_speed_left);

    right_out = right_friction_pid.UpDate(
        right_friction_target,
        friction_current_speed_right);

    bottom_out = bottom_friction_pid.UpDate(
        bottom_friction_target,
        friction_current_speed_bottom);

    if ((left_friction_target < 0.0f && left_out > 0.0f) ||
        (left_friction_target > 0.0f && left_out < 0.0f))
    {
        // PID 输出反向于目标，视为暂时制动；只限幅反向分量，目标同向驱动不受此值约束。
        if (left_out > FRICTION_REVERSE_BRAKE_LIMIT)
        {
            left_out = FRICTION_REVERSE_BRAKE_LIMIT;
        }
        else if (left_out < -FRICTION_REVERSE_BRAKE_LIMIT)
        {
            left_out = -FRICTION_REVERSE_BRAKE_LIMIT;
        }
    }

    if ((right_friction_target < 0.0f && right_out > 0.0f) ||
        (right_friction_target > 0.0f && right_out < 0.0f))
    {
        if (right_out > FRICTION_REVERSE_BRAKE_LIMIT)
        {
            right_out = FRICTION_REVERSE_BRAKE_LIMIT;
        }
        else if (right_out < -FRICTION_REVERSE_BRAKE_LIMIT)
        {
            right_out = -FRICTION_REVERSE_BRAKE_LIMIT;
        }
    }

    if ((bottom_friction_target < 0.0f && bottom_out > 0.0f) ||
        (bottom_friction_target > 0.0f && bottom_out < 0.0f))
    {
        if (bottom_out > FRICTION_REVERSE_BRAKE_LIMIT)
        {
            bottom_out = FRICTION_REVERSE_BRAKE_LIMIT;
        }
        else if (bottom_out < -FRICTION_REVERSE_BRAKE_LIMIT)
        {
            bottom_out = -FRICTION_REVERSE_BRAKE_LIMIT;
        }
    }
}
/********************************************************************************** */
// 按电机逻辑槽写入拨弹轮和三只摩擦轮的电流命令原始值，合并为一次 DJI 电流帧。
friction_motor.setCAN((int16_t)feeder_out, 1);
friction_motor.setCAN((int16_t)left_out, 2);
friction_motor.setCAN((int16_t)right_out, 3);
friction_motor.setCAN((int16_t)bottom_out, 4);

// 一次性发送拨弹轮和三路摩擦轮的最终电流指令。
friction_motor.sendCAN();
/**************************************************************************** */
// VOFA JustFloat 6 路: ch1 左摩擦轮实际转速(RPM), ch2 右摩擦轮实际转速(RPM),
//                      ch3 下摩擦轮实际转速(RPM), ch4~ch6 预留(恒 0)
// vofa_send(friction_current_speed_left,
//           friction_current_speed_right,
//           friction_current_speed_bottom,
//           0.0f,
//           0.0f,
//           0.0f);
/****************************************************************************** */
vTaskDelay(5); // 每5ms执行一次控制循环
    }
}


void DJI3508_feedback() {
    // 把驱动层反馈快照复制到任务状态；拨弹 FSM 使用多圈轴角(deg)和 RPM，摩擦 FSM 使用 RPM。
    for (int i = 0; i < 4; i++) {
        uint8_t motor_id = i + 1; // 电机逻辑 ID 通常从 1 开始
         dji3508_state[i].multi_angle = friction_motor.getAddAngleDeg(motor_id) ; // 获取多圈角度，单位为度
        dji3508_state[i].angle_deg   = friction_motor.getAngleDeg(motor_id); // 如果拨弹轮也是19:1减速比，则同样除以19
        dji3508_state[i].angle_rad   = friction_motor.getAngleRad(motor_id);
if (motor_id == 1) {
            dji3508_state[i].velocity_rpm = friction_motor.getVelocityRpm(motor_id) ;
        } else {
            dji3508_state[i].velocity_rpm = friction_motor.getVelocityRpm(motor_id); // 摩擦轮若还是19:1则保留
        }        dji3508_state[i].velocity_rads  = friction_motor.getVelocityRads(motor_id);   //角速度，用这个控制电机
        dji3508_state[i].current_a     = friction_motor.getCurrent(motor_id);
        dji3508_state[i].temperature        = friction_motor.getTemperature(motor_id);
    }
}

float hz_to_rotor_angle_per_frame(float fire_hz)
{
    // 六弹位拨弹轮每槽 60 度，减速比 51；每周期角增量 = Hz * 60 * 51 * 0.005 度。
    const float slots_per_rotation = 6.0f;
    const float angle_per_slot = 360.0f / slots_per_rotation; // 60 deg
    const float reduction_ratio = 51.0f;
    const float control_period = 0.005f;  // 当前按 5 ms 计算，与 1 kHz 系统 tick 下的 vTaskDelay(5) 对应

    return fire_hz * angle_per_slot * reduction_ratio * control_period;
}

// 输入摩擦轮转速(RPM)，输出 DJI 电流命令原始值；低于死区为零，其余按反向比例制动并限幅。
static float Brake_Friction_To_Stop(float speed)
{
    // 80 RPM 死区之外按 -6*RPM 计算电流命令，再限制到 +/-6000。
    if (speed > -FRICTION_STOP_DEADBAND_RPM && speed < FRICTION_STOP_DEADBAND_RPM)
    {
        return 0.0f;
    }

    float output = -FRICTION_BRAKE_KP * speed;

    if (output > FRICTION_BRAKE_MAX_OUTPUT)
    {
        output = FRICTION_BRAKE_MAX_OUTPUT;
    }
    else if (output < -FRICTION_BRAKE_MAX_OUTPUT)
    {
        output = -FRICTION_BRAKE_MAX_OUTPUT;
    }

    return output;
}
