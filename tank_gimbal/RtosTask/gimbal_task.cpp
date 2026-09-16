#include "gimbal_task.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "main.h"
#include "remote_control_task.hpp"
#include "../communication_between_boards/input_dispatcher.hpp"
#include "../communication_between_boards/refree_receive.hpp"
#include "../user/core/Alg/fire_control/single_shot_heat_limiter.hpp"
#include "../user/core/Alg/fire_control/friction_shot_detector.hpp"

const uint8_t chassis_motor_idxs[4] = {1, 2, 3, 4}; // 4 个 3508 电机的接收偏移 ID
BSP::Motor::Dji::GM3508<4> friction_motor(0x200, chassis_motor_idxs, 0x200); // 3508 电机控制器，初始ID为0x200，发送ID为0x200

using Remote = BSP::REMOTE_CONTROL::RemoteController;

DJI3508_State_t dji3508_state[4]; // 存储 4 个 3508 电机的状态数据

// 1 号电机是拨弹轮，2、3、4 号电机分别是左、右、下方摩擦轮
float feeder_target_angle = 0.0f; // 来自pid计算的目标角度
float feeder_current_angle = 0.0f; // 当前角度
float feeder_speed = 0.0f; // 当前速度
float feeder_iq = 0.0f; // 当前电流
float feeder_target_speed = 0.0f; // 来自pid计算的目标速度
float feeder_out = 0.0f; // 最终控制输出

float friction_current_speed_left = 0.0f; // 当前速度
float friction_current_speed_right = 0.0f;
float friction_current_speed_bottom = 0.0f; // 当前速度

float left_out = 0.0f; // 左侧摩擦轮控制输出
float right_out = 0.0f;
float bottom_out = 0.0f; // 下方摩擦轮控制输出

// feeder_mode, friction_mode, trigger_pressed 已迁移至 FSM 内部判定

void DJI3508_feedback();
static float Brake_Friction_To_Stop(float speed);

Class_Feeder_FSM feeder_fsm;
Class_Friction_FSM friction_fsm;
ALG::FireControl::SingleShotHeatLimiter single_shot_heat_limiter;
ALG::FireControl::FrictionShotDetector friction_shot_detector;

ALG::PID::PID feeder_angle_pid(3.6f, 0.0f, 1.3f, 10000.0f, 1000.0f, 100.0f);
ALG::PID::PID feeder_speed_pid(3.6f, 0.0f, 0.6f, 16384.0f, 1000.0f, 100.0f);
ALG::PID::PID feeder_angle_pid_speed(2.5f, 0.00f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID feeder_stop_pid(0.0f, 0.00f, 0.0f, 20000.0f, 1000.0f, 100.0f);


ALG::PID::PID feeder_speed_pid_speed(1.0f, 0.00f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID left_friction_pid(37.0f, 0.13f, 0.0f, 16384.0f, 1000.0f, 100.0f);
ALG::PID::PID right_friction_pid(37.0f, 0.13f, 0.0f, 16384.0f, 1000.0f, 100.0f);
ALG::PID::PID bottom_friction_pid(37.0f, 0.13f, 0.0f, 16384.0f, 1000.0f, 100.0f);

static constexpr float FRICTION_STOP_DEADBAND_RPM = 80.0f;
static constexpr float FRICTION_BRAKE_KP = 6.0f;
static constexpr float FRICTION_BRAKE_MAX_OUTPUT = 6000.0f;
static constexpr float FRICTION_REVERSE_BRAKE_LIMIT = 2500.0f;
// ROLLBACK_MARKER_FEEDER_STATIC_FF_BEGIN
static constexpr float FEEDER_STATIC_FF_OUTPUT = 3200.0f;
static constexpr float FEEDER_FF_MIN_ERROR_DEG = 275.0f;
static constexpr float FEEDER_FF_MAX_SPEED_RPM = 500.0f;
static constexpr float FEEDER_SINGLE_OUTPUT_LIMIT = 16384.0f;
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
    // 每个控制周期执行一次，周期由末尾的 vTaskDelay(5) 决定。
/************************************************************************************************ */

/************************************************************************************** */
uint8_t force_stop = 0;

 // 读取 4 个 3508 电机的角度、速度、电流和温度反馈。
 DJI3508_feedback();
/******************************************************************************* */
friction_fsm.TIM_Calculate_PeriodElapsedCallback();
feeder_fsm.TIM_Calculate_PeriodElapsedCallback();
/************************************************************************************ */
// FSM 内部自行判断模式：传入原始输入，FSM 根据 s1/s2/键鼠 自行决定
// 键鼠判定统一走 InputDispatcher 的输入源判定，避免两处判定逻辑分裂
bool is_keymouse = (input_dispatcher.GetSource() == InputSource::KeyMouse);

const uint32_t now_tick_ms = HAL_GetTick();
const auto &refree = Communication::GimbalRefree::instance();
single_shot_heat_limiter.UpdateJudgeData(
    0,//refree.isRefereeOnline(),
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
    // 遥控器失联：直接强制停止（不依赖拨杆组合）
    force_stop = 1;
}
else if (!is_keymouse)
{
    // 遥控器模式下特定挡位 = 强制停止
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
    // 键鼠模式下 R=OFF 时 = 停止
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

feeder_fsm.Update(feeder_input, feeder_current_angle, feeder_speed, feeder_iq);
leijia = feeder_fsm.Get_Accumulated_Angle();
/********************************************************************************** */

    // 跟踪控制类型变化，用于检测从其他模式切换到速度模式 (SPEED) 的时刻
    static uint8_t prev_control_type = FEEDER_CONTROL_STOP;
    uint8_t current_control_type = feeder_fsm.Get_Control_Type();

    // 根据 FSM 输出选择停止、角度级联控制或速度控制。
    if (force_stop)
{
    // 强制停止：复位PID，跳过FSM输出，直接刹车
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
    // 角度控制模式：位置外环计算目标转速，速度内环输出电机控制量。
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
    // 连发模式使用速度控制；切入时先同步目标角度，避免目标突变。
    // 刚切换到速度模式时，将目标角度同步为当前累积角度，防止从单发切换连发时
    // feeder_target_angle 还停留在旧值，导致大位置误差 → 先反转再正转
    if (prev_control_type != FEEDER_CONTROL_SPEED)
    {
        feeder_target_angle = feeder_fsm.Get_Accumulated_Angle();
    }

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
// 更新摩擦轮 FSM，得到左、右、下方三路摩擦轮目标转速。
friction_fsm.Update(friction_input,
                    friction_current_speed_left,
                    friction_current_speed_right,
                    friction_current_speed_bottom);

float left_friction_target = friction_fsm.Get_Left_Control_Output();
float right_friction_target = friction_fsm.Get_Right_Control_Output();
float bottom_friction_target = friction_fsm.Get_Bottom_Control_Output();

const bool friction_detection_enabled =
    !force_stop &&
    (left_friction_target != 0.0f ||
     right_friction_target != 0.0f ||
     bottom_friction_target != 0.0f);
friction_shot_detector.Update(dji3508_state[1].current_a,
                              friction_detection_enabled,
                              now_tick_ms);
const bool friction_shot_confirmed =
    friction_shot_detector.ConsumeShotConfirmed();
if (!single_shot_heat_limiter.IsRefereeOnline() && friction_shot_confirmed)
{
    single_shot_heat_limiter.NotifyShotFired(heat_limit_bypassed, now_tick_ms);
}

// 摩擦轮停止时复位速度 PID，并根据当前转速施加反向刹车。
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
    // 将底层电机反馈转换为任务层统一的状态结构。
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
    const float slots_per_rotation = 6.0f;
    const float angle_per_slot = 360.0f / slots_per_rotation; // 60 deg
    const float reduction_ratio = 51.0f;
    const float control_period = 0.005f;  // 与 vTaskDelay(5) 一致

    return fire_hz * angle_per_slot * reduction_ratio * control_period;
}

static float Brake_Friction_To_Stop(float speed)
{
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
