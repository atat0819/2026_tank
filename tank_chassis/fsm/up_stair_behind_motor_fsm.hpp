#ifndef UP_STAIR_BEHIND_MOTOR_FSM_HPP
#define UP_STAIR_BEHIND_MOTOR_FSM_HPP

#include "../user/core/Alg/FSM/alg_fsm.hpp"
#include "../user/core/Alg/UtilityFunction/SlopePlanning.hpp"
#include <stdint.h>

enum Enum_Up_Stair_Behind_Motor_Status
{
    UP_STAIR_BEHIND_MOTOR_DISABLED = 0,       // 后连杆控制关闭，力矩为零
    UP_STAIR_BEHIND_MOTOR_RECOVERING,         // 反馈恢复后，力矩比例软启动
    UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD,      // 正常进行 pitch 水平控制
    UP_STAIR_BEHIND_MOTOR_RETRACTING,          // 收腿位置控制正在按规划速度移动到目标角
    UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD,      // 收腿目标已到位，继续保持该位置
    UP_STAIR_BEHIND_MOTOR_COUNT                // 状态数量，用于 FSM 初始化和边界检查
};

class Class_Up_Stair_Behind_Motor_FSM : public Class_FSM
{
public:
    // 后腿生产角度配置：数组下标 0=左腿、1=右腿；实车标定后在此修改。
    // Config() 仍保留未标定的安全默认值，任务必须显式装入这些角度才能启用控制。
    static constexpr float PITCH_ZERO_DEG = 1.3f; // 实测水平时 HI14 原始 Pitch
    static constexpr float PITCH_TARGET_DEG = -0.3f; // 原始目标 -7° 减去水平零偏 1.3°；实测左71°、右26°
    // 实车机械区间：左 29°→94.7°，右 4.47°→68°，均不跨 ±π。
    static constexpr float ANGLE_START_RAD[2] = { // 左右电机机械安全区起始角，弧度
        29.29f * 3.14159265358979323846f / 180.0f,
        4.47f * 3.14159265358979323846f / 180.0f};
    static constexpr float ANGLE_END_RAD[2] = { // 左右电机机械安全区终止角，弧度
        94.04f * 3.14159265358979323846f / 180.0f,
        68.84f * 3.14159265358979323846f / 180.0f};
    // 端点外侧反馈容差及向外力矩拦截容差。
    static constexpr float ANGLE_LIMIT_TOLERANCE_RAD =
        0.5f * 3.14159265358979323846f / 180.0f;
    static constexpr int8_t MOTOR_DIRECTION[2] = {1, -1}; // 左右电机镜像安装，Pitch 力矩方向相反
    static constexpr float MIT_SPEED_LIMIT_RAD_S = 20.0f; // J6248 协议速度量程
    static constexpr float MOTOR_TORQUE_GAIN[2] = {1.0f, 1.0f}; // 左右电机最终力矩倍率，必须大于 0
    // B 收腿目标保留端点内侧 8° 的位置；该间距不参与机械限位保护。
    static constexpr float RETRACT_MARGIN_RAD = 3.0f * 3.14159265358979323846f / 180.0f;
    static constexpr float RETRACT_TARGET_RAD[2] = {
        ANGLE_END_RAD[0] - RETRACT_MARGIN_RAD,
        ANGLE_START_RAD[1] + RETRACT_MARGIN_RAD};
    static constexpr float RETRACT_SPEED_RAD_S = 1.8f; // 收腿规划速度，弧度/秒；未标定时为 0，禁止收腿
    static constexpr float RETRACT_POSITION_TOLERANCE_RAD = 0.03f; // 收腿到位容差，弧度

    struct Config
    {
        float pitch_zero_deg;       // IMU pitch 零偏，单位：度
        float pitch_target_deg;     // 公共 IMU 工作目标，相对水平零偏，单位：度
        float angle_start_rad[2];   // 两侧机械安全区起始角，单位：弧度
        float angle_end_rad[2];     // 两侧机械安全区终止角，单位：弧度
        int8_t motor_direction[2];  // 电机正方向，取 +1 或 -1

        // 使用安全默认值构造配置；起止角相等时配置会被判定为无效。
        // 左右最终力矩独立校准增益，必须为正的有限值；默认均为 1.0。
        float motor_torque_gain[2];
        float retract_target_rad[2];
        float retract_speed_rad_s;             // 收腿规划速度上限，弧度/秒
        float retract_position_tolerance_rad;  // 收腿到位判定容差，弧度

        Config();
        // 装入本文件的后腿配置常量；默认 Config() 仍保持无效，防止误启用。
        static Config WithAngleConstants();
    };

    typedef Config Struct_Config;

    // 使用默认安全配置构造后部状态机。
    Class_Up_Stair_Behind_Motor_FSM();
    // 使用已提供的机械零位、限位和方向配置构造后部状态机。
    explicit Class_Up_Stair_Behind_Motor_FSM(const Config &config);

    // 更新后部状态机：输入 IMU、编码器、控制权限和系统时间。
    // 状态机维护运行状态、姿态反馈和位置目标，并提供电机可控判断及力矩限幅。
    // PID 计算和电机命令发送由任务层完成。
    // 根据控制权限、IMU/编码器反馈和收腿指令更新后腿 FSM；本函数不执行 PID 或发送电机命令。
    // 左右编码器角度为原始反馈，单位弧度；retract_requested 为持续收腿请求。
    void Update(bool control_enabled,             // 上层是否允许后腿控制
                bool imu_valid,                    // 本次 IMU 数据是否有效
                bool left_feedback_valid,          // 左电机反馈是否有效
                bool right_feedback_valid,         // 右电机反馈是否有效
                float pitch_deg,                   // IMU pitch 角，单位度
                float pitch_rate_dps,              // pitch 角速度，单位度/秒
                float left_angle_rad,              // 左电机原始反馈角，单位弧度
                float right_angle_rad,             // 右电机原始反馈角，单位弧度
                uint32_t now_tick,                 // 当前系统 tick
                bool retract_requested = false,   // true=收腿，false=调平
                float left_velocity_rad_s = 0.0f,
                float right_velocity_rad_s = 0.0f); // IMU 无效时两侧零力矩

    // 返回后部状态枚举值。
    uint8_t Get_State() const;
    // 返回当前恢复斜坡比例，范围为 0 到 1。
    float Get_Output_Scale() const;

    // 获取姿态环工作目标，单位为校正后的度数。
    float Get_Target_Pitch_Deg() const;
    // 获取去除零偏后的 pitch 角度反馈，单位：度。
    float Get_Feedback_Pitch_Deg() const;
    // 控制器使用的弧度接口：IMU 度数和水平零偏校正完成后，在此转换。
    float Get_Target_Pitch_Rad() const;
    float Get_Feedback_Pitch_Rad() const;
    // 获取 pitch 角速度反馈，单位：度/秒。
    float Get_Pitch_Rate_Dps() const;
    // 查询任务当前是否应执行 pitch 姿态控制，包括恢复软启动和正常姿态保持。
    bool Uses_Attitude_Control() const;
    // 查询当前是否处于收腿移动或收腿位置保持状态。
    bool Uses_Retract_Position_Control() const;
    // 查询当前是否需要收腿位置环。
    bool Uses_Position_Control() const;
    // id 为外部电机编号 1=左、2=右；返回对应收腿目标角，单位弧度。
    float Get_Retract_Target_Angle(uint8_t id) const;
    // id 为外部电机编号 1=左、2=右；返回当前状态的位置目标角，单位弧度。
    float Get_Position_Target_Angle(uint8_t id) const;
    // id 为外部电机编号 1=左、2=右；返回对应编码器反馈角，单位弧度。
    float Get_Position_Feedback(uint8_t id) const;

    // 简短别名，方便任务读取姿态目标、角度反馈和角速度反馈。
    float Get_Target_Pitch() const { return Get_Target_Pitch_Deg(); }
    float Get_Feedback_Pitch() const { return Get_Feedback_Pitch_Deg(); }
    float Get_Pitch_Rate() const { return Get_Pitch_Rate_Dps(); }

    // 获取指定后电机的方向系数，供任务修正左右电机的 Pitch 速度方向。
    int8_t Get_Motor_Direction(uint8_t id) const;
    // 判断编码器原始角度是否有效且处于机械安全范围内。
    bool Is_Angle_Valid(uint8_t id, float raw_angle_rad) const;
    // 对外电机编号从 1 开始（1=左、2=右），内部数组从 0 开始。
    // Limit_Torque 接收任务完成电机方向修正后的原始力矩，
    // 再执行单侧反馈、机械范围、恢复比例和左右独立最终增益保护。
    // 判断指定电机当前是否可以输出姿态控制力矩。
    bool Is_Motor_Controllable(uint8_t id) const;
    // 应用软启动/增益和端点单向约束；下限禁止负力矩，上限禁止正力矩。
    // 单位 N·m；区间内不减速，端点不主动回退；反馈无效或禁用时返回 0。
    float Limit_Torque(uint8_t id, float raw_torque_nm) const;
    // 由当前电机速度估算 Kd*(目标速度-实际速度)，复用软启动/增益/限位约束，
    // 再换回 rad/s 目标；采样间隔内的真实力矩上限由驱动器保证。
    // kd 必须与本次 ctrl_Mit 命令中的 Kd 相同。
    float Limit_Velocity(uint8_t id, float target_velocity_rad_s, float kd) const;
    // 返回机械配置是否通过校验。
    bool Is_Config_Valid() const;

private:
    float motor_velocity_rad_s_[2] = {0.0f, 0.0f};
    bool Has_Mechanical_Limit(uint8_t index) const;
    bool retract_config_valid_;             // 收腿目标、速度和容差配置是否通过校验
    float position_feedback_rad_[2];        // 左右电机连续位置反馈，单位弧度
    float retract_target_angle_rad_[2];     // 左右当前收腿规划目标，单位弧度
    Alg::Utility::SlopePlanning retract_planner_[2]; // 左右收腿位置规划器
    uint32_t retract_last_tick_;             // 上次推进收腿规划器的系统 tick
    bool pending_resume_;                    // 是否等待在恢复后重新建立控制目标

    // 后部控制重新上线时，力矩比例在 2000 ms 内从 0 增加到 1。
    static const uint32_t RECOVERY_TIME_MS = 2000U;
    static constexpr float TWO_PI_RAD = 6.28318530717958647692f; // 周期角展开使用的整周弧度

    // 清除规划器、反馈和恢复状态，并切换到禁用状态。
    void Reset();
    // 检查姿态控制所需的零偏、机械角区间、电机方向和左右力矩增益。
    bool Validate_Config() const;
    // 检查收腿目标角、规划速度和到位容差。
    bool Validate_Retract_Config() const;
    // 将外部电机编号 1/2 映射为内部数组下标 0/1。
    uint8_t To_Index(uint8_t id) const;
    // 将编码器的周期角转换为机械区间对应的连续角度。
    float To_Unwrapped_Angle(uint8_t index, float raw_angle_rad) const;
    // 将收腿目标转换到指定电机机械区间内的连续角度。
    float To_Unwrapped_Retract_Target(uint8_t index) const;
    // 关闭输出并清理当前控制状态。
    void Disable();
    // 反馈恢复后开始限时力矩软启动。
    void Start_Recovery(uint32_t now_tick);
    // 根据经过的系统 tick 更新恢复斜坡比例。
    void Update_Recovery_Scale(uint32_t now_tick);
    // 接受有效收腿命令并初始化左右位置规划器。
    void Start_Retracting(uint32_t now_tick);
    // 按速度上限推进左右收腿目标角。
    void Update_Retract_Targets(uint32_t now_tick);
    // 判断左右反馈是否都进入收腿目标容差范围。
    bool Both_Retract_Targets_Reached() const;

    Config config_;                         // 机械零位、限位和方向配置
    bool config_valid_;                     // 配置是否通过安全校验
    bool motor_controllable_[2];            // 左右电机当前是否允许输出
    float motor_angle_rad_[2];              // 左右电机最近一次原始角度
    float feedback_pitch_deg_;              // 去零偏后的 pitch 角度
    float pitch_rate_dps_;                  // pitch 角速度反馈
    float output_scale_;                    // 恢复斜坡输出比例
    uint32_t recovery_start_tick_;          // 本次恢复斜坡开始时间
    uint32_t recovery_last_tick_;           // 上一次更新恢复比例的时间
    Alg::Utility::SlopePlanning recovery_planner_; // 2000 ms 力矩比例规划器
    bool feedback_degraded_;                // 是否曾出现过任一侧反馈降级
    bool feedback_valid_previous_[2];       // 上一周期左右反馈状态，用于检测恢复沿
};

#endif // UP_STAIR_BEHIND_MOTOR_FSM_HPP
