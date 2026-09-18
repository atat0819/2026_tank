#ifndef UP_STAIR_BEHIND_MOTOR_FSM_HPP
#define UP_STAIR_BEHIND_MOTOR_FSM_HPP

#include "../user/core/Alg/FSM/alg_fsm.hpp"
#include "../user/core/Alg/UtilityFunction/SlopePlanning.hpp"
#include <stdint.h>

enum Enum_Up_Stair_Behind_Motor_Status
{
    UP_STAIR_BEHIND_MOTOR_DISABLED = 0,       // 后连杆控制关闭，力矩为零
    UP_STAIR_BEHIND_MOTOR_RECOVERING,         // 反馈恢复后，力矩比例软启动
    UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD,      // 正常进行 pitch/roll 姿态控制
    UP_STAIR_BEHIND_MOTOR_COUNT
};

class Class_Up_Stair_Behind_Motor_FSM : public Class_FSM
{
public:
    struct Config
    {
        float pitch_zero_deg;       // IMU pitch 零偏，单位：度
        float roll_zero_deg;        // IMU roll 零偏，单位：度
        float angle_start_rad[2];   // 两侧机械安全区起始角，单位：弧度
        float angle_end_rad[2];     // 两侧机械安全区终止角，单位：弧度
        int8_t motor_direction[2];  // 电机正方向，取 +1 或 -1

        // 使用安全默认值构造配置；起止角相等时配置会被判定为无效。
        Config();
    };

    typedef Config Struct_Config;

    // 使用默认安全配置构造后部状态机。
    Class_Up_Stair_Behind_Motor_FSM();
    // 使用已提供的机械零位、限位和方向配置构造后部状态机。
    explicit Class_Up_Stair_Behind_Motor_FSM(const Config &config);

    // 更新后部状态机：输入 IMU、编码器、控制权限和系统时间。
    // 状态机只产生姿态反馈、可控判断和力矩限幅结果，不执行 PID。
    void Update(bool control_enabled,
                bool imu_valid,
                bool left_feedback_valid,
                bool right_feedback_valid,
                float pitch_deg,
                float roll_deg,
                float pitch_rate_dps,
                float roll_rate_dps,
                float left_angle_rad,
                float right_angle_rad,
                uint32_t now_tick);

    // 返回后部状态枚举值。
    uint8_t Get_State() const;
    // 返回当前恢复斜坡比例，范围为 0 到 1。
    float Get_Output_Scale() const;

    // 获取姿态环目标角度；当前目标为水平姿态 0 度。
    float Get_Target_Pitch_Deg() const;
    float Get_Target_Roll_Deg() const;
    // 获取去除零偏后的 pitch/roll 角度反馈，单位：度。
    float Get_Feedback_Pitch_Deg() const;
    float Get_Feedback_Roll_Deg() const;
    // 获取 pitch/roll 角速度反馈，单位：度/秒。
    float Get_Pitch_Rate_Dps() const;
    float Get_Roll_Rate_Dps() const;

    // 简短别名，方便任务读取姿态目标、角度反馈和角速度反馈。
    float Get_Target_Pitch() const { return Get_Target_Pitch_Deg(); }
    float Get_Target_Roll() const { return Get_Target_Roll_Deg(); }
    float Get_Feedback_Pitch() const { return Get_Feedback_Pitch_Deg(); }
    float Get_Feedback_Roll() const { return Get_Feedback_Roll_Deg(); }
    float Get_Pitch_Rate() const { return Get_Pitch_Rate_Dps(); }
    float Get_Roll_Rate() const { return Get_Roll_Rate_Dps(); }

    // 获取指定后电机的方向系数，供任务进行左右混控方向修正。
    int8_t Get_Motor_Direction(uint8_t id) const;
    // 判断编码器原始角度是否有效且处于机械安全范围内。
    bool Is_Angle_Valid(uint8_t id, float raw_angle_rad) const;
    // 对外电机编号从 1 开始（1=左、2=右），内部数组从 0 开始。
    // Limit_Torque 接收任务完成方向修正和左右混控后的原始力矩，
    // 再执行单侧反馈、机械范围和恢复比例保护。
    // 判断指定电机当前是否可以输出姿态控制力矩。
    bool Is_Motor_Controllable(uint8_t id) const;
    // 应用反馈状态、机械边界、恢复比例和有限值检查，返回最终允许力矩。
    float Limit_Torque(uint8_t id, float raw_torque_nm) const;
    // 返回机械配置是否通过校验。
    bool Is_Config_Valid() const;

private:
    // 后部控制重新上线时，力矩比例在 300 ms 内从 0 增加到 1。
    static const uint32_t RECOVERY_TIME_MS = 300U;
    static constexpr float TWO_PI_RAD = 6.28318530717958647692f;
    static constexpr float LIMIT_MARGIN_RAD = 0.05235987755982989f;

    void Reset();
    bool Validate_Config() const;
    uint8_t To_Index(uint8_t id) const;
    float To_Unwrapped_Angle(uint8_t index, float raw_angle_rad) const;
    void Disable();
    void Start_Recovery(uint32_t now_tick);
    void Update_Recovery_Scale(uint32_t now_tick);

    Config config_;                         // 机械零位、限位和方向配置
    bool config_valid_;                     // 配置是否通过安全校验
    bool motor_controllable_[2];            // 左右电机当前是否允许输出
    float motor_angle_rad_[2];              // 左右电机最近一次原始角度
    float feedback_pitch_deg_;              // 去零偏后的 pitch 角度
    float feedback_roll_deg_;               // 去零偏后的 roll 角度
    float pitch_rate_dps_;                  // pitch 角速度反馈
    float roll_rate_dps_;                   // roll 角速度反馈
    float output_scale_;                    // 恢复斜坡输出比例
    uint32_t recovery_start_tick_;          // 本次恢复斜坡开始时间
    uint32_t recovery_last_tick_;           // 上一次更新恢复比例的时间
    Alg::Utility::SlopePlanning recovery_planner_; // 300 ms 力矩比例规划器
    bool feedback_degraded_;                // 是否曾出现过任一侧反馈降级
    bool feedback_valid_previous_[2];       // 上一周期左右反馈状态，用于检测恢复沿
};

#endif // UP_STAIR_BEHIND_MOTOR_FSM_HPP
