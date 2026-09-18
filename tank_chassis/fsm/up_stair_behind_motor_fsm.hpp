#ifndef UP_STAIR_BEHIND_MOTOR_FSM_HPP
#define UP_STAIR_BEHIND_MOTOR_FSM_HPP

#include "../user/core/Alg/FSM/alg_fsm.hpp"
#include <stdint.h>

enum Enum_Up_Stair_Behind_Motor_Status
{
    UP_STAIR_BEHIND_MOTOR_DISABLED = 0,
    UP_STAIR_BEHIND_MOTOR_RECOVERING,
    UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD,
    UP_STAIR_BEHIND_MOTOR_COUNT
};

class Class_Up_Stair_Behind_Motor_FSM : public Class_FSM
{
public:
    struct Config
    {
        float pitch_zero_deg;
        float roll_zero_deg;
        float angle_start_rad[2];
        float angle_end_rad[2];
        int8_t motor_direction[2];

        Config();
    };

    typedef Config Struct_Config;

    Class_Up_Stair_Behind_Motor_FSM();
    explicit Class_Up_Stair_Behind_Motor_FSM(const Config &config);

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

    uint8_t Get_State() const;
    float Get_Output_Scale() const;

    float Get_Target_Pitch_Deg() const;
    float Get_Target_Roll_Deg() const;
    float Get_Feedback_Pitch_Deg() const;
    float Get_Feedback_Roll_Deg() const;
    float Get_Pitch_Rate_Dps() const;
    float Get_Roll_Rate_Dps() const;

    // Short aliases keep the attitude interface convenient for control code.
    float Get_Target_Pitch() const { return Get_Target_Pitch_Deg(); }
    float Get_Target_Roll() const { return Get_Target_Roll_Deg(); }
    float Get_Feedback_Pitch() const { return Get_Feedback_Pitch_Deg(); }
    float Get_Feedback_Roll() const { return Get_Feedback_Roll_Deg(); }
    float Get_Pitch_Rate() const { return Get_Pitch_Rate_Dps(); }
    float Get_Roll_Rate() const { return Get_Roll_Rate_Dps(); }

    int8_t Get_Motor_Direction(uint8_t id) const;
    bool Is_Angle_Valid(uint8_t id, float raw_angle_rad) const;
    bool Is_Motor_Controllable(uint8_t id) const;
    float Limit_Torque(uint8_t id, float raw_torque_nm) const;
    bool Is_Config_Valid() const;

private:
    static const uint32_t RECOVERY_TIME_MS = 300U;
    static constexpr float TWO_PI_RAD = 6.28318530717958647692f;
    static constexpr float LIMIT_MARGIN_RAD = 0.05235987755982989f;

    void Reset();
    bool Validate_Config() const;
    uint8_t To_Index(uint8_t id) const;
    float To_Unwrapped_Angle(uint8_t index, float raw_angle_rad) const;
    void Disable();

    Config config_;
    bool config_valid_;
    bool motor_controllable_[2];
    float motor_angle_rad_[2];
    float feedback_pitch_deg_;
    float feedback_roll_deg_;
    float pitch_rate_dps_;
    float roll_rate_dps_;
    float output_scale_;
    uint32_t recovery_start_tick_;
};

#endif // UP_STAIR_BEHIND_MOTOR_FSM_HPP
