#define DT7_HPP
namespace BSP::REMOTE_CONTROL {
class RemoteController { public: enum { UP = 1, DOWN = 2, MIDDLE = 3 }; };
}
#include "../feeder_fsm/gimbal_fsm.cpp"
#include <cassert>
#include <initializer_list>

int main()
{
    constexpr float deg_to_rad = 3.14159265358979323846f / 180.0f;
    Struct_Gimbal_FSM_Config config = {};
    config.angle_step = 0.01f;
    config.mouse_angle_scale = 0.01f;
    config.min_angle = 34.2f * deg_to_rad;
    config.max_angle = 67.9f * deg_to_rad;
    config.limit_angle = 1U;
    config.limit_upper_torque = 0U;
    config.limit_target_angle = 0U;

    // 重新锚定不夹紧绝对目标角；上限允许正力矩支撑/制动，不触发力矩复位。
    {
        Class_Gimbal_FSM pitch;
        pitch.Init(config, GIMBAL_STATUS_ANGLE);
        pitch.ReAnchor(2.0f);
        assert(pitch.Get_Target_Angle() == 2.0f);
        pitch.Update_Angle_Limit(config.max_angle, 2.0f);
        assert(pitch.Take_Limit_Reset_Flag() == 0U);
        assert(pitch.Limit_Torque(1.0f) == 1.0f);
        assert(pitch.Limit_Torque(-1.0f) == -1.0f);
        assert(pitch.Take_Limit_Reset_Flag() == 0U);
    }

    // IMU 闭环角与编码器限位角零点不同，也必须收回到当前控制角而非编码器边界。
    for (int side : {-1, 1})
    {
        const float encoder_boundary = side > 0 ? config.max_angle : config.min_angle;
        const float imu_control_angle = -0.2f;
        Class_Gimbal_FSM pitch;
        pitch.Init(config, GIMBAL_STATUS_ANGLE);
        pitch.Set_Target_Angle(imu_control_angle + side * 0.5f);
        pitch.Update_Angle_Limit(encoder_boundary - side * 0.01f, imu_control_angle);
        assert(pitch.Get_Target_Angle() == imu_control_angle + side * 0.5f);
        assert(pitch.Limit_Torque(side * 2.0f) == side * 2.0f);
        pitch.Update_Angle_Limit(encoder_boundary, imu_control_angle);
        assert(pitch.Get_Target_Angle() == imu_control_angle);
        assert(pitch.Limit_Torque(side * 2.0f) == (side > 0 ? 2.0f : 0.0f));
        assert(pitch.Limit_Torque(side * -2.0f) == side * -2.0f);
        pitch.Set_Target_Angle(imu_control_angle - side * 0.1f);
        pitch.Update_Angle_Limit(encoder_boundary, imu_control_angle);
        assert(pitch.Get_Target_Angle() == imu_control_angle - side * 0.1f);
    }

    for (int side : {-1, 1})
    {
        const float boundary = side > 0 ? config.max_angle : config.min_angle;
        for (bool mouse : {false, true})
        {
            Class_Gimbal_FSM pitch;
            pitch.Init(config);
            Struct_Gimbal_Input input = {};
            input.s1 = 2;
            input.s2 = 3;
            input.is_keymouse = mouse;
            input.joystick_speed = static_cast<float>(side);
            input.mouse_angle_delta = static_cast<float>(5 * side);
            pitch.Update(input, boundary);
            pitch.Take_Mode_Changed_Flag();
            for (int i = 0; i < 1000; ++i)
            {
                pitch.Update(input, boundary);
                pitch.Update_Angle_Limit(boundary, boundary);
                assert(pitch.Get_Target_Angle() == boundary);
            }
            assert(pitch.Limit_Torque(side * 2.0f) == (side > 0 ? 2.0f : 0.0f));
            assert(pitch.Take_Limit_Reset_Flag() != 0U);
            assert(pitch.Take_Limit_Reset_Flag() == 0U);
            assert(pitch.Limit_Torque(side * -2.0f) == side * -2.0f);
            assert(pitch.Take_Limit_Reset_Flag() == 0U);
            assert(pitch.Limit_Torque(0.0f) == 0.0f);

            input.joystick_speed *= -1.0f;
            input.mouse_angle_delta *= -1.0f;
            pitch.Update(input, boundary);
            pitch.Update_Angle_Limit(boundary, boundary);
            assert((pitch.Get_Target_Angle() - boundary) * side < 0.0f);

            // 实际还在范围内，目标允许超过边界，不提前截断跟随力矩。
            const float inside = boundary - side * 0.01f;
            pitch.Set_Target_Angle(boundary + side * 0.1f);
            pitch.Update_Angle_Limit(inside, inside);
            assert(pitch.Get_Target_Angle() == boundary + side * 0.1f);
            assert(pitch.Limit_Torque(side * 2.0f) == side * 2.0f);

            // 实际越界时仍允许朝范围内纠正。
            const float outside = boundary + side * 0.1f;
            pitch.Update_Angle_Limit(outside, outside);
            assert(pitch.Get_Target_Angle() == outside);
            assert(pitch.Limit_Torque(side * 2.0f) == (side > 0 ? 2.0f : 0.0f));
            assert(pitch.Limit_Torque(side * -2.0f) == side * -2.0f);
        }

        Class_Gimbal_FSM pitch;
        pitch.Init(config);
        Struct_Gimbal_Input vision = {};
        vision.s1 = 1;
        vision.s2 = 1;
        vision.vision_ready = true;
        vision.vision_fresh = true;
        vision.vision_angle = boundary + side;
        pitch.Update(vision, boundary);
        pitch.Take_Mode_Changed_Flag();
        pitch.Update(vision, boundary);
        pitch.Update_Angle_Limit(boundary, boundary);
        assert(pitch.Get_Target_Angle() == boundary);
        assert(pitch.Limit_Torque(side * 2.0f) == (side > 0 ? 2.0f : 0.0f));
    }

    // 速度模式同样允许上限正力矩，且不会因此反复复位；下限仍清负力矩并请求复位。
    {
        Class_Gimbal_FSM pitch;
        pitch.Init(config, GIMBAL_STATUS_SPEED);
        pitch.Update_Angle_Limit(config.max_angle + 0.01f, -0.2f);
        assert(pitch.Limit_Torque(2.0f) == 2.0f);
        assert(pitch.Take_Limit_Reset_Flag() == 0U);
        pitch.Update_Angle_Limit(config.min_angle, -0.2f);
        assert(pitch.Limit_Torque(-2.0f) == 0.0f);
        assert(pitch.Take_Limit_Reset_Flag() != 0U);
        assert(pitch.Take_Limit_Reset_Flag() == 0U);
        assert(pitch.Limit_Torque(2.0f) == 2.0f);
        assert(pitch.Take_Limit_Reset_Flag() == 0U);
    }

    // 关闭限位时，目标和力矩均不被保护流程拦截。
    config.limit_angle = 0U;
    Class_Gimbal_FSM disabled;
    disabled.Init(config, GIMBAL_STATUS_ANGLE);
    disabled.Set_Target_Angle(2.0f);
    disabled.Update_Angle_Limit(1.0f, 1.0f);
    assert(disabled.Get_Target_Angle() == 2.0f);
    assert(disabled.Limit_Torque(2.0f) == 2.0f);
    assert(disabled.Take_Limit_Reset_Flag() == 0U);
}
