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
    // 改配置应改变实际边界，不能仍使用 FSM 内写死的 +/-60 度。
    {
        Struct_Gimbal_FSM_Config adjustable = {};
        adjustable.min_angle = -25.0f * deg_to_rad;
        adjustable.max_angle = 40.0f * deg_to_rad;
        adjustable.limit_angle = 1U;
        adjustable.limit_target_angle = 0U;
        Class_Gimbal_FSM yaw;
        yaw.Init(adjustable, GIMBAL_STATUS_ANGLE);
        yaw.Set_Target_Angle(11.0f);
        yaw.Update_Angle_Limit(adjustable.max_angle, 10.0f);
        assert(yaw.Get_Target_Angle() == 10.0f);
        assert(yaw.Limit_Torque(2.0f) == 0.0f);
        assert(yaw.Limit_Torque(-2.0f) == -2.0f);
        yaw.Set_Target_Angle(9.0f);
        yaw.Update_Angle_Limit(adjustable.min_angle, 10.0f);
        assert(yaw.Get_Target_Angle() == 10.0f);
        assert(yaw.Limit_Torque(-2.0f) == 0.0f);
        assert(yaw.Limit_Torque(2.0f) == 2.0f);
        yaw.Update_Angle_Limit(adjustable.max_angle - 0.001f, 10.0f);
        assert(yaw.Limit_Torque(2.0f) == 2.0f);
    }
    Struct_Gimbal_FSM_Config config = {};
    config.min_angle = -60.0f * deg_to_rad;
    config.max_angle = 60.0f * deg_to_rad;
    config.limit_angle = 1U;
    config.limit_target_angle = 0U;
    config.angle_step = 0.01f;
    config.mouse_angle_scale = 0.01f;
    config.normalize_angle = 1U;
    config.continuous_angle = 1U;

    for (int side : {-1, 1})
    {
        for (bool mouse : {false, true})
        {
            Class_Gimbal_FSM yaw;
            yaw.Init(config);
            Struct_Gimbal_Input input = {};
            input.s1 = 2;
            input.s2 = 3;
            input.is_keymouse = mouse;
            input.joystick_speed = static_cast<float>(side);
            input.mouse_angle_delta = static_cast<float>(5 * side);
            yaw.Update(input, 10.0f);
            yaw.Take_Mode_Changed_Flag();
            for (int i = 0; i < 1000; ++i)
            {
                yaw.Update(input, 10.0f);
                yaw.Update_Angle_Limit(side * 60.0f * deg_to_rad, 10.0f);
                assert(yaw.Get_Target_Angle() == 10.0f);
                assert(yaw.Take_Limit_Reset_Flag() != 0U);
                assert(yaw.Take_Limit_Reset_Flag() == 0U);
            }
            input.joystick_speed *= -1.0f;
            input.mouse_angle_delta *= -1.0f;
            yaw.Update(input, 10.0f);
            yaw.Update_Angle_Limit(side * 60.0f * deg_to_rad, 10.0f);
            assert((yaw.Get_Target_Angle() - 10.0f) * side < 0.0f);
            assert(yaw.Take_Limit_Reset_Flag() == 0U);
            assert(yaw.Limit_Torque(side * 2.0f) == 0.0f);
            assert(yaw.Take_Limit_Reset_Flag() != 0U);
            assert(yaw.Limit_Torque(side * -2.0f) == side * -2.0f);
            assert(yaw.Take_Limit_Reset_Flag() == 0U);
            assert(yaw.Limit_Torque(0.0f) == 0.0f);
            // IMU 连续角已越过一圈，边界仍由编码器决定。
            yaw.Set_Target_Angle(16.2831853f + static_cast<float>(side));
            yaw.Update_Angle_Limit(side * 60.0f * deg_to_rad, 16.2831853f);
            assert(yaw.Get_Target_Angle() == 16.2831853f);
            yaw.Update_Angle_Limit(side * 65.0f * deg_to_rad, 16.2831853f);
            assert(yaw.Limit_Torque(side * 2.0f) == 0.0f);
            assert(yaw.Limit_Torque(side * -2.0f) == side * -2.0f);
        }

        Class_Gimbal_FSM yaw;
        yaw.Init(config);
        Struct_Gimbal_Input vision = {};
        vision.s1 = 1;
        vision.s2 = 1;
        vision.vision_ready = true;
        vision.vision_fresh = true;
        vision.vision_angle = static_cast<float>(side);
        yaw.Update(vision, 0.0f);
        yaw.Take_Mode_Changed_Flag();
        yaw.Update(vision, 0.0f);
        yaw.Update_Angle_Limit(side * 60.0f * deg_to_rad, 0.0f);
        assert(yaw.Get_Target_Angle() == 0.0f);
        assert(yaw.Take_Limit_Reset_Flag() != 0U);
        yaw.Update_Angle_Limit(0.0f, 0.0f);
        assert(yaw.Limit_Torque(side * 2.0f) == side * 2.0f);
    }
}
