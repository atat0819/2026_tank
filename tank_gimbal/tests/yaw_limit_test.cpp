#define DT7_HPP
namespace BSP::REMOTE_CONTROL {
class RemoteController { public: enum { UP = 1, DOWN = 2, MIDDLE = 3 }; };
}
#include "../feeder_fsm/gimbal_fsm.cpp"
#include <cassert>
#include <initializer_list>

int main()
{
    Struct_Gimbal_FSM_Config config = {};
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
                yaw.Update_Yaw_Limit(side * 60.0f, 10.0f);
                assert(yaw.Get_Target_Angle() == 10.0f);
                assert(yaw.Take_Yaw_Limit_Reset_Flag() != 0U);
                assert(yaw.Take_Yaw_Limit_Reset_Flag() == 0U);
            }
            input.joystick_speed *= -1.0f;
            input.mouse_angle_delta *= -1.0f;
            yaw.Update(input, 10.0f);
            yaw.Update_Yaw_Limit(side * 60.0f, 10.0f);
            assert((yaw.Get_Target_Angle() - 10.0f) * side < 0.0f);
            assert(yaw.Take_Yaw_Limit_Reset_Flag() == 0U);
            assert(yaw.Limit_Yaw_Torque(side * 2.0f) == 0.0f);
            assert(yaw.Take_Yaw_Limit_Reset_Flag() != 0U);
            assert(yaw.Limit_Yaw_Torque(side * -2.0f) == side * -2.0f);
            assert(yaw.Take_Yaw_Limit_Reset_Flag() == 0U);
            assert(yaw.Limit_Yaw_Torque(0.0f) == 0.0f);
            // IMU 连续角已越过一圈，边界仍由编码器决定。
            yaw.Set_Target_Angle(16.2831853f + static_cast<float>(side));
            yaw.Update_Yaw_Limit(side * 60.0f, 16.2831853f);
            assert(yaw.Get_Target_Angle() == 16.2831853f);
            yaw.Update_Yaw_Limit(side * 65.0f, 16.2831853f);
            assert(yaw.Limit_Yaw_Torque(side * 2.0f) == 0.0f);
            assert(yaw.Limit_Yaw_Torque(side * -2.0f) == side * -2.0f);
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
        yaw.Update_Yaw_Limit(side * 60.0f, 0.0f);
        assert(yaw.Get_Target_Angle() == 0.0f);
        assert(yaw.Take_Yaw_Limit_Reset_Flag() != 0U);
        yaw.Update_Yaw_Limit(0.0f, 0.0f);
        assert(yaw.Limit_Yaw_Torque(side * 2.0f) == side * 2.0f);
    }
}
