#define DT7_HPP
namespace BSP::REMOTE_CONTROL {
class RemoteController { public: enum { UP = 1, DOWN = 2, MIDDLE = 3 }; };
}
#include "../feeder_fsm/gimbal_fsm.cpp"
#include <cassert>
#include <cmath>
#include <initializer_list>

int main()
{
    constexpr uint8_t DOWN = 2;
    constexpr uint8_t MIDDLE = 3;
    constexpr float current_angle = 0.3f;
    for (bool keymouse : {false, true})
    {
        for (float movement : {0.0f, -0.5f, 0.5f})
        {
            Class_Gimbal_FSM pitch;
            Struct_Gimbal_FSM_Config config = {};
            config.angle_step = 0.02f;
            config.mouse_angle_scale = 0.02f;
            pitch.Init(config);
            Struct_Gimbal_Input input = {};
            input.s1 = DOWN;
            input.s2 = DOWN;
            pitch.Update(input, current_angle);
            assert(pitch.Get_Control_Type() == GIMBAL_CONTROL_STOP);
            assert(pitch.Get_Control_Output() == 0.0f);

            input.s1 = MIDDLE;
            input.s2 = keymouse ? MIDDLE : DOWN;
            input.is_keymouse = keymouse;
            pitch.Update(input, current_angle);
            assert(pitch.Get_Control_Type() == GIMBAL_CONTROL_ANGLE);
            assert(std::fabs(pitch.Get_Target_Angle() - current_angle) < 1e-6f);
            assert(std::fabs(pitch.Get_Control_Output() - current_angle) < 1e-6f);

            // Either input direction works without a neutral/upward release sequence.
            input.joystick_speed = movement;
            input.mouse_angle_delta = movement * 10.0f;
            pitch.Update(input, current_angle);
            const float expected = current_angle + movement *
                (keymouse ? 10.0f * config.mouse_angle_scale : config.angle_step);
            assert(std::fabs(pitch.Get_Target_Angle() - expected) < 1e-6f);

            input.is_keymouse = false;
            input.s1 = DOWN;
            input.s2 = DOWN;
            pitch.Update(input, current_angle);
            assert(pitch.Get_Control_Type() == GIMBAL_CONTROL_STOP);
            assert(pitch.Get_Control_Output() == 0.0f);
            input.s1 = 0;
            input.s2 = 0;
            pitch.Update(input, current_angle);
            assert(pitch.Get_Control_Type() == GIMBAL_CONTROL_STOP);
        }
    }
}
