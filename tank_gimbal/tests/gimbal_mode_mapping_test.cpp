#define DT7_HPP
namespace BSP::REMOTE_CONTROL
{
class RemoteController
{
public:
    enum { UP = 1, DOWN = 2, MIDDLE = 3 };
};
}

#include "../feeder_fsm/gimbal_fsm.cpp"
#include <cassert>
#include <initializer_list>

int main()
{
    constexpr uint8_t UP = 1, DOWN = 2, MIDDLE = 3;
    constexpr uint8_t active[][2] = {
        {DOWN, MIDDLE}, {MIDDLE, DOWN}, {DOWN, UP},
        {UP, DOWN}, {MIDDLE, UP}, {UP, MIDDLE}, {UP, UP}
    };

    // Yaw、Pitch 使用同一个 FSM 类；分别验证两轴在所有运行档位都走角度双环。
    for (float step : {0.10f, 0.11f})
    {
        for (const auto &sw : active)
        {
            Class_Gimbal_FSM axis;
            Struct_Gimbal_FSM_Config config = {};
            config.angle_step = step;
            axis.Init(config);
            Struct_Gimbal_Input input = {};
            input.s1 = sw[0];
            input.s2 = sw[1];
            input.joystick_speed = 1.0f;
            axis.Update(input, 0.5f);
            assert(axis.Get_Mode_Command() == GIMBAL_MODE_ANGLE);
            assert(axis.Get_Control_Type() == GIMBAL_CONTROL_ANGLE);
            axis.Update(input, 0.5f);
            assert(axis.Get_Target_Angle() > 0.5f);
        }
    }

    Class_Gimbal_FSM axis;
    Struct_Gimbal_FSM_Config config = {};
    axis.Init(config);
    Struct_Gimbal_Input input = {};
    input.s1 = DOWN;
    input.s2 = DOWN;
    axis.Update(input, 0.0f);
    assert(axis.Get_Control_Type() == GIMBAL_CONTROL_STOP);
    assert(axis.Update_Pitch_Start_Gate(true, false, 0.0f, 0.0f) ==
           Class_Gimbal_FSM::PitchStartDecision::Normal);

    input.s1 = DOWN;
    input.s2 = MIDDLE;
    axis.Update(input, 0.0f);
    assert(axis.Get_Control_Type() == GIMBAL_CONTROL_ANGLE);
    assert(axis.Update_Pitch_Start_Gate(false, false, 0.0f, 0.0f) ==
           Class_Gimbal_FSM::PitchStartDecision::HoldZero);
    assert(axis.Update_Pitch_Start_Gate(false, false, 0.5f, 0.0f) ==
           Class_Gimbal_FSM::PitchStartDecision::ReanchorAndRelease);

    input.s1 = UP;
    input.s2 = UP;
    input.vision_ready = true;
    input.vision_fresh = true;
    axis.Update(input, 0.0f);
    assert(axis.Get_Mode_Command() == GIMBAL_MODE_VISION);
    assert(axis.Get_Control_Type() == GIMBAL_CONTROL_ANGLE);

    input.is_keymouse = true;
    input.mouse_right_held = false;
    axis.Update(input, 0.0f);
    assert(axis.Get_Mode_Command() == GIMBAL_MODE_ANGLE);
    input.mouse_right_held = true;
    axis.Update(input, 0.0f);
    assert(axis.Get_Mode_Command() == GIMBAL_MODE_VISION);
}
