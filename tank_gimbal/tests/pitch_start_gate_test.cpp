#include "../feeder_fsm/gimbal_fsm.hpp"
#include <cassert>

using Decision = Class_Gimbal_FSM::PitchStartDecision;

int main()
{
    constexpr uint8_t UP = 1;
    constexpr uint8_t DOWN = 2;
    constexpr uint8_t MIDDLE = 3;
    Class_Gimbal_FSM pitch;
    Struct_Gimbal_FSM_Config config = {};
    pitch.Init(config);

    Struct_Gimbal_Input input = {};
    auto step = [&](uint8_t s1, uint8_t s2, float stick) {
        input.s1 = s1;
        input.s2 = s2;
        input.joystick_speed = stick;
        pitch.Update(input, 0.0f);
        return pitch.Update_Pitch_Start_Gate(s1 == DOWN && s2 == DOWN, stick);
    };

    assert(step(DOWN, MIDDLE, 0.0f) == Decision::Normal);
    assert(step(MIDDLE, DOWN, 0.5f) == Decision::Normal); // speed -> angle

    assert(step(DOWN, DOWN, 0.0f) == Decision::Normal);
    assert(step(MIDDLE, DOWN, 0.0f) == Decision::HoldZero);
    assert(step(MIDDLE, DOWN, -0.5f) == Decision::HoldZero);
    assert(step(MIDDLE, DOWN, -0.01f) == Decision::HoldZero);
    assert(step(MIDDLE, DOWN, 0.005f) == Decision::HoldZero);
    assert(step(MIDDLE, DOWN, 0.009f) == Decision::HoldZero);
    assert(step(MIDDLE, DOWN, 0.01f) == Decision::HoldZero);
    assert(step(MIDDLE, DOWN, 0.02f) == Decision::ReanchorAndRelease);
    assert(step(MIDDLE, DOWN, 0.0f) == Decision::Normal);
    assert(step(DOWN, MIDDLE, 0.0f) == Decision::Normal);
    assert(step(MIDDLE, DOWN, 0.0f) == Decision::Normal); // speed -> angle

    step(DOWN, DOWN, 0.5f);
    assert(step(UP, DOWN, 0.5f) == Decision::HoldZero);
    assert(step(UP, DOWN, 0.0f) == Decision::HoldZero);
    assert(step(UP, DOWN, 0.5f) == Decision::ReanchorAndRelease);

    step(DOWN, DOWN, 0.0f);
    assert(step(0, 0, 0.0f) == Decision::Normal); // invalid switch position
    assert(step(MIDDLE, DOWN, 0.0f) == Decision::HoldZero);
    assert(step(0, 0, 0.5f) == Decision::HoldZero);
    assert(step(MIDDLE, DOWN, 0.5f) == Decision::ReanchorAndRelease);
}
