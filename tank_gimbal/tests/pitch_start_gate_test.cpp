#define DT7_HPP
namespace BSP::REMOTE_CONTROL {
class RemoteController { public: enum { UP = 1, DOWN = 2, MIDDLE = 3 }; };
}
#include "../feeder_fsm/gimbal_fsm.cpp"
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
    auto step = [&](uint8_t s1, uint8_t s2, float stick,
                    bool keymouse = false, float mouse_y = 0.0f) {
        input.s1 = s1;
        input.s2 = s2;
        input.is_keymouse = keymouse;
        input.joystick_speed = stick;
        input.mouse_angle_delta = mouse_y;
        pitch.Update(input, 0.0f);
        return pitch.Update_Pitch_Start_Gate(
            s1 == DOWN && s2 == DOWN, keymouse, stick, mouse_y);
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

    // 双下转键鼠：右摇杆上拨不能解锁；鼠标必须先静止，再上移超过 2 像素。
    assert(step(DOWN, DOWN, 0.0f) == Decision::Normal);
    assert(step(MIDDLE, MIDDLE, 0.5f, true, 5.0f) == Decision::HoldZero);
    assert(step(MIDDLE, MIDDLE, 0.5f, true, -5.0f) == Decision::HoldZero);
    assert(step(MIDDLE, MIDDLE, 0.5f, true, 2.0f) == Decision::HoldZero);
    assert(step(MIDDLE, MIDDLE, 0.5f, true, 3.0f) == Decision::ReanchorAndRelease);
    assert(step(MIDDLE, MIDDLE, 0.0f, true, 0.0f) == Decision::Normal);

    // 遥控器档看过回中后改键鼠档，仍须等待鼠标自身进入死区。
    assert(step(DOWN, DOWN, 0.0f) == Decision::Normal);
    assert(step(MIDDLE, DOWN, 0.0f) == Decision::HoldZero);
    assert(step(MIDDLE, MIDDLE, 0.0f, true, 5.0f) == Decision::HoldZero);
    assert(step(MIDDLE, MIDDLE, 0.0f, true, 0.0f) == Decision::HoldZero);
    assert(step(MIDDLE, MIDDLE, 0.0f, true, 3.0f) == Decision::ReanchorAndRelease);
}
