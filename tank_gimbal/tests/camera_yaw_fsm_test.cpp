#include "../feeder_fsm/yaw_fsm.hpp"

#include <cassert>
#include <cmath>

namespace
{
constexpr float Rad(float deg) { return deg * 0.017453292519943295f; }
bool Near(float a, float b) { return std::fabs(a - b) < 0.00001f; }
}

int main()
{
    Yaw_FSM yaw;
    Struct_Yaw_Input input{};
    input.is_keymouse = true;
    input.enabled = true;
    yaw.Update(input, Rad(23.0f), 90U);
    assert(yaw.Get_Mode() == Yaw_FSM::Mode::Center && Near(yaw.Get_Target_Angle(), Rad(23.0f)));
    yaw.Update(input, Rad(23.0f), 100U);
    assert(Near(yaw.Get_Target_Angle(), Rad(23.15f)));

    input.mouse_active = true;
    yaw.Update(input, Rad(23.0f), 101U);
    assert(yaw.Is_Target_Valid() && Near(yaw.Get_Target_Angle(), Rad(23.0f)));
    input.mouse_delta_x = 1000;
    yaw.Update(input, Rad(25.0f), 102U);
    assert(Near(yaw.Get_Target_Angle(), Rad(28.5f)));
    assert(yaw.Limit_Torque(0.2f, Rad(28.5f)) == 0.0f);
    assert(Near(yaw.Limit_Torque(-0.2f, Rad(29.0f)), -0.2f));
    input.mouse_delta_x = -1000;
    yaw.Update(input, Rad(25.0f), 103U);
    assert(Near(yaw.Get_Target_Angle(), Rad(19.5f)));
    assert(yaw.Limit_Torque(-0.2f, Rad(19.5f)) == 0.0f);
    assert(Near(yaw.Limit_Torque(0.2f, Rad(19.0f)), 0.2f));
    assert(Near(yaw.Limit_Torque(10.0f, Rad(24.0f)), 7.0f));

    input.mouse_active = false;
    yaw.Update(input, Rad(20.0f), 104U);
    assert(Near(yaw.Get_Target_Angle(), Rad(20.0f))); // Reanchor on release.
    yaw.Update(input, Rad(20.0f), 114U);
    assert(Near(yaw.Get_Target_Angle(), Rad(20.15f)));

    input.mouse_active = true;
    input.mouse_delta_x = 0;
    yaw.Update(input, Rad(20.1f), 115U);
    assert(Near(yaw.Get_Target_Angle(), Rad(20.1f)));
    input.mouse_active = false;
    yaw.Update(input, Rad(26.0f), 116U);
    assert(Near(yaw.Get_Target_Angle(), Rad(26.0f)));
    for (uint32_t tick = 126U; tick <= 316U; tick += 10U)
        yaw.Update(input, Rad(25.0f), tick);
    assert(Near(yaw.Get_Target_Angle(), Rad(24.0f)));

    input.enabled = false;
    yaw.Update(input, Rad(26.0f), 317U);
    assert(!yaw.Is_Target_Valid() && yaw.Limit_Torque(-0.2f, Rad(26.0f)) == 0.0f);
    input.enabled = true;
    yaw.Update(input, Rad(26.0f), 318U);
    assert(yaw.Get_Mode() == Yaw_FSM::Mode::Center && Near(yaw.Get_Target_Angle(), Rad(26.0f)));
    input.mouse_active = true;
    yaw.Update(input, NAN, 319U);
    assert(!yaw.Is_Target_Valid());
    yaw.Update(input, Rad(32.0f), 320U);
    assert(!yaw.Is_Target_Valid());

    input.is_keymouse = false;
    input.mouse_delta_x = 1000;
    yaw.Update(input, Rad(26.0f), 321U);
    assert(yaw.Get_Mode() == Yaw_FSM::Mode::Center && Near(yaw.Get_Target_Angle(), Rad(26.0f)));
    input.enabled = false;
    yaw.Update(input, Rad(26.0f), 322U);
    assert(yaw.Get_Mode() == Yaw_FSM::Mode::Stop && !yaw.Is_Target_Valid());
}
