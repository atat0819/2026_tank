#include "../user/core/BSP/Motor/DM/DmMotor.hpp"

#include <cassert>
#include <cmath>

int main()
{
    using BSP::Motor::DM::ClampMitCommandValue;
    using BSP::Motor::DM::EncodeMitField;
    using BSP::Motor::DM::UnwrapNearestAngle;

    assert(ClampMitCommandValue(NAN, -9.0f, 9.0f) == 0.0f);
    assert(ClampMitCommandValue(-10.0f, -9.0f, 9.0f) == -9.0f);
    assert(ClampMitCommandValue(10.0f, -9.0f, 9.0f) == 9.0f);

    const float pi = 3.14159265358979323846f;
    // 新位置映射：负半圈、零点、正半圈对应编码下限、中点、上限。
    assert(EncodeMitField(-pi, -pi, pi, 16) == 0);
    assert(EncodeMitField(0.0f, -pi, pi, 16) == 32767);
    assert(EncodeMitField(pi, -pi, pi, 16) == 65535);
    assert(EncodeMitField(-2.0f * pi, -pi, pi, 16) == 0);
    assert(EncodeMitField(2.0f * pi, -pi, pi, 16) == 65535);
    assert(EncodeMitField(0.0f, 0.0f, 2.0f * pi, 16) == 0);
    assert(EncodeMitField(2.0f * pi, 0.0f, 2.0f * pi, 16) == 65535);
    assert(std::abs(EncodeMitField(pi, 0.0f, 2.0f * pi, 16) - 32767) <= 1);

    const float current = 181.0f * pi / 180.0f;
    const float bounded_target = -179.0f * pi / 180.0f;
    assert(std::abs(UnwrapNearestAngle(bounded_target, current) -
                    181.0f * pi / 180.0f) < 1.0e-5f);
    assert(std::abs(UnwrapNearestAngle(179.0f * pi / 180.0f,
                                     -179.0f * pi / 180.0f) -
                    (-181.0f * pi / 180.0f)) < 1.0e-5f);

    return 0;
}
