#ifndef DM_MOTOR_MATH_HPP
#define DM_MOTOR_MATH_HPP

#include <cmath>
#include <cstdint>

namespace BSP::Motor::DM
{

inline float ClampMitCommandValue(float value, float minimum, float maximum)
{
    if (!std::isfinite(value))
    {
        value = 0.0f;
    }
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

inline int EncodeMitField(float value, float minimum, float maximum, int bits)
{
    value = ClampMitCommandValue(value, minimum, maximum);
    const float span = maximum - minimum;
    const int max_int = (1 << bits) - 1;
    return static_cast<int>((value - minimum) * static_cast<float>(max_int) / span);
}

inline float UnwrapNearestAngle(float bounded_angle, float reference_angle)
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTwoPi = 2.0f * kPi;

    while ((bounded_angle - reference_angle) > kPi)
    {
        bounded_angle -= kTwoPi;
    }
    while ((bounded_angle - reference_angle) < -kPi)
    {
        bounded_angle += kTwoPi;
    }
    return bounded_angle;
}

} // namespace BSP::Motor::DM

#endif // DM_MOTOR_MATH_HPP
