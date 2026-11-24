#ifndef CHASSIS_POWER_ALLOCATOR_HPP
#define CHASSIS_POWER_ALLOCATOR_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace ALG::PowerControl
{
    // All currents are amperes and speeds are motor-rotor rad/s.
    // Reserve yaw and synchronization together, then add translation.
    class ChassisPowerAllocator
    {
    public:
        float alpha = 1.0f;
        // beta and gamma expose the same shared scale for existing observers.
        float beta = 1.0f;
        float gamma = 1.0f;
        float current[4] = {};
        float predicted_power = 0.0f;
        bool feasible = true;

        void Allocate(const float translation[4], const float rotation[4],
                      const float synchronization[4], const float speed[4],
                      const float coefficients[6], float power_limit, float current_limit)
        {
            const float zero[4] = {};
            float correction[4], demand[4];
            for (unsigned i = 0; i < 4; ++i)
            {
                correction[i] = rotation[i] + synchronization[i];
                demand[i] = translation[i] + correction[i];
            }

            float scale = 0.0f;
            feasible = MaximumScale(zero, demand, speed, coefficients,
                                    power_limit, current_limit, scale);
            if (feasible && scale == 1.0f)
            {
                alpha = beta = gamma = 1.0f;
                for (unsigned i = 0; i < 4; ++i) current[i] = demand[i];
            }
            else
            {
                alpha = 0.0f;
                // Conservative sequential allocation with no translation assumed
                // when reserving yaw and synchronization authority together.
                if (MaximumScale(zero, correction, speed, coefficients,
                                 power_limit, current_limit, beta))
                {
                    feasible = true;
                    for (unsigned i = 0; i < 4; ++i) current[i] = beta * correction[i];
                    // On a rounded boundary, a subsequent interval can be
                    // empty by float precision. Keep the already feasible base.
                    MaximumScale(current, translation, speed, coefficients,
                                 power_limit, current_limit, alpha);
                    for (unsigned i = 0; i < 4; ++i) current[i] += alpha * translation[i];
                }
                else
                {
                    // Regenerative combinations can be feasible even when yaw
                    // and synchronization cannot cover speed-dependent losses alone.
                    alpha = beta = gamma = feasible ? scale : 0.0f;
                    for (unsigned i = 0; i < 4; ++i) current[i] = alpha * demand[i];
                }
            }
            gamma = beta;
            predicted_power = 0.0f;
            for (unsigned i = 0; i < 4; ++i)
            {
                const float amperes = current[i];
                const float omega = speed[i];
                predicted_power += coefficients[0] + coefficients[1] * amperes +
                    coefficients[2] * std::fabs(omega) + coefficients[3] * amperes * omega +
                    coefficients[4] * amperes * amperes + coefficients[5] * omega * omega;
            }
        }

    private:
        // Solve the intersection of [0,1], four current intervals, and the
        // convex quadratic power sublevel set. This also handles regeneration
        // where power is non-monotonic or zero scale is infeasible.
        static bool MaximumScale(const float base[4], const float direction[4],
                                 const float speed[4], const float k[6],
                                 float power_limit, float current_limit, float &scale)
        {
            double lower = 0.0, upper = 1.0;
            double a = 0.0, b = 0.0, c = -static_cast<double>(power_limit);
            scale = 0.0f;
            for (unsigned i = 0; i < 4; ++i)
            {
                const double x = base[i], d = direction[i], w = speed[i];
                if (d == 0.0)
                {
                    if (std::fabs(x) > current_limit) return false;
                }
                else
                {
                    const double first = (-current_limit - x) / d;
                    const double second = (current_limit - x) / d;
                    lower = std::max(lower, std::min(first, second));
                    upper = std::min(upper, std::max(first, second));
                }
                a += k[4] * d * d;
                b += (k[1] + k[3] * w + 2.0 * k[4] * x) * d;
                c += k[0] + k[2] * std::fabs(w) + k[5] * w * w +
                     (k[1] + k[3] * w) * x + k[4] * x * x;
            }
            if (a > 0.0)
            {
                const double discriminant = b * b - 4.0 * a * c;
                if (discriminant < 0.0) return false;
                const double root = std::sqrt(discriminant);
                // Stable quadratic roots, including a repeated root at zero.
                const double q = -0.5 * (b + std::copysign(root, b));
                const double first = q / a;
                const double second = q == 0.0 ? first : c / q;
                lower = std::max(lower, std::min(first, second));
                upper = std::min(upper, std::max(first, second));
            }
            else if (b > 0.0) upper = std::min(upper, -c / b);
            else if (b < 0.0) lower = std::max(lower, -c / b);
            else if (c > 0.0) return false;

            if (lower > upper) return false;
            // Move a rounded upper boundary into the feasible interval.
            scale = static_cast<float>(upper);
            if (scale > upper)
            {
                // scale is positive here. The preceding IEEE-754 float moves
                // toward zero without requiring MicroLIB's nextafterf symbol.
                uint32_t bits;
                std::memcpy(&bits, &scale, sizeof(bits));
                --bits;
                std::memcpy(&scale, &bits, sizeof(scale));
            }
            return true;
        }
    };
}

#endif
