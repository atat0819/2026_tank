#include <cassert>
#include <cmath>
#include <cstdio>
#include "user/core/Alg/UtilityFunction/SecondOrderSetpointPlanner.hpp"

using Planner = Alg::Utility::SecondOrderSetpointPlanner;

static void step(Planner &planner, float target, float &previous_velocity, float &previous_acceleration)
{
    planner.Update(target);
    const float velocity = planner.GetOut(), acceleration = planner.GetAcceleration();
    assert(std::isfinite(velocity) && std::isfinite(acceleration));
    assert(std::fabs(velocity) <= 2.56001f);
    assert(std::fabs(acceleration) <= 4.001f);
    assert(std::fabs(acceleration - previous_acceleration) <= .042f);
    assert(std::fabs((velocity - previous_velocity) / .001f - acceleration) < .001f);
    previous_velocity = velocity;
    previous_acceleration = acceleration;
}

int main()
{
    Planner planner(4.0f, 40.0f, .001f);
    float velocity = 0, acceleration = 0;
    bool reached_acceleration_limit = false, braking = false;
    for (float target : {2.56f, 0.0f, -2.56f, 1.7f, 0.0f})
    {
        for (unsigned i = 0; i < 2000; ++i)
        {
            step(planner, target, velocity, acceleration);
            reached_acceleration_limit |= std::fabs(acceleration) > 3.99f;
            braking |= velocity > .1f && acceleration < -1;
        }
        assert(velocity == target && acceleration == 0);
    }
    assert(reached_acceleration_limit && braking);
    // Retarget while accelerating or at the acceleration cap, rather than only at rest.
    for (unsigned i = 0; i < 10000; ++i)
    {
        const float target = ((i / 137) % 3 == 0) ? 2.56f : ((i / 137) % 3 == 1 ? -1.7f : 0.0f);
        step(planner, target, velocity, acceleration);
    }
    for (unsigned i = 0; i < 2000; ++i) step(planner, 0, velocity, acceleration);
    assert(velocity == 0 && acceleration == 0);
    planner.Update(2.56f);
    planner.Reset();
    assert(planner.GetOut() == 0 && planner.GetAcceleration() == 0);
    planner.Update(0);
    assert(planner.GetOut() == 0 && planner.GetAcceleration() == 0);
    std::puts("PASS: bounded speed/acceleration/jerk, endpoint, braking, mid-ramp reversal and reset");
}
