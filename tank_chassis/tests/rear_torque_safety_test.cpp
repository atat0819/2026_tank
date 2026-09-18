#include "../fsm/rear_torque_safety.hpp"
#include <assert.h>

int main()
{
    // Combined pitch + roll mix must be bounded before MIT packet encoding.
    const float left = 35.0f + 20.0f;
    const float right = 35.0f - 20.0f;
    assert(StairTorqueSafety::ClampJ6248Torque(left) == 40.0f);
    assert(StairTorqueSafety::ClampJ6248Torque(right) == 15.0f);
    assert(StairTorqueSafety::ClampJ6248Torque(-55.0f) == -40.0f);
    return 0;
}
