#include "../fsm/rear_torque_safety.hpp"
#include <assert.h>

int main()
{
    // 最终限幅仍保护左右后腿，即使两侧力矩因独立增益而不同。
    const float left = 35.0f + 20.0f;
    const float right = 35.0f - 20.0f;
    assert(StairTorqueSafety::ClampJ6248Torque(left) == 40.0f);
    assert(StairTorqueSafety::ClampJ6248Torque(right) == 15.0f);
    assert(StairTorqueSafety::ClampJ6248Torque(-55.0f) == -40.0f);
    return 0;
}
