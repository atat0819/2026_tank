#include "../fsm/motor_recovery_fsm.hpp"
#include <assert.h>

int main()
{
    MotorRecoveryFSM fsm;
    fsm.Init(0U);

    // A motor can return feedback while still disabled. Enable once at its
    // initial online transition, then never repeat while it remains online.
    assert(fsm.Should_Enable(true, 0U));
    assert(!fsm.Should_Enable(true, 1U));

    // After a power loss, send probes while offline. Once feedback returns,
    // send another enable frame that the newly powered motor can receive.
    assert(fsm.Should_Enable(false, 2U));
    assert(!fsm.Should_Enable(false, 3U));
    assert(fsm.Should_Enable(true, 4U));
    assert(!fsm.Should_Enable(true, 5U));

    // Offline retry remains rate-limited.
    assert(fsm.Should_Enable(false, 6U));
    assert(!fsm.Should_Enable(false, 105U));
    assert(fsm.Should_Enable(false, 106U));
    return 0;
}
