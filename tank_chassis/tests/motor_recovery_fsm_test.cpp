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

    // Leaving double-down or control-link fault must re-arm the motor even
    // when feedback remained online throughout the safety interval.
    fsm.Force_Enable_On_Next_Check();
    assert(fsm.Should_Enable(true, 6U));
    assert(!fsm.Should_Enable(true, 7U));

    // Offline retry remains rate-limited.
    assert(fsm.Should_Enable(false, 8U));
    assert(!fsm.Should_Enable(false, 107U));
    assert(fsm.Should_Enable(false, 108U));
    return 0;
}
