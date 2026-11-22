#include "../fsm/motor_recovery_fsm.hpp"
#include <assert.h>

int main()
{
    MotorRecoveryFSM fsm;
    using Request = MotorRecoveryFSM::Request;

    // A valid non-stop gear permits On only with a confirmed disabled drive.
    assert(fsm.Update(true, 0xFFU, 0U) == Request::None);
    assert(fsm.Update(true, 1U, 1U) == Request::None);
    assert(fsm.Update(true, 0xAU, 2U) == Request::None);
    assert(fsm.Update(true, 0U, 3U) == Request::On);
    assert(fsm.Update(true, 0U, 102U) == Request::None);
    assert(fsm.Update(true, 0U, 103U) == Request::On);
    assert(fsm.Update(true, 1U, 104U) == Request::None);
    assert(fsm.Update(true, 0U, 105U) == Request::On);

    // Stop takes priority immediately, including when the driver is offline.
    assert(fsm.Update(false, 0xFFU, 106U) == Request::Off);
    assert(fsm.Update(false, 0U, 205U) == Request::None);
    assert(fsm.Update(false, 1U, 206U) == Request::Off);
    assert(fsm.Update(true, 1U, 207U) == Request::None);
    assert(fsm.Update(false, 1U, 208U) == Request::Off);
    assert(fsm.Update(true, 0U, 209U) == Request::On);

    MotorRecoveryFSM wrapping;
    assert(wrapping.Update(false, 0U, 0xFFFFFFF0U) == Request::Off);
    assert(wrapping.Update(false, 0U, 0x53U) == Request::None);
    assert(wrapping.Update(false, 0U, 0x54U) == Request::Off);
    return 0;
}
