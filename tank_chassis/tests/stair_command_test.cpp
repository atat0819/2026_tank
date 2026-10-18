#include "fsm/stair_mode_policy.hpp"
#include <cassert>

int main()
{
    StairCommand command;
    assert(command.mode == StairMode::NORMAL);
    UpdateStairCommand(command, true, false, false, true);
    assert(!command.front_target_requested && !command.rear_retract_requested);

    UpdateStairCommand(command, true, true, false, false);
    assert(command.mode == StairMode::IMU);
    assert(!command.front_target_requested && !command.rear_retract_requested);
    UpdateStairCommand(command, true, false, false, true);
    assert(command.front_target_requested && !command.rear_retract_requested);
    UpdateStairCommand(command, true, false, false, false);
    assert(command.front_target_requested && !command.rear_retract_requested);
    UpdateStairCommand(command, true, false, false, true);
    assert(!command.front_target_requested && command.rear_retract_requested);
    UpdateStairCommand(command, true, false, false, true);
    assert(command.front_target_requested && !command.rear_retract_requested);

    // V restarts home/leveling even in the middle of a B action.
    UpdateStairCommand(command, true, true, false, true);
    assert(command.mode == StairMode::IMU);
    assert(!command.front_target_requested && !command.rear_retract_requested);
    UpdateStairCommand(command, true, true, true, true);
    assert(command.mode == StairMode::NORMAL);
    assert(!command.front_target_requested && !command.rear_retract_requested);

    UpdateStairCommand(command, true, true, false, false);
    UpdateStairCommand(command, true, false, false, true);
    UpdateStairCommand(command, false, true, false, true);
    assert(command.mode == StairMode::NORMAL);
    assert(!command.front_target_requested && !command.rear_retract_requested);
    UpdateStairCommand(command, true, false, false, true);
    assert(command.mode == StairMode::NORMAL);
}
