#include "../fsm/chassis_keyboard_fsm.hpp"
#include <assert.h>

namespace
{
void establish_idle(ChassisKeyboardFSM &fsm)
{
    fsm.Init();
    fsm.Update(0U, true, true, 0U);
    fsm.Update(0U, true, true, ChassisKeyboardFSM::KEY_DEBOUNCE_MS);
}

void test_v_rising_edge_is_one_shot_and_retriggers_after_release()
{
    ChassisKeyboardFSM fsm;
    establish_idle(fsm);

    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 21U);
    assert(!fsm.GetCommand().imu_mode_request);
    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 41U);
    assert(fsm.GetCommand().imu_mode_request);

    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 42U);
    assert(!fsm.GetCommand().imu_mode_request);
    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 62U);
    assert(!fsm.GetCommand().imu_mode_request);

    fsm.Update(0U, true, true, 63U);
    fsm.Update(0U, true, true, 83U);
    assert(!fsm.GetCommand().imu_mode_request);

    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 84U);
    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 104U);
    assert(fsm.GetCommand().imu_mode_request);
}

void test_v_event_is_cleared_when_online_or_double_middle_gate_is_lost()
{
    ChassisKeyboardFSM fsm;
    establish_idle(fsm);

    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 21U);
    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 41U);
    assert(fsm.GetCommand().valid);
    assert(fsm.GetCommand().imu_mode_request);

    fsm.Update(ChassisKeyboardFSM::KEY_V, true, false, 42U);
    assert(!fsm.GetCommand().valid);
    assert(!fsm.GetCommand().imu_mode_request);

    establish_idle(fsm);
    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 21U);
    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 41U);
    assert(fsm.GetCommand().valid);
    assert(fsm.GetCommand().imu_mode_request);

    fsm.Update(ChassisKeyboardFSM::KEY_V, false, true, 42U);
    assert(!fsm.GetCommand().valid);
    assert(!fsm.GetCommand().imu_mode_request);

    // Re-entering the enabled mode with V already held must synchronize,
    // rather than interpret the held key as a new press.
    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 100U);
    assert(!fsm.GetCommand().imu_mode_request);
    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 120U);
    assert(!fsm.GetCommand().imu_mode_request);
}

void test_existing_keyboard_command_aggregate_order_is_preserved()
{
    const KeyboardMotionCommand legacy_command =
        {1.0f, -2.0f, true, false, true, false, true};
    assert(legacy_command.valid);
    assert(!legacy_command.imu_mode_request);
}

void test_b_and_v_edges_are_independent()
{
    ChassisKeyboardFSM fsm;
    establish_idle(fsm);

    fsm.Update(ChassisKeyboardFSM::KEY_B, true, true, 21U);
    fsm.Update(ChassisKeyboardFSM::KEY_B, true, true, 41U);
    assert(fsm.GetCommand().stair_toggle);
    assert(!fsm.GetCommand().imu_mode_request);

    fsm.Update(0U, true, true, 42U);
    fsm.Update(0U, true, true, 62U);

    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 63U);
    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 83U);
    assert(!fsm.GetCommand().stair_toggle);
    assert(fsm.GetCommand().imu_mode_request);
}
}

void test_c_is_debounced_one_shot_and_synchronized_on_entry()
{
    ChassisKeyboardFSM fsm;
    establish_idle(fsm);
    fsm.Update(ChassisKeyboardFSM::KEY_C, true, true, 21U);
    assert(!fsm.GetCommand().normal_mode_request);
    fsm.Update(ChassisKeyboardFSM::KEY_C, true, true, 41U);
    assert(fsm.GetCommand().normal_mode_request);
    fsm.Update(ChassisKeyboardFSM::KEY_C, true, true, 42U);
    assert(!fsm.GetCommand().normal_mode_request);
    fsm.Update(ChassisKeyboardFSM::KEY_C, true, false, 43U);
    assert(!fsm.GetCommand().normal_mode_request);
    fsm.Update(ChassisKeyboardFSM::KEY_C, true, true, 100U);
    fsm.Update(ChassisKeyboardFSM::KEY_C, true, true, 120U);
    assert(!fsm.GetCommand().normal_mode_request);
    fsm.Update(0U, true, true, 121U);
    fsm.Update(0U, true, true, 141U);
    fsm.Update(ChassisKeyboardFSM::KEY_C, true, true, 142U);
    fsm.Update(ChassisKeyboardFSM::KEY_C, true, true, 162U);
    assert(fsm.GetCommand().normal_mode_request);
}

int main()
{
    test_c_is_debounced_one_shot_and_synchronized_on_entry();
    test_v_rising_edge_is_one_shot_and_retriggers_after_release();
    test_v_event_is_cleared_when_online_or_double_middle_gate_is_lost();
    test_b_and_v_edges_are_independent();
    test_existing_keyboard_command_aggregate_order_is_preserved();
    return 0;
}
