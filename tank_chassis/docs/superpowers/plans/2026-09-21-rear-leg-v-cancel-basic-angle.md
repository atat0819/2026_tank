# Rear-Leg V Cancel and Basic-Angle Fallback Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Allow a second V press to cancel rear-leg retraction immediately, and use a smooth encoder-controlled basic leg angle whenever attitude control needs an unavailable IMU.

**Architecture:** Extend the rear J6248 FSM with one `BASIC_ANGLE_CONTROL` state and a separately validated basic-angle configuration. The FSM owns state transitions and smooth target planning; the RTOS task reuses the existing left/right position-speed PID cascades through a generic position-control interface. Retraction remains independent of IMU unless a new V cancellation request arrives.

**Tech Stack:** C++11, existing `Class_FSM`, `SlopePlanning`, existing PID class, STM32 RTOS task loop, host-side `g++` tests, PowerShell source-contract tests.

---

## Execution prerequisite

The main worktree contains unrelated user changes. Before implementation, create an isolated worktree from the approved base commit with `superpowers:using-git-worktrees`. Do not edit or clean the user's current modifications in:

- `tank_chassis/RtosTask/up_stair.cpp`
- `tank_chassis/RtosTask/up_stair.hpp`
- `tank_chassis/fsm/stair_mode_policy.hpp`
- `tank_chassis/fsm/up_stair_fsm.hpp`
- `tank_chassis/tests/fdcan1_dm_routing_test.ps1`
- `tank_chassis/user/core/BSP/Motor/DM/DmMotor.hpp`
- `tank_gimbal/RtosTask/can_send_task.cpp`
- `tank_gimbal/济瞄通信(2).md`

When integrating the completed branch, preserve those user changes and resolve only genuine overlaps.

### Task 1: Specify cancellation and basic-angle behavior with failing FSM tests

**Files:**

- Modify: `tank_chassis/tests/up_stair_behind_motor_fsm_test.cpp`
- Reference: `tank_chassis/fsm/up_stair_behind_motor_fsm.hpp`

- [ ] **Step 1: Extend the host test configuration with calibrated test-only basic angles**

Add finite, in-range values to `valid_config()`:

```cpp
config.basic_target_rad[0] = deg(110.0f);
config.basic_target_rad[1] = deg(330.0f);
config.basic_speed_rad_s = deg(60.0f);
config.basic_position_tolerance_rad = deg(2.0f);
```

These values are test fixtures only. Production values remain zero until physical calibration.

- [ ] **Step 2: Add a failing test for IMU loss during attitude hold**

Add assertions that enter `ATTITUDE_HOLD`, invalidate IMU while both encoders remain valid, and expect the new basic-angle state and position-control API:

```cpp
Class_Up_Stair_Behind_Motor_FSM basic_fsm(valid_config());
enter_attitude_hold(basic_fsm, 20000U);
basic_fsm.Update(true, false, true, true,
                 nan_value, nan_value, nan_value, nan_value,
                 deg(90.0f), deg(350.0f), 20301U);
assert(basic_fsm.Get_State() ==
       UP_STAIR_BEHIND_MOTOR_BASIC_ANGLE_CONTROL);
assert(basic_fsm.Uses_Position_Control());
assert(!basic_fsm.Uses_Attitude_Control());
assert(near(basic_fsm.Get_Position_Target_Angle(1U), deg(90.0f)));
assert(near(basic_fsm.Get_Position_Target_Angle(2U), deg(350.0f)));
```

- [ ] **Step 3: Add failing tests for smooth movement and IMU recovery**

Advance the tick by 500 ms while IMU remains invalid. At 60 deg/s, each target may move by at most 30 degrees toward its configured basic angle:

```cpp
basic_fsm.Update(true, false, true, true,
                 nan_value, nan_value, nan_value, nan_value,
                 deg(90.0f), deg(350.0f), 20801U);
assert(near(basic_fsm.Get_Position_Target_Angle(1U), deg(110.0f)));
assert(near(basic_fsm.Get_Position_Target_Angle(2U), deg(330.0f)));

basic_fsm.Update(true, true, true, true,
                 4.5f, -3.5f, 12.0f, -7.0f,
                 deg(110.0f), deg(330.0f), 20802U);
assert(basic_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
assert(near(basic_fsm.Get_Output_Scale(), 0.0f));
```

- [ ] **Step 4: Replace the old “V is ignored during retracting” expectation**

Cover both cancellation paths:

```cpp
// Valid IMU: second V cancels retract and starts recovery now.
cancel_valid_fsm.Update(true, true, true, true,
                        4.5f, -3.5f, 12.0f, -7.0f,
                        deg(95.0f), deg(345.0f), cancel_tick,
                        true, 2U);
assert(cancel_valid_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);

// Invalid IMU: second V cancels retract and starts basic-angle control.
cancel_invalid_fsm.Update(true, false, true, true,
                          nan_value, nan_value, nan_value, nan_value,
                          deg(95.0f), deg(345.0f), cancel_tick,
                          true, 2U);
assert(cancel_invalid_fsm.Get_State() ==
       UP_STAIR_BEHIND_MOTOR_BASIC_ANGLE_CONTROL);
```

- [ ] **Step 5: Prove that IMU loss alone still does not stop retraction**

Use the original action sequence while IMU is invalid and assert that state remains `RETRACTING`. Only a new sequence may cancel it:

```cpp
retract_fsm.Update(true, false, true, true,
                   nan_value, nan_value, nan_value, nan_value,
                   deg(95.0f), deg(345.0f), retract_tick,
                   true, 1U);
assert(retract_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTING);
```

- [ ] **Step 6: Add failing validation and safety cases**

Add cases for zero speed, zero tolerance, target outside the mechanical interval, non-finite target, wrapped safe interval, and one invalid J6248 feedback during basic-angle control. The invalid-feedback case must assert `DISABLED` and zero torque permission for both sides.

- [ ] **Step 7: Run the rear FSM test and verify RED**

Run from `tank_chassis`:

```powershell
g++ -std=c++11 -I. tests/up_stair_behind_motor_fsm_test.cpp fsm/up_stair_behind_motor_fsm.cpp user/core/Alg/FSM/alg_fsm.cpp user/core/Alg/UtilityFunction/SlopPlanning.cpp -o tests/build/up_stair_behind_motor_fsm_test.exe
& .\tests\build\up_stair_behind_motor_fsm_test.exe
```

Expected: compilation fails because the new enum, configuration fields, and generic position-control getters do not exist. This is the required TDD red result.

### Task 2: Implement the rear FSM basic-angle state and V cancellation

**Files:**

- Modify: `tank_chassis/fsm/up_stair_behind_motor_fsm.hpp`
- Modify: `tank_chassis/fsm/up_stair_behind_motor_fsm.cpp`
- Test: `tank_chassis/tests/up_stair_behind_motor_fsm_test.cpp`

- [ ] **Step 1: Add the new state, configuration, and public interface**

Extend the enum and `Config`:

```cpp
UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD,
UP_STAIR_BEHIND_MOTOR_BASIC_ANGLE_CONTROL,
UP_STAIR_BEHIND_MOTOR_COUNT
```

```cpp
float basic_target_rad[2];
float basic_speed_rad_s;
float basic_position_tolerance_rad;
```

Add generic position-control accessors while preserving existing retract-specific methods:

```cpp
bool Uses_Position_Control() const;
float Get_Position_Target_Angle(uint8_t id) const;
```

- [ ] **Step 2: Add safe default construction and separate validation**

Initialize all new fields to zero in `Config::Config()`:

```cpp
basic_target_rad{0.0f, 0.0f},
basic_speed_rad_s(0.0f),
basic_position_tolerance_rad(0.0f)
```

Add `basic_config_valid_` and `Validate_Basic_Config()`. Validation must require the base mechanical config, finite in-range targets, positive speed, and positive tolerance. Use a helper parallel to `To_Unwrapped_Retract_Target()`:

```cpp
float To_Unwrapped_Basic_Target(uint8_t index) const;
```

For a wrapped interval, add `TWO_PI_RAD` when the raw basic target lies below `angle_start_rad[index]`.

- [ ] **Step 3: Add planner storage and reset behavior**

Add dedicated members so retract planning and basic-angle planning cannot overwrite one another:

```cpp
float basic_target_angle_rad_[2];
Alg::Utility::SlopePlanning basic_planner_[2];
uint32_t basic_last_tick_;
```

Initialize and reset both planners to zero. `Disable()` must clear planner timing and output permission without changing the immutable configuration-validity flags.

- [ ] **Step 4: Implement basic-angle start and update helpers**

Add:

```cpp
void Start_Basic_Angle_Control(uint32_t now_tick);
void Update_Basic_Targets(uint32_t now_tick);
```

`Start_Basic_Angle_Control()` must:

```cpp
Set_Status(UP_STAIR_BEHIND_MOTOR_BASIC_ANGLE_CONTROL);
output_scale_ = 1.0f;
basic_last_tick_ = now_tick;
for (uint8_t index = 0U; index < 2U; ++index)
{
    basic_planner_[index].SetNowReal(position_feedback_rad_[index]);
    basic_planner_[index].SetIncreaseValue(0.0f);
    basic_planner_[index].SetDecreaseValue(0.0f);
    basic_planner_[index].SetTarget(To_Unwrapped_Basic_Target(index));
    basic_target_angle_rad_[index] = position_feedback_rad_[index];
}
```

`Update_Basic_Targets()` computes `step = basic_speed_rad_s * delta_tick / 1000.0f`, applies it as both increase and decrease limits, runs each planner, and stores its output.

- [ ] **Step 5: Restructure `Update()` so encoder feedback is processed before IMU fallback**

Compute these facts independently:

```cpp
const bool imu_ready =
    imu_valid && std::isfinite(pitch_deg) && std::isfinite(roll_deg) &&
    std::isfinite(pitch_rate_dps) && std::isfinite(roll_rate_dps);
const bool retract_state =
    Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTING ||
    Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD;
```

Do not call `Disable()` merely because IMU is invalid before encoder angles have been validated. Position modes need valid motor feedback but do not need finite attitude values.

- [ ] **Step 6: Implement exact state transition order**

Apply the following order after control/config and encoder checks:

```cpp
if (retract_state)
{
    if (new_retract_action)
    {
        last_action_sequence_ = retract_action_sequence;
        if (imu_ready)
        {
            Start_Recovery(now_tick);
        }
        else if (basic_config_valid_)
        {
            Start_Basic_Angle_Control(now_tick);
        }
        else
        {
            Disable();
        }
        return;
    }
    // Existing retract planning or retract hold continues here.
}

if (!imu_ready)
{
    if (basic_config_valid_ && both_feedback_valid)
    {
        if (Get_State() != UP_STAIR_BEHIND_MOTOR_BASIC_ANGLE_CONTROL)
        {
            Start_Basic_Angle_Control(now_tick);
        }
        else
        {
            Update_Basic_Targets(now_tick);
        }
    }
    else
    {
        Disable();
    }
    return;
}

if (Get_State() == UP_STAIR_BEHIND_MOTOR_BASIC_ANGLE_CONTROL)
{
    Start_Recovery(now_tick);
    return;
}
```

In `BASIC_ANGLE_CONTROL`, consume new V sequence values without starting a retract action, so no event is replayed after IMU recovery.

- [ ] **Step 7: Implement generic position-control getters and torque-state permission**

```cpp
bool Class_Up_Stair_Behind_Motor_FSM::Uses_Position_Control() const
{
    return Uses_Retract_Position_Control() ||
           Get_State() == UP_STAIR_BEHIND_MOTOR_BASIC_ANGLE_CONTROL;
}

float Class_Up_Stair_Behind_Motor_FSM::Get_Position_Target_Angle(
    uint8_t id) const
{
    const uint8_t index = To_Index(id);
    if (index > 1U)
    {
        return 0.0f;
    }
    return Get_State() == UP_STAIR_BEHIND_MOTOR_BASIC_ANGLE_CONTROL
               ? basic_target_angle_rad_[index]
               : retract_target_angle_rad_[index];
}
```

Allow `Limit_Torque()` in `BASIC_ANGLE_CONTROL`, while preserving finite torque checks and mechanical direction limits.

- [ ] **Step 8: Run the rear FSM test and verify GREEN**

Run the same compile and executable command from Task 1. Expected: exit code `0` with no failed assertions.

- [ ] **Step 9: Commit the FSM behavior**

```powershell
git add tank_chassis/fsm/up_stair_behind_motor_fsm.hpp tank_chassis/fsm/up_stair_behind_motor_fsm.cpp tank_chassis/tests/up_stair_behind_motor_fsm_test.cpp
git commit -m "feat: add rear basic-angle fallback control"
```

### Task 3: Specify task-loop integration with a failing contract test

**Files:**

- Modify: `tank_chassis/tests/up_stair_integration_contract_test.ps1`
- Reference: `tank_chassis/RtosTask/up_stair.cpp`

- [ ] **Step 1: Add source-contract assertions for safe production configuration**

Require explicit assignments:

```powershell
Assert-Contains $source 'config.basic_target_rad[0] = 0.0f;'
Assert-Contains $source 'config.basic_target_rad[1] = 0.0f;'
Assert-Contains $source 'config.basic_speed_rad_s = 0.0f;'
Assert-Contains $source 'config.basic_position_tolerance_rad = 0.0f;'
```

- [ ] **Step 2: Require generic position-control integration**

Require the task to use:

```powershell
Assert-Contains $source 'Uses_Position_Control()'
Assert-Contains $source 'Get_Position_Target_Angle(1U)'
Assert-Contains $source 'Get_Position_Target_Angle(2U)'
```

Retain existing assertions for velocity feedback, left/right position-speed PIDs, torque limiting, and attitude/position PID resets.

- [ ] **Step 3: Run the contract and verify RED**

Run from `tank_chassis`:

```powershell
powershell -ExecutionPolicy Bypass -File tests/up_stair_integration_contract_test.ps1
```

Expected: FAIL because the four basic configuration assignments and generic position-control method calls do not exist.

### Task 4: Integrate basic-angle position control into the RTOS task

**Files:**

- Modify: `tank_chassis/RtosTask/up_stair.cpp`
- Test: `tank_chassis/tests/up_stair_integration_contract_test.ps1`

- [ ] **Step 1: Add explicit zero-valued production defaults**

In `BuildRear6248Config()` add:

```cpp
config.basic_target_rad[0] = 0.0f;
config.basic_target_rad[1] = 0.0f;
config.basic_speed_rad_s = 0.0f;
config.basic_position_tolerance_rad = 0.0f;
```

Do not insert guessed angles or speed values.

- [ ] **Step 2: Generalize the existing rear position-control branch**

Replace the retract-specific condition and targets:

```cpp
else if (up_stair_behind_motor_fsm.Uses_Position_Control())
{
    rear_6248_pitch_pid[0].reset();
    rear_6248_pitch_pid[1].reset();
    rear_6248_roll_pid[0].reset();
    rear_6248_roll_pid[1].reset();

    const float left_target_velocity =
        rear_6248_left_retract_pid[0].UpDate(
            up_stair_behind_motor_fsm.Get_Position_Target_Angle(1U),
            up_stair_behind_motor_fsm.Get_Position_Feedback(1U));
    const float left_position_torque =
        rear_6248_left_retract_pid[1].UpDate(
            left_target_velocity, rear_left_velocity);

    const float right_target_velocity =
        rear_6248_right_retract_pid[0].UpDate(
            up_stair_behind_motor_fsm.Get_Position_Target_Angle(2U),
            up_stair_behind_motor_fsm.Get_Position_Feedback(2U));
    const float right_position_torque =
        rear_6248_right_retract_pid[1].UpDate(
            right_target_velocity, rear_right_velocity);
```

Send both torques through `Limit_Torque()` and `ClampJ6248Torque()` exactly as the existing retract branch does. Do not multiply either position torque by `Get_Motor_Direction()`.

- [ ] **Step 3: Preserve mutually exclusive PID resets**

The position branch resets pitch/roll PIDs. The attitude branch resets both left/right position PID cascades. Disabled and catch-all branches reset both families and send zero torque.

- [ ] **Step 4: Run the integration contract and verify GREEN**

```powershell
powershell -ExecutionPolicy Bypass -File tests/up_stair_integration_contract_test.ps1
```

Expected output:

```text
up stair integration contract passed
```

- [ ] **Step 5: Re-run the rear FSM host test**

Use the Task 1 compile/run command. Expected: exit code `0`.

- [ ] **Step 6: Commit task integration**

```powershell
git add tank_chassis/RtosTask/up_stair.cpp tank_chassis/tests/up_stair_integration_contract_test.ps1
git commit -m "feat: use basic rear angle when imu is unavailable"
```

### Task 5: Regression verification and handoff

**Files:**

- Verify: `tank_chassis/tests/chassis_keyboard_fsm_test.cpp`
- Verify: `tank_chassis/tests/stair_mode_policy_test.cpp`
- Verify: `tank_chassis/tests/up_stair_fsm_test.cpp`
- Verify: `tank_chassis/tests/up_stair_behind_motor_fsm_test.cpp`
- Verify: `tank_chassis/tests/motor_recovery_fsm_test.cpp`
- Verify: `tank_chassis/tests/rear_torque_safety_test.cpp`
- Verify: `tank_chassis/tests/*.ps1`

- [ ] **Step 1: Run all relevant host tests**

From `tank_chassis`, compile and run the keyboard FSM, stair policy, front stair FSM, rear stair FSM, motor recovery, and rear torque safety tests with their existing source dependencies. Every executable must return exit code `0`.

- [ ] **Step 2: Run all PowerShell contracts**

```powershell
Get-ChildItem tests -Filter '*.ps1' | ForEach-Object {
    & $_.FullName
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
```

Expected: every contract exits `0`, including `up stair integration contract passed`.

- [ ] **Step 3: Check the final diff**

```powershell
git diff --check
git status --short
git diff --stat
```

Expected: no whitespace errors; only the approved FSM, task, tests, design, and plan files are present in the feature branch diff. Generated executables must not be committed.

- [ ] **Step 4: Document physical calibration requirements in the handoff**

Report that these production values intentionally remain zero and must be measured before enabling the behavior:

```cpp
basic_target_rad[0]
basic_target_rad[1]
basic_speed_rad_s
basic_position_tolerance_rad
```

Also report that full STM32/HAL firmware compilation and physical robot testing remain separate from the host tests.
