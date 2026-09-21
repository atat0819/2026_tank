# Rear Leg V-Toggle Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a debounced V-key toggle that retracts both rear J6248 legs to calibrated encoder angles without IMU input, holds them there, and restores the existing IMU attitude controller on the next V press.

**Architecture:** Keep keyboard edge detection and switch policy pure, and pass V presses to the rear FSM through a monotonic action sequence. Extend the rear FSM with `RETRACTING` and `RETRACTED_HOLD`; it owns state transitions, target ramps, feedback validity, and mode selection, while `up_stair_task` owns the position-speed PID calculations and MIT output. Attitude and retract controllers are mutually exclusive, and the second V press enters the existing 300 ms attitude recovery ramp without commanding a normal leg angle.

**Tech Stack:** C++11, STM32H7 HAL, FreeRTOS/CMSIS-RTOS, DM MIT motor control, existing PID and `SlopePlanning`, host-side `g++` tests, PowerShell integration contracts.

---

## File map

- `fsm/chassis_keyboard_fsm.hpp/.cpp`: define V-key mapping and emit one-cycle `rear_retract_toggle` events.
- `fsm/stair_mode_policy.hpp`: expose whether V commands are authorized in the current switch/keyboard mode.
- `fsm/up_stair_behind_motor_fsm.hpp/.cpp`: own rear retract states, target planning, event consumption, IMU-independent retract safety, and attitude resume requests.
- `RtosTask/up_stair.hpp`: expose the monotonic V action sequence shared by keyboard and stair tasks.
- `RtosTask/can_send_task.cpp`: increment the V action sequence on each debounced V press.
- `RtosTask/up_stair.cpp`: snapshot the V sequence, run either attitude PID or rear position-speed PID, and send mutually exclusive torque commands.
- `tests/chassis_keyboard_fsm_test.cpp`: verify V debounce and one-shot behavior.
- `tests/stair_mode_policy_test.cpp`: verify V authorization policy.
- `tests/up_stair_behind_motor_fsm_test.cpp`: verify retract transitions, target ramping, event consumption, IMU independence, and failure behavior.
- `tests/up_stair_integration_contract_test.ps1`: verify task-level wiring and controller exclusivity.

The Keil project already contains every modified production source, so no `.uvprojx` file change is required.

### Task 1: Emit a debounced one-shot V command

**Files:**
- Create: `tests/chassis_keyboard_fsm_test.cpp`
- Modify: `fsm/chassis_keyboard_fsm.hpp:6-61`
- Modify: `fsm/chassis_keyboard_fsm.cpp:25-160`

- [ ] **Step 1: Write the failing keyboard test**

Create `tests/chassis_keyboard_fsm_test.cpp`:

```cpp
#include "../fsm/chassis_keyboard_fsm.hpp"
#include <assert.h>

namespace {
void establish_idle(ChassisKeyboardFSM &fsm)
{
    fsm.Init();
    fsm.Update(0U, true, true, 0U);
    fsm.Update(0U, true, true,
               ChassisKeyboardFSM::KEY_DEBOUNCE_MS);
}
}

int main()
{
    ChassisKeyboardFSM fsm;
    establish_idle(fsm);
    assert(!fsm.GetCommand().rear_retract_toggle);

    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 21U);
    assert(!fsm.GetCommand().rear_retract_toggle);
    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 41U);
    assert(fsm.GetCommand().rear_retract_toggle);

    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 42U);
    assert(!fsm.GetCommand().rear_retract_toggle);
    fsm.Update(0U, true, true, 43U);
    fsm.Update(0U, true, true, 63U);
    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 64U);
    fsm.Update(ChassisKeyboardFSM::KEY_V, true, true, 84U);
    assert(fsm.GetCommand().rear_retract_toggle);

    fsm.Update(0U, false, false, 85U);
    assert(!fsm.GetCommand().rear_retract_toggle);
    assert(!fsm.GetCommand().valid);
    return 0;
}
```

- [ ] **Step 2: Compile the test and verify that it fails**

Run:

```powershell
New-Item -ItemType Directory -Force tests/build | Out-Null
g++ -std=c++11 -I. tests/chassis_keyboard_fsm_test.cpp fsm/chassis_keyboard_fsm.cpp -o tests/build/chassis_keyboard_fsm_test.exe
```

Expected: compilation fails because `KEY_V` and `rear_retract_toggle` do not exist.

- [ ] **Step 3: Add V to the keyboard command and edge detector**

In `fsm/chassis_keyboard_fsm.hpp`, add the command member, key bit, and edge state:

```cpp
struct KeyboardMotionCommand
{
    float vx;
    float vy;
    bool shift_pressed;
    bool gyro_enabled;
    bool follow_enabled;
    bool stair_toggle;
    bool rear_retract_toggle;
    bool valid;
};

enum KeyMask : uint16_t
{
    KEY_W     = (1U << 0),
    KEY_S     = (1U << 1),
    KEY_A     = (1U << 2),
    KEY_D     = (1U << 3),
    KEY_SHIFT = (1U << 4),
    KEY_CTRL  = (1U << 5),
    KEY_Z     = (1U << 11),
    KEY_V     = (1U << 14),
    KEY_B     = (1U << 15)
};

bool last_v_pressed_ = false;
bool last_b_pressed_ = false;
```

In `Reset()`, clear `last_v_pressed_`. At the beginning of every valid `Update()`, clear both one-cycle commands:

```cpp
command_.valid = true;
command_.stair_toggle = false;
command_.rear_retract_toggle = false;
```

When establishing the initial stable mask, synchronize V without emitting an event:

```cpp
last_v_pressed_ = (stable_key_mask_ & KEY_V) != 0U;
last_b_pressed_ = (stable_key_mask_ & KEY_B) != 0U;
```

Immediately before the existing B-edge block, add:

```cpp
const bool v_pressed = (stable_key_mask_ & KEY_V) != 0U;
if (v_pressed && !last_v_pressed_)
{
    command_.rear_retract_toggle = true;
}
last_v_pressed_ = v_pressed;
```

- [ ] **Step 4: Run the keyboard test**

Run:

```powershell
g++ -std=c++11 -I. tests/chassis_keyboard_fsm_test.cpp fsm/chassis_keyboard_fsm.cpp -o tests/build/chassis_keyboard_fsm_test.exe
& .\tests\build\chassis_keyboard_fsm_test.exe
```

Expected: process exits with code 0 and prints no assertion failure.

- [ ] **Step 5: Commit the keyboard event**

```powershell
git add fsm/chassis_keyboard_fsm.hpp fsm/chassis_keyboard_fsm.cpp tests/chassis_keyboard_fsm_test.cpp
git commit -m "feat: emit rear leg toggle from V key"
```

### Task 2: Authorize V only in live double-middle keyboard mode

**Files:**
- Modify: `fsm/stair_mode_policy.hpp:10-31`
- Modify: `tests/stair_mode_policy_test.cpp:8-97`

- [ ] **Step 1: Extend the policy tests first**

Add `rear_retract_command_enabled` assertions everywhere the front command is checked:

```cpp
assert(policy.front_stair_command_enabled == expected_front_command);
assert(policy.rear_retract_command_enabled == expected_front_command);
```

For double-down, stale control link, stale keyboard, and invalid switches, assert:

```cpp
assert(!policy.rear_retract_command_enabled);
```

In `test_all_valid_switch_pairs()`, use:

```cpp
assert(policy.rear_retract_command_enabled == is_double_middle);
```

- [ ] **Step 2: Run the policy test and verify that it fails**

Run:

```powershell
g++ -std=c++11 -I. tests/stair_mode_policy_test.cpp -o tests/build/stair_mode_policy_test.exe
```

Expected: compilation fails because `rear_retract_command_enabled` is absent.

- [ ] **Step 3: Add the explicit rear-command policy field**

Change `StairModePolicy` to:

```cpp
struct StairModePolicy {
  bool zero_all_torque;
  bool control_fault;
  bool front_hold_enabled;
  bool rear_attitude_enabled;
  bool front_stair_command_enabled;
  bool rear_retract_command_enabled;
};
```

Return six fields from every branch:

```cpp
if (control_fault) {
  return {true, true, false, false, false, false};
}
if (s1 == DOWN && s2 == DOWN) {
  return {true, false, false, false, false, false};
}

const bool keyboard_command_enabled =
    s1 == MIDDLE && s2 == MIDDLE && keyboard_online;
return {false, false, true, true,
        keyboard_command_enabled, keyboard_command_enabled};
```

- [ ] **Step 4: Run the policy test**

Run:

```powershell
g++ -std=c++11 -I. tests/stair_mode_policy_test.cpp -o tests/build/stair_mode_policy_test.exe
& .\tests\build\stair_mode_policy_test.exe
```

Expected: process exits with code 0.

- [ ] **Step 5: Commit the policy change**

```powershell
git add fsm/stair_mode_policy.hpp tests/stair_mode_policy_test.cpp
git commit -m "feat: gate rear leg toggle by stair mode"
```

### Task 3: Extend the rear FSM with encoder-only retract states

**Files:**
- Modify: `fsm/up_stair_behind_motor_fsm.hpp:8-92`
- Modify: `fsm/up_stair_behind_motor_fsm.cpp:20-434`
- Modify: `tests/up_stair_behind_motor_fsm_test.cpp:20-324`

- [ ] **Step 1: Add failing configuration and transition tests**

Extend `valid_config()` with host-test calibration values:

```cpp
config.retract_target_rad[0] = deg(140.0f);
config.retract_target_rad[1] = deg(20.0f);
config.retract_speed_rad_s = deg(90.0f);
config.retract_position_tolerance_rad = deg(2.0f);
```

Append a focused test block before `return 0`:

```cpp
Class_Up_Stair_Behind_Motor_FSM retract_fsm(valid_config());
update_valid(retract_fsm, 10000U);
update_valid(retract_fsm, 10300U);
assert(retract_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD);

retract_fsm.Update(true, true, true, true,
                   4.5f, -3.5f, 12.0f, -7.0f,
                   deg(90.0f), deg(350.0f), 10301U,
                   true, 1U);
assert(retract_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTING);
assert(retract_fsm.Uses_Retract_Position_Control());
assert(!retract_fsm.Uses_Attitude_Control());

// Stale/nonfinite IMU is ignored while encoder-only retract control is active.
retract_fsm.Update(true, false, true, true,
                   nan_value, nan_value, nan_value, nan_value,
                   deg(100.0f), deg(340.0f), 10401U,
                   true, 1U);
assert(retract_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTING);

// A V event during motion is consumed without reversing the state.
retract_fsm.Update(true, false, true, true,
                   nan_value, nan_value, nan_value, nan_value,
                   deg(110.0f), deg(330.0f), 10501U,
                   true, 2U);
assert(retract_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTING);

// Both encoders at the calibrated targets complete the motion.
retract_fsm.Update(true, false, true, true,
                   nan_value, nan_value, nan_value, nan_value,
                   deg(140.0f), deg(20.0f), 11000U,
                   true, 2U);
assert(retract_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD);

// The next V is the resume request. Invalid IMU keeps position hold active.
retract_fsm.Update(true, false, true, true,
                   nan_value, nan_value, nan_value, nan_value,
                   deg(140.0f), deg(20.0f), 11001U,
                   true, 3U);
assert(retract_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD);

// Fresh IMU completes the pending request through the existing recovery ramp.
retract_fsm.Update(true, true, true, true,
                   4.5f, -3.5f, 12.0f, -7.0f,
                   deg(140.0f), deg(20.0f), 11002U,
                   true, 3U);
assert(retract_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING);
assert(near(retract_fsm.Get_Output_Scale(), 0.0f));

Class_Up_Stair_Behind_Motor_FSM retract_fault_fsm(valid_config());
update_valid(retract_fault_fsm, 12000U);
update_valid(retract_fault_fsm, 12300U);
retract_fault_fsm.Update(true, true, true, true,
                         4.5f, -3.5f, 12.0f, -7.0f,
                         deg(90.0f), deg(350.0f), 12301U,
                         true, 1U);
retract_fault_fsm.Update(true, false, false, true,
                         nan_value, nan_value, nan_value, nan_value,
                         deg(90.0f), deg(350.0f), 12302U,
                         true, 1U);
assert(retract_fault_fsm.Get_State() == UP_STAIR_BEHIND_MOTOR_DISABLED);
assert(near(retract_fault_fsm.Limit_Torque(1U, 2.0f), 0.0f));
assert(near(retract_fault_fsm.Limit_Torque(2U, 2.0f), 0.0f));
```

Add a validation case proving an out-of-range retract target invalidates the whole rear configuration:

```cpp
bad = valid_config();
bad.retract_target_rad[0] = deg(200.0f);
Class_Up_Stair_Behind_Motor_FSM bad_retract_target(bad);
assert(!bad_retract_target.Is_Config_Valid());
```

- [ ] **Step 2: Compile and verify the new test fails**

Run:

```powershell
g++ -std=c++11 -I. tests/up_stair_behind_motor_fsm_test.cpp fsm/up_stair_behind_motor_fsm.cpp user/core/Alg/FSM/alg_fsm.cpp user/core/Alg/UtilityFunction/SlopPlanning.cpp -o tests/build/up_stair_behind_motor_fsm_test.exe
```

Expected: compilation fails because the retract states, config fields, getters, and extended `Update` arguments do not exist.

- [ ] **Step 3: Add the rear FSM public contract**

Extend the state enum:

```cpp
enum Enum_Up_Stair_Behind_Motor_Status
{
    UP_STAIR_BEHIND_MOTOR_DISABLED = 0,
    UP_STAIR_BEHIND_MOTOR_RECOVERING,
    UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD,
    UP_STAIR_BEHIND_MOTOR_RETRACTING,
    UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD,
    UP_STAIR_BEHIND_MOTOR_COUNT
};
```

Extend `Config`:

```cpp
float retract_target_rad[2];
float retract_speed_rad_s;
float retract_position_tolerance_rad;
```

Append optional command arguments to preserve all existing call sites while the task integration is staged:

```cpp
void Update(bool control_enabled,
            bool imu_valid,
            bool left_feedback_valid,
            bool right_feedback_valid,
            float pitch_deg,
            float roll_deg,
            float pitch_rate_dps,
            float roll_rate_dps,
            float left_angle_rad,
            float right_angle_rad,
            uint32_t now_tick,
            bool retract_command_enabled = false,
            uint32_t retract_action_sequence = 0U);

bool Uses_Attitude_Control() const;
bool Uses_Retract_Position_Control() const;
float Get_Retract_Target_Angle(uint8_t id) const;
float Get_Position_Feedback(uint8_t id) const;
```

Add private members and helpers:

```cpp
void Start_Retraction(uint32_t now_tick);
void Update_Retraction_Target(uint32_t now_tick);
bool Both_At_Retract_Target() const;
bool Is_Configured_Angle_Valid(uint8_t index, float raw_angle_rad) const;

Alg::Utility::SlopePlanning retract_planner_[2];
float control_angle_rad_[2];
float retract_target_control_rad_[2];
float planned_retract_target_rad_[2];
uint32_t retract_last_tick_;
uint32_t last_retract_action_sequence_;
bool attitude_resume_pending_;
```

- [ ] **Step 4: Implement configuration validation and target planning**

Initialize new `Config` fields to fail-safe values:

```cpp
retract_target_rad{0.0f, 0.0f},
retract_speed_rad_s(0.0f),
retract_position_tolerance_rad(0.0f)
```

Initialize both planners in each constructor with:

```cpp
retract_planner_{
    Alg::Utility::SlopePlanning(0.0f, 0.0f),
    Alg::Utility::SlopePlanning(0.0f, 0.0f)},
control_angle_rad_{0.0f, 0.0f},
retract_target_control_rad_{0.0f, 0.0f},
planned_retract_target_rad_{0.0f, 0.0f},
retract_last_tick_(0U),
last_retract_action_sequence_(0U),
attitude_resume_pending_(false)
```

Clear every new scalar/array member and reset both planners in `Reset()` and `Disable()`.

`Validate_Config()` must additionally require:

```cpp
std::isfinite(config_.retract_target_rad[index]) &&
Is_Configured_Angle_Valid(index, config_.retract_target_rad[index]) &&
std::isfinite(config_.retract_speed_rad_s) &&
config_.retract_speed_rad_s > 0.0f &&
std::isfinite(config_.retract_position_tolerance_rad) &&
config_.retract_position_tolerance_rad > 0.0f
```

Implement `Is_Configured_Angle_Valid()` with the same normal/wrapped interval rules as `Is_Angle_Valid()`, but without checking `config_valid_`. Make public `Is_Angle_Valid()` call it after validating ID, finite raw domain, and `config_valid_`.

On `Start_Retraction()`:

```cpp
Set_Status(UP_STAIR_BEHIND_MOTOR_RETRACTING);
output_scale_ = 1.0f;
retract_last_tick_ = now_tick;
attitude_resume_pending_ = false;
for (uint8_t i = 0U; i < 2U; ++i)
{
    retract_target_control_rad_[i] =
        To_Unwrapped_Angle(i, config_.retract_target_rad[i]);
    retract_planner_[i].SetNowReal(control_angle_rad_[i]);
    retract_planner_[i].SetTarget(control_angle_rad_[i]);
    retract_planner_[i].TIM_Calculate_PeriodElapsedCallback(
        control_angle_rad_[i], control_angle_rad_[i]);
    planned_retract_target_rad_[i] = control_angle_rad_[i];
}
```

On every retract update, calculate a tick-aware step and advance both planners:

```cpp
const uint32_t delta_tick = now_tick - retract_last_tick_;
retract_last_tick_ = now_tick;
const float step = config_.retract_speed_rad_s *
                   static_cast<float>(delta_tick) * 0.001f;
for (uint8_t i = 0U; i < 2U; ++i)
{
    retract_planner_[i].SetIncreaseValue(step);
    retract_planner_[i].SetDecreaseValue(step);
    retract_planner_[i].TIM_Calculate_PeriodElapsedCallback(
        retract_target_control_rad_[i], control_angle_rad_[i]);
    planned_retract_target_rad_[i] = retract_planner_[i].GetOut();
}
```

`Both_At_Retract_Target()` requires both motor feedback flags and both control-angle errors within `retract_position_tolerance_rad`.

- [ ] **Step 5: Implement state-aware IMU and V-event handling**

Restructure `Update()` in this order:

```cpp
// 1. Validate/store both raw motor angles and derive unwrapped control angles.
// 2. If control/config is invalid, Disable() and consume the current sequence.
// 3. In RETRACTING/RETRACTED_HOLD, require both motor feedbacks but do not
//    require IMU validity or finite IMU values.
// 4. Consume every changed sequence exactly once. Only ATTITUDE_HOLD starts
//    retraction; RETRACTING ignores it; RETRACTED_HOLD requests resume.
// 5. A resume request enters Start_Recovery(now_tick) only with a valid,
//    finite IMU sample; otherwise it remains pending in RETRACTED_HOLD.
// 6. Outside retract states, preserve existing attitude validation,
//    per-motor degradation behavior, and recovery behavior.
```

Use these mode getters:

```cpp
bool Class_Up_Stair_Behind_Motor_FSM::Uses_Attitude_Control() const
{
    return Get_State() == UP_STAIR_BEHIND_MOTOR_RECOVERING ||
           Get_State() == UP_STAIR_BEHIND_MOTOR_ATTITUDE_HOLD;
}

bool Class_Up_Stair_Behind_Motor_FSM::Uses_Retract_Position_Control() const
{
    return Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTING ||
           Get_State() == UP_STAIR_BEHIND_MOTOR_RETRACTED_HOLD;
}
```

Allow `Limit_Torque()` in all four active states. Retraction starts with `output_scale_ == 1.0f`; the second V calls existing `Start_Recovery()`, which restarts output at zero. Mechanical margin blocking and per-motor gain remain unchanged.

- [ ] **Step 6: Run the rear FSM test**

Run:

```powershell
g++ -std=c++11 -I. tests/up_stair_behind_motor_fsm_test.cpp fsm/up_stair_behind_motor_fsm.cpp user/core/Alg/FSM/alg_fsm.cpp user/core/Alg/UtilityFunction/SlopPlanning.cpp -o tests/build/up_stair_behind_motor_fsm_test.exe
& .\tests\build\up_stair_behind_motor_fsm_test.exe
```

Expected: process exits with code 0.

- [ ] **Step 7: Commit the rear FSM**

```powershell
git add fsm/up_stair_behind_motor_fsm.hpp fsm/up_stair_behind_motor_fsm.cpp tests/up_stair_behind_motor_fsm_test.cpp
git commit -m "feat: add encoder-only rear leg retract states"
```

### Task 4: Wire the V sequence and mutually exclusive controllers into the task

**Files:**
- Modify: `RtosTask/up_stair.hpp:14-19`
- Modify: `RtosTask/can_send_task.cpp:52-58,327-356`
- Modify: `RtosTask/up_stair.cpp:27-311`
- Modify: `tests/up_stair_integration_contract_test.ps1:1-22`

- [ ] **Step 1: Strengthen the integration contract before production changes**

Append checks to `tests/up_stair_integration_contract_test.ps1`:

```powershell
$canTask = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\can_send_task.cpp')
$keyboardHeader = Get-Content -Raw (Join-Path $PSScriptRoot '..\fsm\chassis_keyboard_fsm.hpp')

if ($keyboardHeader -notmatch 'rear_retract_toggle') { throw 'keyboard command must expose V edge' }
if ($canTask -notmatch 'rear_retract_action_sequence') { throw 'CAN task must own rear retract sequence' }
if ($canTask -notmatch 'keyboard_cmd\.rear_retract_toggle[\s\S]*?\+\+rear_retract_action_sequence') { throw 'V edge must increment rear sequence' }
if ($source -notmatch 'policy\.rear_retract_command_enabled') { throw 'rear FSM must receive policy-gated V command' }
if ($source -notmatch 'Uses_Retract_Position_Control') { throw 'task must select rear position control by FSM mode' }
if ($source -notmatch 'rear_6248_left_retract_pid\[2\]') { throw 'missing left rear retract cascade' }
if ($source -notmatch 'rear_6248_right_retract_pid\[2\]') { throw 'missing right rear retract cascade' }
if ($source -notmatch 'getVelocityRads\(1\)' -or $source -notmatch 'getVelocityRads\(2\)') { throw 'rear retract speed loops require motor velocity' }
if ($source -notmatch 'if \(up_stair_behind_motor_fsm\.Uses_Retract_Position_Control\(\)\)[\s\S]*?else if \(up_stair_behind_motor_fsm\.Uses_Attitude_Control\(\)\)') { throw 'retract and attitude branches must be mutually exclusive' }
```

- [ ] **Step 2: Run the integration contract and verify that it fails**

Run:

```powershell
& .\tests\up_stair_integration_contract_test.ps1
```

Expected: FAIL at the first missing V/retract integration assertion.

- [ ] **Step 3: Generate and export the V action sequence**

In `RtosTask/up_stair.hpp`, add:

```cpp
extern volatile uint32_t rear_retract_action_sequence;
```

Beside `stair_action_sequence` in `RtosTask/can_send_task.cpp`, define:

```cpp
volatile uint32_t rear_retract_action_sequence = 0U;
```

After the existing B event handling, add:

```cpp
if (keyboard_cmd.rear_retract_toggle)
{
    ++rear_retract_action_sequence;
}
```

- [ ] **Step 4: Add rear retract PID instances and feedback sampling**

In `RtosTask/up_stair.cpp`, add conservative bench-limit cascades:

```cpp
ALG::PID::PID rear_6248_left_retract_pid[2] = {
    {5.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f, 8.0f, 0.0f, 0.0f},
};
ALG::PID::PID rear_6248_right_retract_pid[2] = {
    {5.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f, 8.0f, 0.0f, 0.0f},
};
```

Extend `ResetAllStairPid()` to reset all four new PIDs. Sample rear velocities beside rear angles:

```cpp
const float rear_left_velocity = rear_6248.getVelocityRads(1);
const float rear_right_velocity = rear_6248.getVelocityRads(2);
```

Snapshot both front and rear action sequences in the existing critical section:

```cpp
uint32_t front_action_sequence;
uint32_t rear_action_sequence;
// inside taskENTER_CRITICAL()/taskEXIT_CRITICAL()
front_action_sequence = stair_action_sequence;
rear_action_sequence = rear_retract_action_sequence;
```

Replace direct uses of `stair_action_sequence` in this loop with `front_action_sequence`.

- [ ] **Step 5: Pass V state into the rear FSM and select one controller**

Extend the rear FSM update call:

```cpp
up_stair_behind_motor_fsm.Update(
    policy.rear_attitude_enabled,
    imu_valid,
    rear_left_online,
    rear_right_online,
    imu_snapshot.pitch_deg,
    imu_snapshot.roll_deg,
    imu_snapshot.pitch_rate_dps,
    imu_snapshot.roll_rate_dps,
    rear_left_angle,
    rear_right_angle,
    now_tick,
    policy.rear_retract_command_enabled,
    rear_action_sequence);
```

Replace the current disabled/attitude rear output branch with three exclusive branches:

```cpp
if (up_stair_behind_motor_fsm.Get_State() ==
    UP_STAIR_BEHIND_MOTOR_DISABLED)
{
    rear_6248_pitch_pid[0].reset();
    rear_6248_pitch_pid[1].reset();
    rear_6248_roll_pid[0].reset();
    rear_6248_roll_pid[1].reset();
    rear_6248_left_retract_pid[0].reset();
    rear_6248_left_retract_pid[1].reset();
    rear_6248_right_retract_pid[0].reset();
    rear_6248_right_retract_pid[1].reset();
    rear_6248.ctrl_Mit(1, SafeMotorAngle(rear_left_angle), 0.0f,
                       0.0f, 0.0f, 0.0f);
    rear_6248.ctrl_Mit(2, SafeMotorAngle(rear_right_angle), 0.0f,
                       0.0f, 0.0f, 0.0f);
}
else if (up_stair_behind_motor_fsm.Uses_Retract_Position_Control())
{
    rear_6248_pitch_pid[0].reset();
    rear_6248_pitch_pid[1].reset();
    rear_6248_roll_pid[0].reset();
    rear_6248_roll_pid[1].reset();

    const float left_target_velocity =
        rear_6248_left_retract_pid[0].UpDate(
            up_stair_behind_motor_fsm.Get_Retract_Target_Angle(1U),
            up_stair_behind_motor_fsm.Get_Position_Feedback(1U));
    const float left_torque = rear_6248_left_retract_pid[1].UpDate(
        left_target_velocity, rear_left_velocity);

    const float right_target_velocity =
        rear_6248_right_retract_pid[0].UpDate(
            up_stair_behind_motor_fsm.Get_Retract_Target_Angle(2U),
            up_stair_behind_motor_fsm.Get_Position_Feedback(2U));
    const float right_torque = rear_6248_right_retract_pid[1].UpDate(
        right_target_velocity, rear_right_velocity);

    rear_6248.ctrl_Mit(
        1, SafeMotorAngle(rear_left_angle), 0.0f, 0.0f, 0.0f,
        StairTorqueSafety::ClampJ6248Torque(
            up_stair_behind_motor_fsm.Limit_Torque(1U, left_torque)));
    rear_6248.ctrl_Mit(
        2, SafeMotorAngle(rear_right_angle), 0.0f, 0.0f, 0.0f,
        StairTorqueSafety::ClampJ6248Torque(
            up_stair_behind_motor_fsm.Limit_Torque(2U, right_torque)));
}
else if (up_stair_behind_motor_fsm.Uses_Attitude_Control())
{
    rear_6248_left_retract_pid[0].reset();
    rear_6248_left_retract_pid[1].reset();
    rear_6248_right_retract_pid[0].reset();
    rear_6248_right_retract_pid[1].reset();

    const float pitch_target_rate = rear_6248_pitch_pid[0].UpDate(
        up_stair_behind_motor_fsm.Get_Target_Pitch_Deg(),
        up_stair_behind_motor_fsm.Get_Feedback_Pitch_Deg());
    const float pitch_torque = rear_6248_pitch_pid[1].UpDate(
        pitch_target_rate,
        up_stair_behind_motor_fsm.Get_Pitch_Rate_Dps());
    const float roll_target_rate = rear_6248_roll_pid[0].UpDate(
        up_stair_behind_motor_fsm.Get_Target_Roll_Deg(),
        up_stair_behind_motor_fsm.Get_Feedback_Roll_Deg());
    const float roll_torque = rear_6248_roll_pid[1].UpDate(
        roll_target_rate,
        up_stair_behind_motor_fsm.Get_Roll_Rate_Dps());
    const float left_mixed_torque =
        static_cast<float>(
            up_stair_behind_motor_fsm.Get_Motor_Direction(1U)) *
        (pitch_torque + roll_torque);
    const float right_mixed_torque =
        static_cast<float>(
            up_stair_behind_motor_fsm.Get_Motor_Direction(2U)) *
        (pitch_torque - roll_torque);

    rear_6248.ctrl_Mit(
        1, SafeMotorAngle(rear_left_angle), 0.0f, 0.0f, 0.0f,
        StairTorqueSafety::ClampJ6248Torque(
            up_stair_behind_motor_fsm.Limit_Torque(
                1U, left_mixed_torque)));
    rear_6248.ctrl_Mit(
        2, SafeMotorAngle(rear_right_angle), 0.0f, 0.0f, 0.0f,
        StairTorqueSafety::ClampJ6248Torque(
            up_stair_behind_motor_fsm.Limit_Torque(
                2U, right_mixed_torque)));
}
```

Do not multiply retract PID torque by `Get_Motor_Direction()`: retract targets and feedback are both encoder-coordinate angles, so the PID output is already expressed in encoder-positive torque direction. Keep `Get_Motor_Direction()` only in the attitude mixing branch.

- [ ] **Step 6: Keep deployment calibration fail-safe**

Extend `BuildRear6248Config()` to assign the new fields explicitly:

```cpp
config.retract_target_rad[0] = 0.0f;
config.retract_target_rad[1] = 0.0f;
config.retract_speed_rad_s = 0.0f;
config.retract_position_tolerance_rad = 0.0f;
```

These values deliberately keep the existing uncalibrated rear configuration invalid. Before powered bench use, replace the existing zero mechanical limits and these four values with measured values; `Validate_Config()` must prevent any rear torque until all measurements are valid. Do not insert guessed physical angles into production firmware.

- [ ] **Step 7: Run integration and host tests**

Run:

```powershell
& .\tests\up_stair_integration_contract_test.ps1
g++ -std=c++11 -I. tests/chassis_keyboard_fsm_test.cpp fsm/chassis_keyboard_fsm.cpp -o tests/build/chassis_keyboard_fsm_test.exe
& .\tests\build\chassis_keyboard_fsm_test.exe
g++ -std=c++11 -I. tests/stair_mode_policy_test.cpp -o tests/build/stair_mode_policy_test.exe
& .\tests\build\stair_mode_policy_test.exe
g++ -std=c++11 -I. tests/up_stair_behind_motor_fsm_test.cpp fsm/up_stair_behind_motor_fsm.cpp user/core/Alg/FSM/alg_fsm.cpp user/core/Alg/UtilityFunction/SlopPlanning.cpp -o tests/build/up_stair_behind_motor_fsm_test.exe
& .\tests\build\up_stair_behind_motor_fsm_test.exe
```

Expected: contract prints `up stair integration contract passed`; all executables exit with code 0.

- [ ] **Step 8: Commit task integration**

```powershell
git add RtosTask/up_stair.hpp RtosTask/can_send_task.cpp RtosTask/up_stair.cpp tests/up_stair_integration_contract_test.ps1
git commit -m "feat: integrate V-controlled rear leg retraction"
```

### Task 5: Regression verification and calibration handoff

**Files:**
- Verify only; no production file changes expected.

- [ ] **Step 1: Run the full stair-related host suite**

Run:

```powershell
g++ -std=c++11 -I. tests/up_stair_fsm_test.cpp fsm/up_stair_fsm.cpp user/core/Alg/FSM/alg_fsm.cpp -o tests/build/up_stair_fsm_test.exe
& .\tests\build\up_stair_fsm_test.exe
g++ -std=c++11 -I. tests/motor_recovery_fsm_test.cpp -o tests/build/motor_recovery_fsm_test.exe
& .\tests\build\motor_recovery_fsm_test.exe
g++ -std=c++11 -I. tests/rear_torque_safety_test.cpp -o tests/build/rear_torque_safety_test.exe
& .\tests\build\rear_torque_safety_test.exe
& .\tests\fdcan1_dm_routing_test.ps1
& .\tests\id1_enable_dispatch_test.ps1
& .\tests\dm_motor_clamp_contract_test.ps1
& .\tests\switch_heartbeat_contract_test.ps1
```

Expected: every executable exits with code 0 and every PowerShell contract prints its pass message.

- [ ] **Step 2: Review the final diff against the approved design**

Run:

```powershell
git diff HEAD~4 --check
git diff HEAD~4 -- fsm/chassis_keyboard_fsm.hpp fsm/chassis_keyboard_fsm.cpp fsm/stair_mode_policy.hpp fsm/up_stair_behind_motor_fsm.hpp fsm/up_stair_behind_motor_fsm.cpp RtosTask/up_stair.hpp RtosTask/can_send_task.cpp RtosTask/up_stair.cpp tests
```

Confirm all of the following from the diff:

- First V enters encoder-only retract control.
- Both motors reaching the calibrated target enters angle hold.
- Second V enters the existing attitude recovery ramp without a normal-angle command.
- IMU input is ignored in retract and hold states.
- B/front behavior and operator-controlled wheel speed remain unchanged.
- Any retract feedback fault zeros both rear outputs.
- Production remains safely disabled until real rear calibration values replace the explicit zero configuration.

- [ ] **Step 3: Perform the staged physical calibration after code review**

Use this fixed order on the robot:

```text
1. Raise the chassis so rear wheels cannot load the floor.
2. Keep J6248 torque limited to the 8 Nm retract-loop limit.
3. Measure left/right safe angle intervals and encoder-positive directions.
4. Measure left/right retract targets that clear the stair edge.
5. Enter those measurements in BuildRear6248Config().
6. Verify one V press retracts and holds both suspended wheels.
7. Verify the next V press produces a 300 ms attitude-torque ramp.
8. Test the four-step stair sequence at low wheel speed before loaded runs.
```

Expected: no powered rear output is possible before valid calibration; after calibration, V toggles only the rear control mode and never commands chassis motion.
