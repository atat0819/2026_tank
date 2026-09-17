# Up-stair Wrapped Mechanical Limits Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the two-motor up-stair controller to support ordinary and zero-crossing mechanical intervals while generating safe continuous radian targets for PID control.

**Architecture:** Keep the shared three-state FSM (`DISABLED`, `HOME`, `TARGET`) and independent per-motor configuration arrays. For each motor, validate raw encoder feedback on a circular 0–2π domain, then map valid raw values into a continuous interval coordinate; ordinary intervals remain unchanged, while wrapped intervals add 2π to values below the interval start. Use the continuous angle for position PID and raw angle for the DM MIT position field.

**Tech Stack:** C++11 embedded code, existing `Class_FSM`, DM J4310 driver, two-stage PID, host-side `g++` assertion tests.

---

## Files

- Modify: `fsm/up_stair_fsm.hpp` — rename limit arrays to start/end semantics, add continuous-angle accessor, preserve raw-angle accessor.
- Modify: `fsm/up_stair_fsm.cpp` — implement ordinary/wrapped interval validation and raw-to-continuous mapping.
- Modify: `RtosTask/up_stair.cpp` — feed continuous position feedback to position PID and raw position to MIT.
- Modify: `tests/up_stair_fsm_test.cpp` — add left ordinary-range and right wrapped-range tests.

### Task 1: Add failing tests for wrapped intervals

**Files:**
- Modify: `tests/up_stair_fsm_test.cpp`

- [ ] **Step 1: Replace ordinary-only range assertions with wrapped-range expectations**

Add assertions expressing the confirmed configuration and desired mapping:

```cpp
// Left motor: ordinary interval 10 -> 178 degrees.
assert(fsm.Is_Angle_Valid(1U, deg_to_rad(10.0f)));
assert(fsm.Is_Angle_Valid(1U, deg_to_rad(178.0f)));
assert(!fsm.Is_Angle_Valid(1U, deg_to_rad(9.0f)));
assert(!fsm.Is_Angle_Valid(1U, deg_to_rad(179.0f)));

// Right motor: wrapped interval 300 -> 70 degrees.
assert(fsm.Is_Angle_Valid(2U, deg_to_rad(300.0f)));
assert(fsm.Is_Angle_Valid(2U, deg_to_rad(350.0f)));
assert(fsm.Is_Angle_Valid(2U, deg_to_rad(0.0f)));
assert(fsm.Is_Angle_Valid(2U, deg_to_rad(70.0f)));
assert(!fsm.Is_Angle_Valid(2U, deg_to_rad(71.0f)));
assert(!fsm.Is_Angle_Valid(2U, deg_to_rad(299.0f)));

// Right home 340 -> 340 control degrees; target 60 -> 420 control degrees.
fsm.Update(deg_to_rad(34.0f), deg_to_rad(340.0f), true, true, 0U);
assert(near(fsm.Get_Position_Feedback(2U), deg_to_rad(340.0f)));
fsm.Update(deg_to_rad(34.0f), deg_to_rad(340.0f), true, true, 1U);
assert(near(fsm.Get_Target_Angle(2U), deg_to_rad(420.0f)));
assert(near(fsm.Get_Position_Error(2U), deg_to_rad(80.0f)));
```

The helper `deg_to_rad()` should be a test-only function. The production FSM
must expose `Get_Position_Feedback(id)` and `Get_Position_Error(id)` for
verification of the continuous coordinate; `Get_Current_Angle(id)` remains the
raw encoder value.

- [ ] **Step 2: Run the test and confirm it fails for the expected reason**

Run:

```powershell
g++ -std=c++11 -I. tests/up_stair_fsm_test.cpp fsm/up_stair_fsm.cpp user/core/Alg/FSM/alg_fsm.cpp -o tests/build/up_stair_fsm_test.exe
```

Expected: compile failure because the current production implementation has
ordinary-only limits and no continuous position-feedback/error accessors.

### Task 2: Implement ordinary and wrapped interval mapping

**Files:**
- Modify: `fsm/up_stair_fsm.hpp`
- Modify: `fsm/up_stair_fsm.cpp`

- [ ] **Step 1: Configure the confirmed left/right values**

Use independently editable arrays in radians:

```cpp
static constexpr float LIMIT_START_RAD[2] = {
    10.0f * PI / 180.0f,
    300.0f * PI / 180.0f};
static constexpr float LIMIT_END_RAD[2] = {
    178.0f * PI / 180.0f,
    70.0f * PI / 180.0f};
static constexpr float HOME_ANGLE_RAD[2] = {
    34.0f * PI / 180.0f,
    340.0f * PI / 180.0f};
static constexpr float TARGET_ANGLE_RAD[2] = {
    122.0f * PI / 180.0f,
    60.0f * PI / 180.0f};
```

Use the existing numeric π literal or a C++11-compatible constant; do not
introduce a dependency on a platform math macro in the header.

- [ ] **Step 2: Implement wrapped validity**

For motor index `i`:

```cpp
const bool wrapped = LIMIT_START_RAD[i] > LIMIT_END_RAD[i];
return wrapped
           ? (angle >= LIMIT_START_RAD[i] || angle <= LIMIT_END_RAD[i])
           : (angle >= LIMIT_START_RAD[i] && angle <= LIMIT_END_RAD[i]);
```

Reject IDs outside 1–2. Validate both targets against their corresponding
intervals during `Init()`; an invalid configuration leaves the FSM disabled.

- [ ] **Step 3: Implement continuous coordinate conversion**

Add:

```cpp
float To_Control_Angle(uint8_t index, float raw_angle) const
{
    if (LIMIT_START_RAD[index] > LIMIT_END_RAD[index] &&
        raw_angle < LIMIT_START_RAD[index])
    {
        return raw_angle + 2.0f * PI;
    }
    return raw_angle;
}
```

Store both values each update:

```cpp
raw_angle_[i] = raw_angle;
control_angle_[i] = To_Control_Angle(i, raw_angle);
```

Map each target using the same function. For the right motor this produces:

```text
340° -> 340°
60°  -> 420°
```

The position error is `target_control_angle - control_angle`; do not apply a
second ±2π normalization after this mapping.

- [ ] **Step 4: Add accessors and retain raw-angle semantics**

Implement:

```cpp
float Get_Position_Feedback(uint8_t id) const;
float Get_Position_Error(uint8_t id) const;
float Get_Current_Angle(uint8_t id) const; // raw [0, 2π]
float Get_Target_Angle(uint8_t id) const;  // continuous control coordinate
```

`Get_Position_Error()` must return the signed error for that motor. Keep the
three-state transitions and disabled behavior unchanged.

- [ ] **Step 5: Run the host tests and verify they pass**

Run:

```powershell
g++ -std=c++11 -I. tests/up_stair_fsm_test.cpp fsm/up_stair_fsm.cpp user/core/Alg/FSM/alg_fsm.cpp -o tests/build/up_stair_fsm_test.exe
tests/build/up_stair_fsm_test.exe
```

Expected: exit code 0 with no assertion failures.

### Task 3: Connect the two coordinate types to the motor task

**Files:**
- Modify: `RtosTask/up_stair.cpp`

- [ ] **Step 1: Use continuous feedback in each position PID**

Replace raw-angle position feedback with the continuous FSM value:

```cpp
const float front_left_target_velocity =
    front_4340_left_pid[0].UpDate(up_stair_fsm.Get_Target_Angle(1),
                                  up_stair_fsm.Get_Position_Feedback(1));
const float front_right_target_velocity =
    front_4340_right_pid[0].UpDate(up_stair_fsm.Get_Target_Angle(2),
                                   up_stair_fsm.Get_Position_Feedback(2));
```

- [ ] **Step 2: Keep raw feedback in MIT position field**

Continue passing `Get_Current_Angle(1/2)` to `ctrl_Mit()` because the DM J4310
position encoding is raw `0～2π` radians and the external position PID is
providing the torque command.

- [ ] **Step 3: Verify disabled behavior and compile references**

Confirm the disabled branch resets both position and velocity PID objects and
sends zero velocity, zero KP, zero KD, and zero torque. Search for removed
symbols:

```powershell
rg -n "Get_Accumulated_Angle|normalize_delta|accumulated_angle_|UP_STAIR_FAULT|UP_STAIR_MOVING" fsm RtosTask --glob '*.cpp' --glob '*.hpp'
```

Expected: no matches.

- [ ] **Step 4: Run syntax checks**

```powershell
g++ -std=c++11 -fsyntax-only -I. fsm/up_stair_fsm.cpp
g++ -std=c++11 -fsyntax-only -I. fsm/chassis_keyboard_fsm.cpp
```

Expected: both commands exit with code 0. A complete RTOS task compile requires
the project's Keil/CMSIS environment; if unavailable, record that limitation.

- [ ] **Step 5: Commit only task-owned files**

Because `RtosTask/up_stair.cpp` already contains unrelated user changes, inspect
its diff before staging. Commit the FSM/test changes and the intended task
integration without reverting or staging unrelated files:

```powershell
git add fsm/up_stair_fsm.hpp fsm/up_stair_fsm.cpp tests/up_stair_fsm_test.cpp RtosTask/up_stair.cpp
git commit -m "feat: support wrapped per-motor stair limits"
```

## Final verification

- [ ] Left interval `10°→178°` validates normally.
- [ ] Right interval `300°→70°` validates across zero.
- [ ] Right `340°→60°` produces a continuous `+80°` error.
- [ ] Position PID uses continuous feedback; MIT uses raw feedback.
- [ ] Any invalid feedback or invalid configuration disables both motors.
- [ ] Host test passes; FSM syntax checks pass.
