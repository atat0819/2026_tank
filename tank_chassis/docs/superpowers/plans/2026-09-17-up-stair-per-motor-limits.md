# Up-stair Per-Motor Mechanical Limits Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rewrite the up-stair controller so each J4310 motor has independent radian targets and mechanical limits, while direct signed position error selects the shortest safe path inside each motor's allowed interval.

**Architecture:** Keep a shared three-state command FSM (`DISABLED`, `HOME`, `TARGET`) because both motors operate in the same mode, but store angle configuration and feedback independently for motor 1 and motor 2. Use raw J4310 angles in radians and reject feedback outside each motor's configured mechanical interval; do not unwrap across 0/2π because the allowed intervals are bounded mechanical segments.

**Tech Stack:** C++11-compatible embedded C++, CMSIS-RTOS task, existing `Class_FSM`, existing DM J4310 driver, existing two-stage position/velocity PID, host-side `g++` assertions for FSM behavior.

---

## Files and responsibilities

- Modify `fsm/up_stair_fsm.hpp`: define the three states, the simplified `Update()` interface, per-motor configuration declarations, and per-motor accessors.
- Modify `fsm/up_stair_fsm.cpp`: implement direct radian feedback validation, independent target selection, disabled behavior, and B action-sequence handling.
- Modify `RtosTask/up_stair.cpp`: pass the keyboard enable condition and two raw angle feedback values to the new FSM interface; retain PID and zero-torque handling.
- Create `tests/up_stair_fsm_test.cpp`: exercise the FSM without hardware using the existing `alg_fsm.cpp` and `up_stair_fsm.cpp` implementations.

### Task 1: Add failing host tests for per-motor limits and direct-path behavior

**Files:**
- Create: `tests/up_stair_fsm_test.cpp`

- [ ] **Step 1: Write the failing test executable**

Use a small assertion-based test program that constructs the FSM, supplies two
different motor configurations through the production constants, and checks
these behaviors:

```cpp
#include "../fsm/up_stair_fsm.hpp"
#include <assert.h>
#include <math.h>

static bool near(float actual, float expected)
{
    return fabsf(actual - expected) < 1.0e-5f;
}

int main()
{
    Class_Up_Stair_FSM fsm;
    fsm.Init(0U);

    // Disabled state must not expose a motion target as enabled.
    fsm.Update(1.0f, 1.0f, true, false, 0U);
    assert(fsm.Get_State() == UP_STAIR_DISABLED);
    assert(!fsm.Is_Enabled());

    // Enabling enters HOME and returns each motor to its own home angle.
    fsm.Update(2.0f, 2.2f, true, true, 0U);
    assert(fsm.Get_State() == UP_STAIR_HOME);
    assert(fsm.Is_Enabled());
    assert(near(fsm.Get_Target_Angle(1), HOME_ANGLE_RAD[0]));
    assert(near(fsm.Get_Target_Angle(2), HOME_ANGLE_RAD[1]));

    // A new action sequence selects independent target angles.
    fsm.Update(2.0f, 2.2f, true, true, 1U);
    assert(fsm.Get_State() == UP_STAIR_TARGET);
    assert(near(fsm.Get_Target_Angle(1), TARGET_ANGLE_RAD[0]));
    assert(near(fsm.Get_Target_Angle(2), TARGET_ANGLE_RAD[1]));

    // A valid angle near the upper limit must use the direct negative error;
    // it must not be unwrapped through 2*pi.
    assert(fsm.Is_Angle_Valid(1, LIMIT_MAX_RAD[0] - 0.01f));
    assert(!fsm.Is_Angle_Valid(1, LIMIT_MIN_RAD[0] - 0.01f));
    assert(!fsm.Is_Angle_Valid(2, LIMIT_MAX_RAD[1] + 0.01f));

    // Either motor outside its own mechanical interval disables both.
    fsm.Update(LIMIT_MIN_RAD[0] - 0.01f, HOME_ANGLE_RAD[1], true, true, 1U);
    assert(fsm.Get_State() == UP_STAIR_DISABLED);
    assert(!fsm.Is_Enabled());
    return 0;
}
```

The production API may expose the configuration arrays and `Is_Angle_Valid()`
as `public` read-only test/configuration symbols, or the test may use the
documented configured values directly; choose one interface and use it
consistently in the implementation task below.

- [ ] **Step 2: Run the test and verify the expected failure**

Run:

```powershell
New-Item -ItemType Directory -Force tests\build | Out-Null
g++ -std=c++11 -I. tests\up_stair_fsm_test.cpp fsm\up_stair_fsm.cpp user\core\Alg\FSM\alg_fsm.cpp -o tests\build\up_stair_fsm_test.exe
```

Expected: compilation failure because the production FSM does not yet expose
per-motor configuration or the simplified validation behavior.

### Task 2: Implement the per-motor bounded-angle FSM

**Files:**
- Modify: `fsm/up_stair_fsm.hpp`
- Modify: `fsm/up_stair_fsm.cpp`

- [ ] **Step 1: Define independently configurable radian arrays**

Add four configuration arrays with one entry per motor:

```cpp
static constexpr float LIMIT_MIN_RAD[2] = {
    10.0f * 3.14159265359f / 180.0f,
    10.0f * 3.14159265359f / 180.0f};
static constexpr float LIMIT_MAX_RAD[2] = {
    178.0f * 3.14159265359f / 180.0f,
    178.0f * 3.14159265359f / 180.0f};
static constexpr float HOME_ANGLE_RAD[2] = {
    34.0f * 3.14159265359f / 180.0f,
    34.0f * 3.14159265359f / 180.0f};
static constexpr float TARGET_ANGLE_RAD[2] = {
    122.0f * 3.14159265359f / 180.0f,
    122.0f * 3.14159265359f / 180.0f};
```

Keep them easy to edit for left/right calibration. If the two physical motors
need different encoder angles, only these array entries change.

- [ ] **Step 2: Remove circular-angle accumulation**

Remove `normalize_delta()`, `last_raw_angle_`, and `accumulated_angle_`. Store
the current raw radian angle in `raw_angle_[2]` and make
`Get_Accumulated_Angle(id)` either return the raw angle for compatibility or
replace its call sites with a clearly named `Get_Current_Angle(id)`. The
position PID must receive the same raw radian coordinate as the fixed target.

- [ ] **Step 3: Add configuration and feedback validation**

Implement a helper with this exact behavior:

```cpp
bool Class_Up_Stair_FSM::Is_Angle_Valid(uint8_t id, float angle) const
{
    if (id < 1U || id > 2U)
    {
        return false;
    }
    const uint8_t index = id - 1U;
    return angle >= LIMIT_MIN_RAD[index] &&
           angle <= LIMIT_MAX_RAD[index];
}
```

At initialization, validate every configured target satisfies:

```text
LIMIT_MIN_RAD[i] <= HOME_ANGLE_RAD[i] <= LIMIT_MAX_RAD[i]
LIMIT_MIN_RAD[i] <= TARGET_ANGLE_RAD[i] <= LIMIT_MAX_RAD[i]
LIMIT_MIN_RAD[i] < LIMIT_MAX_RAD[i]
```

If configuration is invalid, keep the FSM disabled. During `Update()`, if
feedback is invalid or either motor angle fails its own interval check, set
`UP_STAIR_DISABLED`, reset angle initialization, synchronize
`last_action_sequence_`, and return.

- [ ] **Step 4: Implement the three-state transitions**

Use this transition order:

```cpp
if (!feedback_valid || !enabled || !config_valid_ ||
    !Is_Angle_Valid(1U, current_left_angle) ||
    !Is_Angle_Valid(2U, current_right_angle)) {
    state = UP_STAIR_DISABLED;
    last_action_sequence_ = action_sequence;
    return;
}

raw_angle_[0] = current_left_angle;
raw_angle_[1] = current_right_angle;

if (state == UP_STAIR_DISABLED) {
    state = UP_STAIR_HOME;
}

if (action_sequence != last_action_sequence_) {
    last_action_sequence_ = action_sequence;
    state = UP_STAIR_TARGET;
}

for (uint8_t i = 0U; i < 2U; ++i) {
    target_angle_[i] = (state == UP_STAIR_TARGET)
                           ? TARGET_ANGLE_RAD[i]
                           : HOME_ANGLE_RAD[i];
}
```

The target for motor `i` must be returned by `Get_Target_Angle(i + 1)`.
Do not calculate `target ± 2π`; direct signed error within each configured
mechanical interval is the shortest permitted path.

- [ ] **Step 5: Run the FSM host test and verify it passes**

Run:

```powershell
g++ -std=c++11 -I. tests\up_stair_fsm_test.cpp fsm\up_stair_fsm.cpp user\core\Alg\FSM\alg_fsm.cpp -o tests\build\up_stair_fsm_test.exe
tests\build\up_stair_fsm_test.exe
```

Expected: process exits with code 0 and no assertion failures.

- [ ] **Step 6: Commit the FSM and test changes**

```powershell
git add fsm\up_stair_fsm.hpp fsm\up_stair_fsm.cpp tests\up_stair_fsm_test.cpp
git commit -m "feat: bound up stair targets per motor"
```

### Task 3: Update the RTOS task integration

**Files:**
- Modify: `RtosTask/up_stair.cpp`

- [ ] **Step 1: Update the FSM call to pass raw radian feedback and enable state**

Keep the existing feedback sources:

```cpp
const float front_left_angle = front_4340.getAngleRad(1);
const float front_right_angle = front_4340.getAngleRad(2);
```

Call the simplified API:

```cpp
up_stair_fsm.Update(front_left_angle,
                    front_right_angle,
                    motors_online,
                    keyboard_mode,
                    stair_action_sequence);
```

- [ ] **Step 2: Feed each motor's target and raw radian feedback to its own PID**

Use:

```cpp
const float front_left_target_velocity =
    front_4340_left_pid[0].UpDate(up_stair_fsm.Get_Target_Angle(1),
                                  up_stair_fsm.Get_Current_Angle(1));
const float front_right_target_velocity =
    front_4340_right_pid[0].UpDate(up_stair_fsm.Get_Target_Angle(2),
                                   up_stair_fsm.Get_Current_Angle(2));
```

The velocity PID continues to use `getVelocityRads()` feedback. The MIT
position field continues to receive the corresponding raw radian angle.

- [ ] **Step 3: Preserve zero-torque behavior while disabled**

When `!motors_online || !up_stair_fsm.Is_Enabled()`, reset all four PID
objects. When motors are online but the FSM is disabled, send each motor its
own current radian position with zero velocity, zero KP, zero KD, and zero
torque. Do not send a stale target while disabled.

- [ ] **Step 4: Run host syntax checks**

Run:

```powershell
g++ -std=c++11 -fsyntax-only -I. fsm\up_stair_fsm.cpp
g++ -std=c++11 -fsyntax-only -I. fsm\chassis_keyboard_fsm.cpp
```

Expected: both commands exit with code 0.

- [ ] **Step 5: Review the Keil project source list and build**

Confirm `fsm/up_stair_fsm.cpp` remains included in `MDK-ARM/tank_chassis.uvprojx`,
then build the target in Keil. Verify there are no stale references to removed
states such as `UP_STAIR_MOVING_FORWARD`, `UP_STAIR_FAULT`, or
`Get_Accumulated_Angle()`.

- [ ] **Step 6: Commit the task integration**

```powershell
git add RtosTask\up_stair.cpp
git commit -m "feat: use bounded per-motor stair targets"
```

## Final verification checklist

- [ ] Motor 1 and motor 2 have independent limit, home, and target values.
- [ ] All configured angles and feedback angles remain in radians.
- [ ] The position PID compares target rad against the matching motor's raw rad feedback.
- [ ] A valid current angle near either mechanical limit uses direct signed error and never circularly unwraps through 0/2π.
- [ ] Any invalid feedback or out-of-range feedback disables both motors.
- [ ] Disabled mode resets PID and sends zero torque when motors are connected.
- [ ] B action changes both motors to their independently configured target angles.
- [ ] Host tests pass and the Keil target builds successfully.
