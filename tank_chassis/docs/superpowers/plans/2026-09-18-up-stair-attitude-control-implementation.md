# Up-Stair Attitude Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep the front J4310 pair actively holding or executing the `B`-key up-stair motion, while the rear J6248 pair continuously levels chassis pitch and roll in every active switch mode and all four mechanism motors produce zero torque in double-down.

**Architecture:** Put switch policy in a small pure header, retain front position sequencing in `Class_Up_Stair_FSM`, and add a hardware-independent rear attitude controller that owns the pitch/roll cascaded PIDs, torque mixing, slew limiting, and encoder safety limits. `up_stair_task` remains the hardware adapter: it samples switches, IMU, and motors, invokes the pure controllers, and sends MIT commands.

**Tech Stack:** Embedded C++, STM32H7/CMSIS-RTOS, BMI088, DM J4310/J6248 MIT control, existing `ALG::PID::PID`, host `g++` assertion tests, Keil MDK. Host tests that link `pid.cpp` use C++17 because the existing PID implementation uses `std::clamp`.

---

## File map

- Create `fsm/stair_mode_policy.hpp`: pure double-down/double-middle policy.
- Create `tests/stair_mode_policy_test.cpp`: exhaustive switch-policy tests.
- Modify `fsm/up_stair_fsm.hpp`: explicit moving/holding/return states and separate mechanism/command enables.
- Modify `fsm/up_stair_fsm.cpp`: controlled return and active home-hold transitions.
- Modify `tests/up_stair_fsm_test.cpp`: front transition and mode-exit coverage.
- Create `fsm/rear_attitude_controller.hpp`: rear controller data types and interface.
- Create `fsm/rear_attitude_controller.cpp`: cascaded attitude control, mixing, limits, and slew handling.
- Create `tests/rear_attitude_controller_test.cpp`: pitch, roll, saturation, disable, and limit tests.
- Modify `RtosTask/can_send_task.hpp`: publish a fresh-switch-frame timestamp.
- Modify `RtosTask/can_send_task.cpp`: update switch-frame freshness on CAN ID `0x303`.
- Modify `RtosTask/up_stair.hpp`: expose only mechanism objects needed outside the task.
- Modify `RtosTask/up_stair.cpp`: integrate front and rear controllers and the safety gate.
- Modify `MDK-ARM/tank_chassis.uvprojx`: add the new controller source/header and policy header to the existing FSM group.

### Task 1: Encode the switch-mode safety policy

**Files:**
- Create: `fsm/stair_mode_policy.hpp`
- Create: `tests/stair_mode_policy_test.cpp`

- [ ] **Step 1: Write the failing policy test**

Create `tests/stair_mode_policy_test.cpp`:

```cpp
#include "../fsm/stair_mode_policy.hpp"
#include <assert.h>

int main()
{
    for (uint8_t s1 = 1U; s1 <= 3U; ++s1)
    {
        for (uint8_t s2 = 1U; s2 <= 3U; ++s2)
        {
            const StairModePolicy active =
                EvaluateStairModePolicy(s1, s2, true, true);

            if (s1 == STAIR_SWITCH_DOWN && s2 == STAIR_SWITCH_DOWN)
            {
                assert(active.zero_all_torque);
                assert(!active.front_hold_enabled);
                assert(!active.rear_attitude_enabled);
                assert(!active.front_stair_command_enabled);
            }
            else
            {
                assert(!active.zero_all_torque);
                assert(active.front_hold_enabled);
                assert(active.rear_attitude_enabled);
                assert(active.front_stair_command_enabled ==
                       (s1 == STAIR_SWITCH_MIDDLE &&
                        s2 == STAIR_SWITCH_MIDDLE));
            }
        }
    }

    const StairModePolicy stale_link =
        EvaluateStairModePolicy(3U, 3U, false, true);
    assert(stale_link.zero_all_torque);
    assert(!stale_link.front_hold_enabled);
    assert(!stale_link.rear_attitude_enabled);

    const StairModePolicy stale_keyboard =
        EvaluateStairModePolicy(3U, 3U, true, false);
    assert(!stale_keyboard.zero_all_torque);
    assert(stale_keyboard.front_hold_enabled);
    assert(stale_keyboard.rear_attitude_enabled);
    assert(!stale_keyboard.front_stair_command_enabled);

    const StairModePolicy invalid_switch =
        EvaluateStairModePolicy(0U, 3U, true, true);
    assert(invalid_switch.zero_all_torque);
    assert(!invalid_switch.front_hold_enabled);
    assert(!invalid_switch.rear_attitude_enabled);
    return 0;
}
```

- [ ] **Step 2: Run the test and verify the missing-header failure**

```powershell
New-Item -ItemType Directory -Force tests/build | Out-Null
g++ -std=c++11 -I. tests/stair_mode_policy_test.cpp -o tests/build/stair_mode_policy_test.exe
```

Expected: compilation fails because `fsm/stair_mode_policy.hpp` does not exist.

- [ ] **Step 3: Add the complete header-only policy**

Create `fsm/stair_mode_policy.hpp`:

```cpp
#ifndef STAIR_MODE_POLICY_HPP
#define STAIR_MODE_POLICY_HPP

#include <stdint.h>

static constexpr uint8_t STAIR_SWITCH_UP = 1U;
static constexpr uint8_t STAIR_SWITCH_DOWN = 2U;
static constexpr uint8_t STAIR_SWITCH_MIDDLE = 3U;

struct StairModePolicy
{
    bool zero_all_torque;
    bool front_hold_enabled;
    bool rear_attitude_enabled;
    bool front_stair_command_enabled;
};

inline StairModePolicy EvaluateStairModePolicy(uint8_t s1,
                                               uint8_t s2,
                                               bool control_link_online,
                                               bool keyboard_online)
{
    const bool valid_switches =
        s1 >= STAIR_SWITCH_UP && s1 <= STAIR_SWITCH_MIDDLE &&
        s2 >= STAIR_SWITCH_UP && s2 <= STAIR_SWITCH_MIDDLE;
    const bool double_down =
        s1 == STAIR_SWITCH_DOWN && s2 == STAIR_SWITCH_DOWN;

    if (!control_link_online || !valid_switches || double_down)
    {
        return {true, false, false, false};
    }

    const bool double_middle =
        s1 == STAIR_SWITCH_MIDDLE && s2 == STAIR_SWITCH_MIDDLE;
    return {false, true, true, double_middle && keyboard_online};
}

#endif
```

- [ ] **Step 4: Run the policy test**

```powershell
g++ -std=c++11 -I. tests/stair_mode_policy_test.cpp -o tests/build/stair_mode_policy_test.exe
tests/build/stair_mode_policy_test.exe
```

Expected: exit code 0.

- [ ] **Step 5: Commit the policy unit**

```powershell
git add fsm/stair_mode_policy.hpp tests/stair_mode_policy_test.cpp
git commit -m "feat: define stair mechanism mode policy"
```

### Task 2: Make the front FSM return and hold home outside double-middle

**Files:**
- Modify: `fsm/up_stair_fsm.hpp`
- Modify: `fsm/up_stair_fsm.cpp`
- Modify: `tests/up_stair_fsm_test.cpp`

- [ ] **Step 1: Replace the front-state assertions with the confirmed behavior**

Update every existing test call by inserting `mechanism_enabled` and
`stair_command_enabled` before `action_sequence`, then add these transitions
after the existing wrapped-angle assertions:

Add these transitions after the existing wrapped-angle assertions:

```cpp
fsm.Update(deg_to_rad(34.0f), deg_to_rad(340.0f),
           true, true, true, false, 0U);
assert(fsm.Get_State() == UP_STAIR_HOME_HOLD);

fsm.Update(deg_to_rad(34.0f), deg_to_rad(340.0f),
           true, true, true, true, 1U);
assert(fsm.Get_State() == UP_STAIR_MOVING_TO_TARGET);

fsm.Update(deg_to_rad(122.0f), deg_to_rad(60.0f),
           true, true, true, true, 1U);
assert(fsm.Get_State() == UP_STAIR_TARGET_HOLD);

// Leaving double-middle forces a controlled return, not zero torque.
fsm.Update(deg_to_rad(122.0f), deg_to_rad(60.0f),
           true, true, true, false, 1U);
assert(fsm.Get_State() == UP_STAIR_RETURNING_HOME);
assert(fsm.Is_Enabled());

fsm.Update(deg_to_rad(34.0f), deg_to_rad(340.0f),
           true, true, true, false, 1U);
assert(fsm.Get_State() == UP_STAIR_HOME_HOLD);

// Double-down bypasses return and disables torque production.
fsm.Update(deg_to_rad(34.0f), deg_to_rad(340.0f),
           true, true, false, false, 1U);
assert(fsm.Get_State() == UP_STAIR_DISABLED);
assert(!fsm.Is_Enabled());

// Re-enable away from home: return first and ignore the old B sequence.
fsm.Update(deg_to_rad(100.0f), deg_to_rad(30.0f),
           true, true, true, true, 1U);
assert(fsm.Get_State() == UP_STAIR_RETURNING_HOME);
assert(near(fsm.Get_Target_Angle(1U),
            Class_Up_Stair_FSM::HOME_ANGLE_RAD[0]));
```

- [ ] **Step 2: Run the test and verify the signature/state failure**

```powershell
g++ -std=c++11 -I. tests/up_stair_fsm_test.cpp fsm/up_stair_fsm.cpp user/core/Alg/FSM/alg_fsm.cpp -o tests/build/up_stair_fsm_test.exe
```

Expected: compilation fails because the new states and `Update` parameter do not exist.

- [ ] **Step 3: Add the explicit state model and tolerance helper**

In `fsm/up_stair_fsm.hpp`, replace the enum and declaration with:

```cpp
enum Enum_Up_Stair_Status
{
    UP_STAIR_DISABLED = 0,
    UP_STAIR_HOME_HOLD,
    UP_STAIR_MOVING_TO_TARGET,
    UP_STAIR_TARGET_HOLD,
    UP_STAIR_RETURNING_HOME,
    UP_STAIR_STATUS_COUNT
};

static constexpr float POSITION_TOLERANCE_RAD =
    2.0f * 3.14159265359f / 180.0f;

void Update(float current_left_angle,
            float current_right_angle,
            bool left_feedback_valid,
            bool right_feedback_valid,
            bool mechanism_enabled,
            bool stair_command_enabled,
            uint32_t action_sequence);

bool Both_At_Commanded_Target() const;
```

Define `POSITION_TOLERANCE_RAD` beside the existing constexpr definitions in
`fsm/up_stair_fsm.cpp`.

- [ ] **Step 4: Implement controlled return and active hold**

Replace `Refresh_Target()` and `Update()` with this state logic while retaining
the existing wrapped-angle conversion:

```cpp
void Class_Up_Stair_FSM::Refresh_Target()
{
    const bool target_commanded =
        Get_Now_Status_Serial() == UP_STAIR_MOVING_TO_TARGET ||
        Get_Now_Status_Serial() == UP_STAIR_TARGET_HOLD;

    for (uint8_t i = 0U; i < 2U; ++i)
    {
        target_angle_[i] = To_Control_Angle(
            i, target_commanded ? TARGET_ANGLE_RAD[i] : HOME_ANGLE_RAD[i]);
    }
}

bool Class_Up_Stair_FSM::Both_At_Commanded_Target() const
{
    return fabsf(target_angle_[0] - control_angle_[0]) <= POSITION_TOLERANCE_RAD &&
           fabsf(target_angle_[1] - control_angle_[1]) <= POSITION_TOLERANCE_RAD;
}

void Class_Up_Stair_FSM::Update(float left_angle,
                                float right_angle,
                                bool left_feedback_valid,
                                bool right_feedback_valid,
                                bool mechanism_enabled,
                                bool stair_command_enabled,
                                uint32_t action_sequence)
{
    const bool left_valid = left_feedback_valid && Is_Angle_Valid(1U, left_angle);
    const bool right_valid = right_feedback_valid && Is_Angle_Valid(2U, right_angle);

    if (left_valid)
    {
        raw_angle_[0] = left_angle;
        control_angle_[0] = To_Control_Angle(0U, left_angle);
    }
    if (right_valid)
    {
        raw_angle_[1] = right_angle;
        control_angle_[1] = To_Control_Angle(1U, right_angle);
    }

    if (!mechanism_enabled || !config_valid_ || (!left_valid && !right_valid))
    {
        Set_Status(UP_STAIR_DISABLED);
        last_action_sequence_ = action_sequence;
        Refresh_Target();
        return;
    }

    if (Get_Now_Status_Serial() == UP_STAIR_DISABLED)
    {
        Set_Status(UP_STAIR_RETURNING_HOME);
        last_action_sequence_ = action_sequence;
        Refresh_Target();
    }

    if (!stair_command_enabled)
    {
        last_action_sequence_ = action_sequence;
        Set_Status(UP_STAIR_RETURNING_HOME);
        Refresh_Target();
    }
    else if (action_sequence != last_action_sequence_)
    {
        last_action_sequence_ = action_sequence;
        const bool heading_to_target =
            Get_Now_Status_Serial() == UP_STAIR_MOVING_TO_TARGET ||
            Get_Now_Status_Serial() == UP_STAIR_TARGET_HOLD;
        Set_Status(heading_to_target ? UP_STAIR_RETURNING_HOME
                                    : UP_STAIR_MOVING_TO_TARGET);
        Refresh_Target();
    }

    if (left_valid && right_valid && Both_At_Commanded_Target())
    {
        if (Get_Now_Status_Serial() == UP_STAIR_MOVING_TO_TARGET)
        {
            Set_Status(UP_STAIR_TARGET_HOLD);
        }
        else if (Get_Now_Status_Serial() == UP_STAIR_RETURNING_HOME)
        {
            Set_Status(UP_STAIR_HOME_HOLD);
        }
        Refresh_Target();
    }
}
```

Add `#include <math.h>` to `fsm/up_stair_fsm.cpp`.

- [ ] **Step 5: Run the front FSM test**

```powershell
g++ -std=c++11 -I. tests/up_stair_fsm_test.cpp fsm/up_stair_fsm.cpp user/core/Alg/FSM/alg_fsm.cpp -o tests/build/up_stair_fsm_test.exe
tests/build/up_stair_fsm_test.exe
```

Expected: exit code 0.

- [ ] **Step 6: Commit the front state machine**

```powershell
git add fsm/up_stair_fsm.hpp fsm/up_stair_fsm.cpp tests/up_stair_fsm_test.cpp
git commit -m "feat: return and hold front stair arms"
```

### Task 3: Specify the rear pitch/roll controller with tests

**Files:**
- Create: `tests/rear_attitude_controller_test.cpp`
- Produced in Task 4: `fsm/rear_attitude_controller.hpp`
- Produced in Task 4: `fsm/rear_attitude_controller.cpp`

- [ ] **Step 1: Write the complete failing controller test**

Create `tests/rear_attitude_controller_test.cpp`:

```cpp
#include "../fsm/rear_attitude_controller.hpp"
#include <assert.h>
#include <math.h>

static RearAttitudeConfig TestConfig()
{
    RearAttitudeConfig c = {};
    c.pitch_angle_kp = 1.0f;
    c.pitch_rate_kp = 1.0f;
    c.roll_angle_kp = 1.0f;
    c.roll_rate_kp = 1.0f;
    c.max_target_rate_dps = 50.0f;
    c.max_torque_nm = 10.0f;
    c.torque_slew_per_cycle_nm = 100.0f;
    c.angle_deadband_deg = 0.0f;
    c.limit_margin_rad = 0.05f;
    c.limit_start_rad[0] = 0.2f;
    c.limit_start_rad[1] = 5.5f;
    c.limit_end_rad[0] = 2.8f;
    c.limit_end_rad[1] = 0.8f;
    c.motor_direction[0] = 1.0f;
    c.motor_direction[1] = -1.0f;
    return c;
}

static RearAttitudeInput NominalInput()
{
    RearAttitudeInput in = {};
    in.enabled = true;
    in.imu_valid = true;
    in.left_feedback_valid = true;
    in.right_feedback_valid = true;
    in.left_angle_rad = 1.0f;
    in.right_angle_rad = 6.0f;
    return in;
}

int main()
{
    const RearAttitudeConfig config = TestConfig();
    RearAttitudeController controller(config);
    assert(controller.Is_Config_Valid());

    RearAttitudeInput in = NominalInput();
    in.pitch_deg = 5.0f;
    RearAttitudeOutput out = controller.Update(in);
    assert(out.left_torque_nm < 0.0f);
    assert(out.right_torque_nm > 0.0f);
    assert(fabsf(out.left_torque_nm + out.right_torque_nm) < 1.0e-5f);

    controller.Reset();
    in = NominalInput();
    in.roll_deg = 4.0f;
    out = controller.Update(in);
    const float normalized_left = out.left_torque_nm * config.motor_direction[0];
    const float normalized_right = out.right_torque_nm * config.motor_direction[1];
    assert(normalized_left * normalized_right < 0.0f);

    controller.Reset();
    in = NominalInput();
    in.pitch_deg = -100.0f;
    in.roll_deg = -100.0f;
    out = controller.Update(in);
    assert(fabsf(fabsf(out.left_torque_nm) - config.max_torque_nm) < 1.0e-5f);
    assert(fabsf(out.right_torque_nm) < 1.0e-5f);

    controller.Reset();
    in = NominalInput();
    in.enabled = false;
    out = controller.Update(in);
    assert(out.left_torque_nm == 0.0f);
    assert(out.right_torque_nm == 0.0f);

    controller.Reset();
    in = NominalInput();
    in.left_angle_rad = config.limit_start_rad[0];
    in.pitch_deg = 5.0f; // requests negative left torque, farther outward
    out = controller.Update(in);
    assert(out.left_torque_nm == 0.0f);
    assert(out.right_torque_nm != 0.0f);

    // The right interval crosses zero. At its upper endpoint, positive raw
    // motor torque must be blocked while the left motor remains available.
    controller.Reset();
    in = NominalInput();
    in.right_angle_rad = config.limit_end_rad[1];
    in.pitch_deg = 5.0f;
    out = controller.Update(in);
    assert(out.left_torque_nm != 0.0f);
    assert(out.right_torque_nm == 0.0f);

    RearAttitudeConfig slow = config;
    slow.torque_slew_per_cycle_nm = 0.5f;
    RearAttitudeController slow_controller(slow);
    in = NominalInput();
    in.pitch_deg = 5.0f;
    out = slow_controller.Update(in);
    assert(fabsf(fabsf(out.left_torque_nm) - 0.5f) < 1.0e-5f);
    assert(fabsf(fabsf(out.right_torque_nm) - 0.5f) < 1.0e-5f);

    RearAttitudeConfig invalid = TestConfig();
    invalid.limit_end_rad[0] = invalid.limit_start_rad[0];
    RearAttitudeController invalid_controller(invalid);
    assert(!invalid_controller.Is_Config_Valid());
    out = invalid_controller.Update(NominalInput());
    assert(out.left_torque_nm == 0.0f);
    assert(out.right_torque_nm == 0.0f);
    return 0;
}
```

- [ ] **Step 2: Run the test and verify the missing-controller failure**

```powershell
g++ -std=c++17 -I. tests/rear_attitude_controller_test.cpp user/core/Alg/PID/pid.cpp -o tests/build/rear_attitude_controller_test.exe
```

Expected: compilation fails because the rear controller files do not exist.

### Task 4: Implement the rear attitude controller

**Files:**
- Create: `fsm/rear_attitude_controller.hpp`
- Create: `fsm/rear_attitude_controller.cpp`
- Test: `tests/rear_attitude_controller_test.cpp`

- [ ] **Step 1: Create the controller interface**

Create `fsm/rear_attitude_controller.hpp`:

```cpp
#ifndef REAR_ATTITUDE_CONTROLLER_HPP
#define REAR_ATTITUDE_CONTROLLER_HPP

#include "../user/core/Alg/PID/pid.hpp"
#include <stdint.h>

struct RearAttitudeConfig
{
    float pitch_angle_kp;
    float pitch_rate_kp;
    float roll_angle_kp;
    float roll_rate_kp;
    float max_target_rate_dps;
    float max_torque_nm;
    float torque_slew_per_cycle_nm;
    float angle_deadband_deg;
    float limit_margin_rad;
    float pitch_zero_deg;
    float roll_zero_deg;
    float limit_start_rad[2];
    float limit_end_rad[2];
    float motor_direction[2];
};

struct RearAttitudeInput
{
    bool enabled;
    bool imu_valid;
    bool left_feedback_valid;
    bool right_feedback_valid;
    float pitch_deg;
    float roll_deg;
    float pitch_rate_dps;
    float roll_rate_dps;
    float left_angle_rad;
    float right_angle_rad;
};

struct RearAttitudeOutput
{
    float left_torque_nm;
    float right_torque_nm;
};

class RearAttitudeController
{
public:
    explicit RearAttitudeController(const RearAttitudeConfig &config);
    RearAttitudeOutput Update(const RearAttitudeInput &input);
    void Reset();
    bool Is_Config_Valid() const { return config_valid_; }
    bool Is_Angle_Valid(uint8_t index, float raw_angle) const;

private:
    bool Validate_Config() const;
    float To_Control_Angle(uint8_t index, float raw_angle) const;
    float Apply_Deadband(float value) const;
    float Slew(float requested, float previous) const;
    bool Blocks_Outward_Torque(uint8_t index,
                               float raw_angle,
                               float torque) const;

    RearAttitudeConfig config_;
    ALG::PID::PID pitch_angle_pid_;
    ALG::PID::PID pitch_rate_pid_;
    ALG::PID::PID roll_angle_pid_;
    ALG::PID::PID roll_rate_pid_;
    float previous_torque_[2];
    bool config_valid_;
};

#endif
```

- [ ] **Step 2: Implement validation, cascaded control, mixing, and protection**

Create `fsm/rear_attitude_controller.cpp` with the following implementation:

```cpp
#include "rear_attitude_controller.hpp"
#include <algorithm>
#include <math.h>

namespace
{
constexpr float TWO_PI = 6.28318530718f;
}

RearAttitudeController::RearAttitudeController(const RearAttitudeConfig &config)
    : config_(config),
      pitch_angle_pid_(config.pitch_angle_kp, 0.0f, 0.0f,
                       config.max_target_rate_dps, 0.0f, 0.0f),
      pitch_rate_pid_(config.pitch_rate_kp, 0.0f, 0.0f,
                      config.max_torque_nm, 0.0f, 0.0f),
      roll_angle_pid_(config.roll_angle_kp, 0.0f, 0.0f,
                      config.max_target_rate_dps, 0.0f, 0.0f),
      roll_rate_pid_(config.roll_rate_kp, 0.0f, 0.0f,
                     config.max_torque_nm, 0.0f, 0.0f),
      config_valid_(Validate_Config())
{
    previous_torque_[0] = 0.0f;
    previous_torque_[1] = 0.0f;
}

bool RearAttitudeController::Validate_Config() const
{
    if (config_.max_target_rate_dps <= 0.0f ||
        config_.max_torque_nm <= 0.0f ||
        config_.torque_slew_per_cycle_nm <= 0.0f ||
        config_.limit_margin_rad < 0.0f)
    {
        return false;
    }
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        if (config_.limit_start_rad[i] == config_.limit_end_rad[i] ||
            fabsf(config_.motor_direction[i]) != 1.0f)
        {
            return false;
        }
    }
    return true;
}

bool RearAttitudeController::Is_Angle_Valid(uint8_t index, float angle) const
{
    if (index >= 2U || !config_valid_)
    {
        return false;
    }
    const float start = config_.limit_start_rad[index];
    const float end = config_.limit_end_rad[index];
    return start < end ? (angle >= start && angle <= end)
                       : (angle >= start || angle <= end);
}

float RearAttitudeController::To_Control_Angle(uint8_t index, float angle) const
{
    return config_.limit_start_rad[index] > config_.limit_end_rad[index] &&
                   angle < config_.limit_start_rad[index]
               ? angle + TWO_PI
               : angle;
}

float RearAttitudeController::Apply_Deadband(float value) const
{
    return fabsf(value) <= config_.angle_deadband_deg ? 0.0f : value;
}

float RearAttitudeController::Slew(float requested, float previous) const
{
    const float delta = std::max(-config_.torque_slew_per_cycle_nm,
                                 std::min(config_.torque_slew_per_cycle_nm,
                                          requested - previous));
    return previous + delta;
}

bool RearAttitudeController::Blocks_Outward_Torque(uint8_t index,
                                                   float raw_angle,
                                                   float torque) const
{
    const float angle = To_Control_Angle(index, raw_angle);
    const float start = config_.limit_start_rad[index];
    const float end = config_.limit_start_rad[index] > config_.limit_end_rad[index]
                          ? config_.limit_end_rad[index] + TWO_PI
                          : config_.limit_end_rad[index];
    return (angle <= start + config_.limit_margin_rad && torque < 0.0f) ||
           (angle >= end - config_.limit_margin_rad && torque > 0.0f);
}

void RearAttitudeController::Reset()
{
    pitch_angle_pid_.reset();
    pitch_rate_pid_.reset();
    roll_angle_pid_.reset();
    roll_rate_pid_.reset();
    previous_torque_[0] = 0.0f;
    previous_torque_[1] = 0.0f;
}

RearAttitudeOutput RearAttitudeController::Update(const RearAttitudeInput &in)
{
    if (!in.enabled || !in.imu_valid || !config_valid_)
    {
        Reset();
        return {0.0f, 0.0f};
    }

    const float pitch = Apply_Deadband(in.pitch_deg - config_.pitch_zero_deg);
    const float roll = Apply_Deadband(in.roll_deg - config_.roll_zero_deg);
    const float pitch_rate_target = pitch_angle_pid_.UpDate(0.0f, pitch);
    const float roll_rate_target = roll_angle_pid_.UpDate(0.0f, roll);
    const float pitch_torque =
        pitch_rate_pid_.UpDate(pitch_rate_target, in.pitch_rate_dps);
    const float roll_torque =
        roll_rate_pid_.UpDate(roll_rate_target, in.roll_rate_dps);

    float torque[2] = {
        config_.motor_direction[0] * (pitch_torque + roll_torque),
        config_.motor_direction[1] * (pitch_torque - roll_torque)};

    const float peak = std::max(fabsf(torque[0]), fabsf(torque[1]));
    if (peak > config_.max_torque_nm)
    {
        const float scale = config_.max_torque_nm / peak;
        torque[0] *= scale;
        torque[1] *= scale;
    }

    torque[0] = Slew(torque[0], previous_torque_[0]);
    torque[1] = Slew(torque[1], previous_torque_[1]);

    const bool valid[2] = {in.left_feedback_valid, in.right_feedback_valid};
    const float angle[2] = {in.left_angle_rad, in.right_angle_rad};
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        if (!valid[i] || !Is_Angle_Valid(i, angle[i]) ||
            Blocks_Outward_Torque(i, angle[i], torque[i]))
        {
            torque[i] = 0.0f;
        }
        previous_torque_[i] = torque[i];
    }

    return {torque[0], torque[1]};
}
```

- [ ] **Step 3: Run the rear controller test**

```powershell
g++ -std=c++17 -I. tests/rear_attitude_controller_test.cpp fsm/rear_attitude_controller.cpp user/core/Alg/PID/pid.cpp -o tests/build/rear_attitude_controller_test.exe
tests/build/rear_attitude_controller_test.exe
```

Expected: exit code 0.

- [ ] **Step 4: Commit the controller unit**

```powershell
git add fsm/rear_attitude_controller.hpp fsm/rear_attitude_controller.cpp tests/rear_attitude_controller_test.cpp
git commit -m "feat: add rear pitch roll attitude controller"
```

### Task 5: Publish a trustworthy switch-link heartbeat

**Files:**
- Modify: `RtosTask/can_send_task.hpp:52-60`
- Modify: `RtosTask/can_send_task.cpp:44-57,276-281`
- Create: `tests/switch_heartbeat_contract_test.ps1`

- [ ] **Step 1: Add a failing source-contract test**

Create `tests/switch_heartbeat_contract_test.ps1`:

```powershell
$header = Get-Content -Raw 'RtosTask/can_send_task.hpp'
$source = Get-Content -Raw 'RtosTask/can_send_task.cpp'

if ($header -notmatch 'extern volatile uint32_t gimbal_switch_last_tick') { exit 1 }
if ($header -notmatch 'extern volatile bool gimbal_switch_received') { exit 1 }
if ($source -notmatch 'gimbal_switch_last_tick\s*=\s*HAL_GetTick\(\)') { exit 1 }
if ($source -notmatch 'gimbal_switch_received\s*=\s*true') { exit 1 }
exit 0
```

Run:

```powershell
& tests/switch_heartbeat_contract_test.ps1
```

Expected: exit code 1.

- [ ] **Step 2: Declare and define the heartbeat**

Add to `RtosTask/can_send_task.hpp`:

```cpp
extern volatile uint32_t gimbal_switch_last_tick;
extern volatile bool gimbal_switch_received;
```

Add beside the keyboard globals in `RtosTask/can_send_task.cpp`:

```cpp
volatile uint32_t gimbal_switch_last_tick = 0U;
volatile bool gimbal_switch_received = false;
```

Update the `0x303` receive branch:

```cpp
else if (frame.id == 0x303 && frame.dlc >= 2U)
{
    gimbalChassis_communicate.s1 = frame.data[0];
    gimbalChassis_communicate.s2 = frame.data[1];
    gimbal_switch_last_tick = HAL_GetTick();
    gimbal_switch_received = true;
}
```

- [ ] **Step 3: Run the heartbeat contract test**

```powershell
& tests/switch_heartbeat_contract_test.ps1
```

Expected: exit code 0.

- [ ] **Step 4: Commit the heartbeat**

```powershell
git add RtosTask/can_send_task.hpp RtosTask/can_send_task.cpp tests/switch_heartbeat_contract_test.ps1
git commit -m "feat: track stair switch link freshness"
```

### Task 6: Integrate both controllers into `up_stair_task`

**Files:**
- Modify: `RtosTask/up_stair.hpp:5-16`
- Modify: `RtosTask/up_stair.cpp:1-132`
- Modify: `RtosTask/can_send_task.cpp:290-305`
- Create: `tests/up_stair_integration_contract_test.ps1`

- [ ] **Step 1: Add a failing integration contract test**

Create `tests/up_stair_integration_contract_test.ps1`:

```powershell
$source = Get-Content -Raw 'RtosTask/up_stair.cpp'

$required = @(
    'EvaluateStairModePolicy',
    'gimbal_switch_last_tick',
    'RearAttitudeController',
    'bmi088.GetPitchAngleDeg',
    'bmi088.GetRollAngleDeg',
    'bmi088.GetGyroRateYDps',
    'bmi088.GetGyroRateXDps',
    'rear_6248.ctrl_Mit',
    'front_hold_enabled',
    'rear_attitude_enabled'
)

foreach ($pattern in $required)
{
    if ($source -notmatch [regex]::Escape($pattern)) { exit 1 }
}
exit 0
```

Run:

```powershell
& tests/up_stair_integration_contract_test.ps1
```

Expected: exit code 1.

- [ ] **Step 2: Add the production rear configuration and includes**

In `RtosTask/up_stair.cpp`, include:

```cpp
#include "imu_task.hpp"
#include "../fsm/rear_attitude_controller.hpp"
#include "../fsm/stair_mode_policy.hpp"
```

Create the configuration through a function so C++11 global initialization is
unambiguous:

```cpp
static RearAttitudeConfig MakeRearAttitudeConfig()
{
    RearAttitudeConfig c = {};
    c.pitch_angle_kp = 1.0f;
    c.pitch_rate_kp = 0.1f;
    c.roll_angle_kp = 1.0f;
    c.roll_rate_kp = 0.1f;
    c.max_target_rate_dps = 20.0f;
    c.max_torque_nm = 3.0f;
    c.torque_slew_per_cycle_nm = 0.02f;
    c.angle_deadband_deg = 0.5f;
    c.limit_margin_rad = 3.0f * 3.14159265359f / 180.0f;
    c.pitch_zero_deg = 0.0f;
    c.roll_zero_deg = 0.0f;

    // Equal start/end values deliberately make the configuration invalid.
    // Rear torque remains zero until measured safe intervals are installed.
    c.limit_start_rad[0] = 0.0f;
    c.limit_start_rad[1] = 0.0f;
    c.limit_end_rad[0] = 0.0f;
    c.limit_end_rad[1] = 0.0f;
    c.motor_direction[0] = 1.0f;
    c.motor_direction[1] = -1.0f;
    return c;
}

RearAttitudeController rear_attitude_controller(MakeRearAttitudeConfig());
```

The invalid production limit configuration is a deliberate interlock, not a
usable calibration. Replace it only with measured left/right safe intervals
after low-torque direction verification.

- [ ] **Step 3: Enable and recover all four DM motors**

Replace the single recovery object with:

```cpp
MotorRecoveryFSM front_recovery_fsm[2];
MotorRecoveryFSM rear_recovery_fsm[2];
```

Remove the one-shot `front_4340.On(1, ...)` call from `can_send_task` and the
one-shot `front_4340.On(2, ...)` call from `up_stair_task`; the four recovery
objects become the sole owners of MIT enable requests.

Initialize all four before the loop:

```cpp
const uint32_t init_tick = HAL_GetTick();
for (uint8_t i = 0U; i < 2U; ++i)
{
    front_recovery_fsm[i].Init(init_tick);
    rear_recovery_fsm[i].Init(init_tick);
}
```

At the top of each loop, sample all feedback into fixed arrays:

```cpp
const float front_angle[2] = {
    front_4340.getAngleRad(1U), front_4340.getAngleRad(2U)};
const float front_velocity[2] = {
    front_4340.getVelocityRads(1U), front_4340.getVelocityRads(2U)};
const bool front_online[2] = {
    front_4340.isConnected(1U, 1U), front_4340.isConnected(2U, 2U)};
const bool front_valid[2] = {
    front_online[0] && up_stair_fsm.Is_Angle_Valid(1U, front_angle[0]),
    front_online[1] && up_stair_fsm.Is_Angle_Valid(2U, front_angle[1])};

const float rear_angle[2] = {
    rear_6248.getAngleRad(1U), rear_6248.getAngleRad(2U)};
const bool rear_online[2] = {
    rear_6248.isConnected(1U, 3U), rear_6248.isConnected(2U, 4U)};
const bool rear_valid[2] = {
    rear_online[0] && rear_attitude_controller.Is_Angle_Valid(0U, rear_angle[0]),
    rear_online[1] && rear_attitude_controller.Is_Angle_Valid(1U, rear_angle[1])};
```

After the double-down/offline zero-torque branch in Step 4, service enable
recovery for all four motors:

```cpp
for (uint8_t id = 1U; id <= 2U; ++id)
{
    if (front_recovery_fsm[id - 1U].Should_Enable(front_online[id - 1U], now_tick))
    {
        front_4340.On(id, BSP::Motor::DM::Model::MIT);
    }
    if (rear_recovery_fsm[id - 1U].Should_Enable(rear_online[id - 1U], now_tick))
    {
        rear_6248.On(id, BSP::Motor::DM::Model::MIT);
    }
}
```

- [ ] **Step 4: Evaluate the safety policy before either controller**

Use a 100 ms switch heartbeat and the independent keyboard heartbeat:

```cpp
static constexpr uint32_t CONTROL_LINK_TIMEOUT_MS = 100U;

const bool control_link_online =
    gimbal_switch_received &&
    now_tick - gimbal_switch_last_tick < CONTROL_LINK_TIMEOUT_MS;
const bool keyboard_online =
    gimbal_keyboard_received &&
    now_tick - gimbal_keyboard_last_tick < 100U;

const StairModePolicy policy = EvaluateStairModePolicy(
    static_cast<uint8_t>(gimbalChassis_communicate.s1),
    static_cast<uint8_t>(gimbalChassis_communicate.s2),
    control_link_online,
    keyboard_online);

up_stair_fsm.Update(front_angle[0], front_angle[1],
                    front_valid[0], front_valid[1],
                    policy.front_hold_enabled,
                    policy.front_stair_command_enabled,
                    stair_action_sequence);
```

When `policy.zero_all_torque` is true, execute this branch before motor recovery
or any normal PID update:

```cpp
if (policy.zero_all_torque)
{
    front_4340_left_pid[0].reset();
    front_4340_left_pid[1].reset();
    front_4340_right_pid[0].reset();
    front_4340_right_pid[1].reset();
    rear_attitude_controller.Reset();

    front_4340.ctrl_Mit(1U, front_angle[0], 0.0f, 0.0f, 0.0f, 0.0f);
    front_4340.ctrl_Mit(2U, front_angle[1], 0.0f, 0.0f, 0.0f, 0.0f);
    rear_6248.ctrl_Mit(1U, rear_angle[0], 0.0f, 0.0f, 0.0f, 0.0f);
    rear_6248.ctrl_Mit(2U, rear_angle[1], 0.0f, 0.0f, 0.0f, 0.0f);
    osDelay(1U);
    continue;
}
```

- [ ] **Step 5: Keep front home hold active in every non-double-down mode**

Run the existing left and right position/velocity cascades whenever the FSM is
enabled. Use the continuous control coordinate for the position PID and the raw
encoder angle in the MIT position field:

```cpp
if (front_valid[0] && up_stair_fsm.Is_Enabled())
{
    const float velocity_target = front_4340_left_pid[0].UpDate(
        up_stair_fsm.Get_Target_Angle(1U),
        up_stair_fsm.Get_Position_Feedback(1U));
    const float torque_target = front_4340_left_pid[1].UpDate(
        velocity_target, front_velocity[0]);
    front_4340.ctrl_Mit(1U, front_angle[0], 0.0f, 0.0f, 0.0f,
                        torque_target);
}
else
{
    front_4340_left_pid[0].reset();
    front_4340_left_pid[1].reset();
    front_4340.ctrl_Mit(1U, front_angle[0], 0.0f, 0.0f, 0.0f, 0.0f);
}

if (front_valid[1] && up_stair_fsm.Is_Enabled())
{
    const float velocity_target = front_4340_right_pid[0].UpDate(
        up_stair_fsm.Get_Target_Angle(2U),
        up_stair_fsm.Get_Position_Feedback(2U));
    const float torque_target = front_4340_right_pid[1].UpDate(
        velocity_target, front_velocity[1]);
    front_4340.ctrl_Mit(2U, front_angle[1], 0.0f, 0.0f, 0.0f,
                        torque_target);
}
else
{
    front_4340_right_pid[0].reset();
    front_4340_right_pid[1].reset();
    front_4340.ctrl_Mit(2U, front_angle[1], 0.0f, 0.0f, 0.0f, 0.0f);
}
```

Invalid feedback sends zero torque only to that motor and resets its two PID
objects.

- [ ] **Step 6: Feed BMI088 and rear feedback into the attitude controller**

Build one input per loop:

```cpp
RearAttitudeInput rear_input = {};
rear_input.enabled = policy.rear_attitude_enabled;
rear_input.imu_valid = bmi088.IsReady();
rear_input.left_feedback_valid = rear_valid[0];
rear_input.right_feedback_valid = rear_valid[1];
rear_input.pitch_deg = bmi088.GetPitchAngleDeg();
rear_input.roll_deg = bmi088.GetRollAngleDeg();
rear_input.pitch_rate_dps = bmi088.GetGyroRateYDps();
rear_input.roll_rate_dps = bmi088.GetGyroRateXDps();
rear_input.left_angle_rad = rear_angle[0];
rear_input.right_angle_rad = rear_angle[1];

const RearAttitudeOutput rear_output =
    rear_attitude_controller.Update(rear_input);

rear_6248.ctrl_Mit(1U, rear_angle[0], 0.0f, 0.0f, 0.0f,
                   rear_output.left_torque_nm);
rear_6248.ctrl_Mit(2U, rear_angle[1], 0.0f, 0.0f, 0.0f,
                   rear_output.right_torque_nm);
```

Because the production configuration starts invalid, both `rear_valid` values
and both torque outputs remain zero until the measured safe intervals are
installed.

- [ ] **Step 7: Run the integration contract test**

```powershell
& tests/up_stair_integration_contract_test.ps1
```

Expected: exit code 0.

- [ ] **Step 8: Commit only the intended integration hunks**

`RtosTask/up_stair.cpp`, `RtosTask/up_stair.hpp`, and the Keil files already
contain user changes. Inspect each diff and stage only task-owned hunks:

```powershell
git diff -- RtosTask/up_stair.cpp RtosTask/up_stair.hpp RtosTask/can_send_task.cpp
git add -p RtosTask/up_stair.cpp RtosTask/up_stair.hpp RtosTask/can_send_task.cpp
git add tests/up_stair_integration_contract_test.ps1
git commit -m "feat: integrate stair attitude control"
```

### Task 7: Register sources and run the complete verification set

**Files:**
- Modify: `MDK-ARM/tank_chassis.uvprojx`
- Test: all mechanism tests

- [ ] **Step 1: Add the new files to the existing FSM group**

Add these entries next to `up_stair_fsm.cpp/.hpp`:

```xml
<File>
  <FileName>rear_attitude_controller.cpp</FileName>
  <FileType>8</FileType>
  <FilePath>..\fsm\rear_attitude_controller.cpp</FilePath>
</File>
<File>
  <FileName>rear_attitude_controller.hpp</FileName>
  <FileType>5</FileType>
  <FilePath>..\fsm\rear_attitude_controller.hpp</FilePath>
</File>
<File>
  <FileName>stair_mode_policy.hpp</FileName>
  <FileType>5</FileType>
  <FilePath>..\fsm\stair_mode_policy.hpp</FilePath>
</File>
```

- [ ] **Step 2: Run all host tests**

```powershell
g++ -std=c++11 -I. tests/stair_mode_policy_test.cpp -o tests/build/stair_mode_policy_test.exe
tests/build/stair_mode_policy_test.exe

g++ -std=c++11 -I. tests/up_stair_fsm_test.cpp fsm/up_stair_fsm.cpp user/core/Alg/FSM/alg_fsm.cpp -o tests/build/up_stair_fsm_test.exe
tests/build/up_stair_fsm_test.exe

g++ -std=c++17 -I. tests/rear_attitude_controller_test.cpp fsm/rear_attitude_controller.cpp user/core/Alg/PID/pid.cpp -o tests/build/rear_attitude_controller_test.exe
tests/build/rear_attitude_controller_test.exe

g++ -std=c++11 -I. tests/motor_recovery_fsm_test.cpp -o tests/build/motor_recovery_fsm_test.exe
tests/build/motor_recovery_fsm_test.exe

& tests/switch_heartbeat_contract_test.ps1
& tests/up_stair_integration_contract_test.ps1
```

Expected: every command exits with code 0 and no assertion failure.

- [ ] **Step 3: Build the Keil target**

```powershell
& 'D:\Keil5\Core\UV4\UV4.exe' -b MDK-ARM/tank_chassis.uvprojx -j0 -o MDK-ARM/up_stair_attitude_build.log
```

Expected: build log reports 0 errors. Warnings introduced by this feature must
be resolved before proceeding.

- [ ] **Step 4: Inspect the final diff and commit the project registration**

```powershell
git diff --check
git diff --stat
git add -p MDK-ARM/tank_chassis.uvprojx
git commit -m "build: register rear attitude controller"
```

## Hardware commissioning gate

Do not raise the torque limit above the initial 3 N·m or load the stair
mechanism until all items below are complete:

- [ ] Measure both J6248 safe raw-angle intervals and replace the deliberately
      invalid production intervals.
- [ ] Verify whether each interval crosses `0/2*pi`.
- [ ] Verify both motor direction signs with the chassis supported.
- [ ] Record level `pitch_zero_deg` and `roll_zero_deg` in the normal posture.
- [ ] Verify nose-up produces rear motion that reduces pitch error.
- [ ] Verify left-low and right-low disturbances produce the correct differential response.
- [ ] Verify double-down produces zero torque within one control cycle from every front state.
- [ ] Verify double-middle exit returns both J4310 motors home and continues active home hold.
- [ ] Verify normal acceleration and braking do not cause excessive rear-arm travel.

After commissioning, change one gain or limit at a time, repeat the host and
firmware build checks, and record the measured value in the production
configuration rather than deriving it from the power-on position.
