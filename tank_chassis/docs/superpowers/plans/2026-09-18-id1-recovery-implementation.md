# ID1 Motor Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep ID1 recoverable after a feedback timeout by issuing safe zero-torque MIT probes and rate-limited enable retries until feedback returns.

**Architecture:** A hardware-independent `MotorRecoveryFSM` decides whether the current tick requires an MIT enable command. It retries only while feedback is offline, at a 100 ms interval, and immediately stops emitting enable requests once feedback is online. `up_stair_task` lets this helper exclusively own ID1 enable requests and always sends the existing zero-torque MIT command in its ID1 fallback branch.

**Tech Stack:** C++11, CMSIS `HAL_GetTick`, existing host `assert` tests, Keil MDK project.

---

### Task 1: Add a failing host test for recovery enable behavior

**Files:**
- Create: `tests/motor_recovery_fsm_test.cpp`
- Create later: `fsm/motor_recovery_fsm.hpp`

- [ ] **Step 1: Write the failing test**

```cpp
#include "../fsm/motor_recovery_fsm.hpp"
#include <assert.h>

int main()
{
    MotorRecoveryFSM fsm;
    fsm.Init(0U);

    assert(fsm.Should_Enable(false, 0U));
    assert(!fsm.Should_Enable(false, 1U));
    assert(!fsm.Should_Enable(false, 99U));
    assert(fsm.Should_Enable(false, 100U));
    assert(!fsm.Should_Enable(true, 101U));
    assert(!fsm.Should_Enable(true, 200U));
    assert(fsm.Should_Enable(false, 201U));
    return 0;
}
```

- [ ] **Step 2: Run the test to verify it fails because the state machine does not yet exist**

Run:

```powershell
g++ -std=c++11 -I. tests/motor_recovery_fsm_test.cpp -o tests/build/motor_recovery_fsm_test.exe
```

Expected: compilation fails because `fsm/motor_recovery_fsm.hpp` is absent.

### Task 2: Implement and verify the recovery state machine

**Files:**
- Create: `fsm/motor_recovery_fsm.hpp`
- Test: `tests/motor_recovery_fsm_test.cpp`

- [ ] **Step 1: Add the minimal header-only implementation**

```cpp
class MotorRecoveryFSM
{
public:
    static constexpr uint32_t ENABLE_RETRY_MS = 100U;

    void Init(uint32_t now_tick)
    {
        last_enable_tick_ = now_tick - ENABLE_RETRY_MS;
        enable_requested_ = false;
    }

    bool Should_Enable(bool feedback_online, uint32_t now_tick)
    {
        if (feedback_online)
        {
            enable_requested_ = false;
            return false;
        }

        if (!enable_requested_ || now_tick - last_enable_tick_ >= ENABLE_RETRY_MS)
        {
            last_enable_tick_ = now_tick;
            enable_requested_ = true;
            return true;
        }

        return false;
    }

private:
    uint32_t last_enable_tick_ = 0U;
    bool enable_requested_ = false;
};
```

- [ ] **Step 2: Run the recovery-state test**

Run:

```powershell
g++ -std=c++11 -I. tests/motor_recovery_fsm_test.cpp -o tests/build/motor_recovery_fsm_test.exe
tests/build/motor_recovery_fsm_test.exe
```

Expected: exit code 0.

### Task 3: Connect ID1 fallback control to the recovery state machine

**Files:**
- Modify: `RtosTask/up_stair.cpp`
- Modify: `MDK-ARM/tank_chassis.uvprojx`

- [ ] **Step 1: Include and initialize `MotorRecoveryFSM` before entering the control loop**

```cpp
#include "../fsm/motor_recovery_fsm.hpp"

MotorRecoveryFSM front_left_recovery_fsm;

// After up_stair_fsm.Init(...):
front_left_recovery_fsm.Init(HAL_GetTick());
```

- [ ] **Step 2: Make the fallback always send a zero-torque MIT probe and only enable when requested**

```cpp
front_4340.ctrl_Mit(1, front_left_angle, 0.0f, 0.0f, 0.0f, 0.0f);

if (front_left_recovery_fsm.Should_Enable(left_online, now_tick))
{
    front_4340.On(1, BSP::Motor::DM::Model::MIT);
}
```

- [ ] **Step 3: Add the new header to the Keil FSM project group**

Add `motor_recovery_fsm.hpp` as a header file in the existing `fsm` group. The implementation is header-only, so no `.cpp` entry is required.

### Task 4: Verify regression coverage and firmware compilation

**Files:**
- Test: `tests/motor_recovery_fsm_test.cpp`
- Test: `tests/up_stair_fsm_test.cpp`

- [ ] **Step 1: Run both host tests**

```powershell
g++ -std=c++11 -I. tests/motor_recovery_fsm_test.cpp -o tests/build/motor_recovery_fsm_test.exe
tests/build/motor_recovery_fsm_test.exe
g++ -std=c++11 -I. tests/up_stair_fsm_test.cpp fsm/up_stair_fsm.cpp user/core/Alg/FSM/alg_fsm.cpp -o tests/build/up_stair_fsm_test.exe
tests/build/up_stair_fsm_test.exe
```

- [ ] **Step 2: Build the Keil target and inspect the result**

```powershell
& 'D:\Keil5\Core\UV4\UV4.exe' -b MDK-ARM/tank_chassis.uvprojx -j0 -o MDK-ARM/build_verify.log
```

Expected: build reports 0 errors.
