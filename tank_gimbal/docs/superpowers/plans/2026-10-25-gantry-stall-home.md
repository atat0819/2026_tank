# Gantry Stall Home Implementation Plan

**Goal:** Remove the lift bottom switch, use current/speed feedback to confirm a return to the bottom, and zero both rigidly coupled lift motors together.

**Architecture:** Keep the existing five-stage FSM and direct task loop, including its current `osDelay(2)`. The user guarantees the lift rests at its mechanical bottom when firmware starts; capture the first valid low-speed encoder snapshot as the software home reference, without commanding a startup descent. Retain the four other switches, existing safety gates, DM recovery and no-Off behavior.

**Tech stack:** STM32H723, FreeRTOS, C++17, C620 CAN feedback, host C++ fixtures, Keil armclang.

## Accepted behavior

- The two 3508 motors drive one rigid mechanism; never stop them independently during homing.
- Keep accumulated angles in deg, velocity in output-shaft rad/s, measured torque current in A, and command current in the existing raw units.
- Extend the scalar FSM Update arguments with the two measured currents and velocities; do not add input/output wrappers.
- Record the boot home once, including while the remote/control gate is closed. Later mode changes and faults must not silently redefine it.
- Starting a cycle requires the DM retract switch and both lift encoders near the saved home with low speed.
- In Lower, either motor exceeding the current threshold while both speeds are low starts a continuous confirmation timer. A broken condition resets the timer.
- Confirmed stall near the saved home completes Lower; confirmed stall elsewhere latches a Stall fault. Keep timeout, travel and mismatch checks ahead of successful completion.
- After completion, the lift motors are neither moving nor actively keeping position; reset their PIDs and send zero current. The DM retains its normal holding path.
- Limit Lower current separately. All new thresholds default to zero and invalidate configuration until the user supplies measured values.
- Only PA0..PA3 are sampled. Remove PA4 from GPIO/CubeMX configuration and the Watch switch array.

## Files

- `feeder_fsm/gantry_fsm.hpp/.cpp`: threshold validation, home reference, continuous stall detection, completion/fault semantics.
- `RtosTask/gantry_task.hpp/.cpp`: four switch inputs, measured-current Watch snapshot, scalar FSM arguments, Lower command limit, idle lift PID reset.
- `Core/Src/gpio.c`, `tank_gimbal.ioc`: remove PA4 input configuration only.
- `tests/gantry_fsm_test.cpp`, `tests/gantry_fsm_test.py`: adapt the sequence and add homing/fault/boundary cases.
- `tests/gantry_motor_definition_test.py`: check actual current conversion, four-pin sampling, current limiting and zero lift output after completion while DM holds.
- `docs/gantry_auto.md`: document the new endpoint, boot assumption, parameters and measured-versus-command units.

## Steps and verification

- [x] Add regression cases before implementation. An old FSM may only accept switch completion; the new contract must reject transient current spikes, movement, early jams and stale feedback.
- [x] Run `python -B tests/gantry_fsm_test.py` and observe a failure against the old scalar signature.
- [x] Implement the FSM changes with these adjustable parameters:

```cpp
static constexpr float LOWER_CURRENT_LIMIT_A = 0.0f;
static constexpr float LOWER_STALL_CURRENT_A = 0.0f;
static constexpr float LOWER_STALL_SPEED_RAD_S = 0.0f;
static constexpr uint32_t LOWER_STALL_CONFIRM_MS = 0;
static constexpr double LIFT_HOME_TOLERANCE_DEG = 0.0;
```

- [x] Require finite positive thresholds, stall current below the Lower current limit, a current limit no greater than 20 A, low-speed threshold below the final Lower target speed, and confirmation time shorter than the Lower timeout.
- [x] Run the FSM fixtures until all sequences, stall confirmation/reset, wraparound, home persistence and invalid configurations pass.
- [x] Extend the existing task fixture before adapting the task. Check positive/negative measured current, zero PID state on completion, no Off frames, and unchanged gated On retries.
- [x] Adapt task sampling/output and GPIO/CubeMX configuration; run `python -B tests/gantry_motor_definition_test.py`.
- [x] Update the operation document and review the scoped diff against the pre-change backup.
- [x] Run `python -B tests/gantry_fsm_test.py` and `python -B -m unittest tests.gimbal_motor_recovery_test tests.gantry_input_test`.
- [x] Build the existing `tank_gimbal` Keil target without flashing. Inspect the link result and log error/warning counts, rather than treating the UV4 process code as proof.
- [x] Request focused code review of the homing logic and regression cases; resolve material findings before reporting completion.

No commits, flashing, motor operation or memory updates are part of this task. Hardware current/speed/time thresholds, contact forces and load behavior remain unvalidated.

## Verified result

FSM 14 fixtures and task/driver 8 fixtures passed; shared recovery/input checks passed. Independent review identified delayed home capture behind DM readiness; the separate lift-feedback gate and regressions resolved it, and follow-up review found no remaining material issue. Keil linked with 0 errors and 16 warnings; no flashing or hardware validation performed.
