# Rear IMU damping implementation plan

**Goal:** Add Pitch-rate damping through the rear MIT torque feedforward field.

**Architecture:** Keep the angle P loop and MIT speed gain. Calculate
`raw_ff[i] = -direction[i] * REAR_IMU_PITCH_DAMPING * pitch_rate_dps * pi / 180`.
Pass this raw feedforward to `Limit_Velocity` through an optional fourth argument
(default zero). In attitude mode, protect both the total requested torque and
the sent feedforward, then subtract the sent feedforward before reconstructing
the speed target. Retraction retains its existing configured feedforward.

**Tech Stack:** Existing C++ FSM/task, Python task-branch harness, host g++, Keil.

- [x] Extend `tests/rear_soft_limit_task_test.py` to execute the real task branch
  with positive/negative Pitch rates, both motor directions, recovery scaling,
  endpoint cancellation, torque saturation, faults and retraction. Run it and
  observe failure because attitude feedforward remains zero.
- [x] Add `float REAR_IMU_PITCH_DAMPING = 1.0f` beside the mode-specific MIT gains
  in `RtosTask/up_stair.cpp`; document N*m/(rad/s) and zero-to-disable. Compute
  raw feedforward per side, protect it with `Limit_Torque` and
  `ClampJ6248Torque`, and send it with the matching speed command.
- [x] Extend the declaration/definition of `Limit_Velocity` in
  `fsm/up_stair_behind_motor_fsm.hpp/.cpp` with the optional attitude feedforward.
  Preserve the existing total-torque protection and reconstruct speed using
  the same protected feedforward that the task sends.
- [x] Update the integration contract for the fourth argument and run
  `python tests/rear_soft_limit_task_test.py`, rear FSM/limit/loss/retract host
  tests, `python tests/rear_imu_filter_test.py`, and the PowerShell contract.
- [x] Build with `D:/Keil5/Core/UV4/UV4.exe -b tank_chassis.uvprojx -j0 -o
  rear_imu_damping_build.log` in `MDK-ARM`; inspect the fresh result and review
  changes against the saved turn-start snapshot. No flashing or commit.

The gain is an initial tuning value, not a physically validated setting. Host
tests check command algebra and protection, not closed-loop vehicle overshoot.

Verification: The task harness failed before implementation on the new nonzero
attitude feedforward assertion, then passed with production gains, distinct MIT
gains plus damping 0.7, and damping disabled. The soft-limit, IMU-loss, retract,
IMU-filter and integration-contract tests passed. The old
`up_stair_behind_motor_fsm_test.cpp:82` assertion still expects a 94.7 degree
endpoint while the active header is 94.04 degrees; the same failure was
reproduced using the unmodified turn-start FSM source/header snapshot. This
pre-existing calibration assertion was left untouched. Keil
`MDK-ARM/rear_imu_damping_build.log` reports 0 errors and 5 warnings; target AXF
and HEX were generated. Source/test diff checks passed. No flash or commit.
