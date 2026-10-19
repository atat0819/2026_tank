# Rear small Roll compensation

**Goal:** Add weak Roll angle restoring torque during rear IMU attitude control.

**Current ownership:** The subsequent `2026-10-19-rear-roll-fsm.md` migration moves
parameters and correction into the FSM. Current production tuning is target 1.5
deg, gain 4.0 Nm/deg and cap 5.0 Nm; initial task-local settings below are history.

**Confirmed calibration:** Raw HI14 Roll is 1.5 degrees when level and increases
when the vehicle's left side rises. The user corrected their direction report and
confirmed the original mixing: add Roll on the left and subtract it on the right
before the existing motor direction mapping. With directions {1, -1}, left-high
correction adds positive shaft torque on both sides. Host tests verify commanded
signs; vehicle observations are supplied by the user.

**Design:** Publish Roll with Pitch in the existing atomic UART7 snapshot. Reject
nonfinite Roll using the existing IMU-invalid path. Filter Roll once per new frame
with the existing first-order angle weight 0.09, seeding on startup/recovery.
Use independent, visible task parameters: target 1.5 deg, proportional gain
0.05 Nm/deg, compensation cap 0.5 Nm per motor. No Roll integral or rate term.

```text
roll = clamp(Kp * (filtered_roll - target), -cap, cap)
left_raw_ff  = direction_left  * (pitch_damping + roll)
right_raw_ff = direction_right * (pitch_damping - roll)
```

Both combined raw feedforwards must enter Limit_Torque and Limit_Velocity;
the existing final send uses protected feedforward. This keeps fault recovery,
total torque limits and directional endpoint guards consistent. Roll is absent
from retract/disabled output branches. Existing Pitch tuning and VOFA layout stay
as they are. No commit or flashing is part of this work.

## Implementation and acceptance

- [x] Extend `tests/hi14_uart7_test.py` to check published Roll from decoded frames;
  run before implementation and observe the missing-field-copy failure.
- [x] Extend snapshot, filter and task-output tests for Roll publication, NaN,
  reseeding, repeated timestamps, sign, zero gain, saturation, MIT Kd independence,
  combined damping, endpoint protection, recovery and retract isolation.
- [x] Add `roll_deg` in `RtosTask/remote_task.hpp`, populate it in the UART7 callback
  and `PublishHi14ImuFromISR` in `RtosTask/remote_task.cpp`.
- [x] Add Roll filter, three tunables and bounded differential feedforward in
  `RtosTask/up_stair.cpp`; keep the existing FSM API and protection implementation.
- [x] Update the two integration contracts whose former Pitch-only expectations
  are superseded by the user's Roll request.
- [x] Run the affected Python host tests and PowerShell contracts, then Keil build
  and check task diff for unrelated changes. Report software checks separately
  from unperformed loaded-vehicle validation.

## Verification results

- Before implementation, `python tests/hi14_uart7_test.py` failed at
  `data.roll==ImuFloat.roll`, demonstrating the missing callback copy.
- After implementation, `hi14_uart7_test.py`, `imu_snapshot_test.py`,
  `rear_imu_filter_test.py`, `rear_soft_limit_task_test.py` (three configurations),
  and `vofa_send_test.py` passed.
- `up_stair_integration_contract_test.ps1` and `imu_snapshot_contract_test.ps1`
  passed using PowerShell's process-local execution-policy option.
- Keil build log: `MDK-ARM/rear_roll_build.log`: 0 errors, 5 existing
  `RemoteData_t` typedef linkage warnings. HEX generated, no flashing performed.
- Targeted `git diff --check` passed for the affected source/tests. The full
  checkout check reports whitespace in pre-existing/generated MDK artifacts;
  those unrelated artifacts were not cleaned up.
- No loaded-vehicle validation was performed by the agent. The user's latest
  correction confirms the original mixing and supersedes their reversed report.
- Restoring the original direction preserves the user's current tuning: Roll Kp 8.0 Nm/deg
  and compensation cap 10.0 Nm. Tests derive magnitudes from active tuning.
