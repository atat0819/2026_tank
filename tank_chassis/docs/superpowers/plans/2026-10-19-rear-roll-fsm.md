# Move rear Roll compensation into the existing FSM

**Goal:** The rear FSM owns Roll calibration, bounded proportional correction,
mode gating and per-motor direction mapping; preserve current output behavior.

**Design:** Move the task's three tunables to visible FSM header constants and
Config fields: raw target 1.5 deg, gain 4.0 Nm/deg, cap 5.0 Nm. WithAngleConstants
loads them. Follow existing configuration validation and reject nonfinite values
or negative gain/cap. Add filtered raw Roll as the trailing Update argument, with
the production target as its default for existing Pitch-only callers. Store the
feedback only when IMU data is valid. Invalid Roll uses the existing IMU failure
path. Do not introduce states, setters or a second copy of the calibration.

`Get_Raw_Roll_Torque(id)` returns the per-motor, direction-mapped Roll correction
before recovery scaling, motor gain and endpoint protection. It is zero outside
attitude modes, for invalid ids, or if either motor is uncontrollable. Left adds
the bounded Roll error; right subtracts it before motor-direction mapping.

The task keeps IMU acquisition/filtering, Pitch PID/damping and motor sending.
Combine direction-mapped Pitch damping with Get_Raw_Roll_Torque once, then pass
that same raw sum to Limit_Torque and Limit_Velocity as before.

## Steps and acceptance

- [x] Extend host tests for FSM configuration ownership, raw Roll output, mode
  gating, invalid configuration/feedback, recovery and direction mapping.
- [x] Move constants/calculation into `fsm/up_stair_behind_motor_fsm.hpp/.cpp`.
- [x] Update the task's Update call and replace inline Roll calculation by getter.
- [x] Adapt task tests to FSM configuration and retain final-command sign, cap,
  Kd independence, recovery and retract checks; verify no Roll parameter remains
  in the task. Keep all unrelated calibration and dirty changes.
- [x] Run focused FSM, task, preprocessing and integration checks; Keil build.
  No commit or flashing. Changes to header constants take effect after rebuilding;
  Config is copied into the FSM at construction, as for existing mechanical data.

## Results

- The new `rear_roll_fsm_test.cpp` initially failed to compile because the FSM
  did not yet expose Roll configuration, input or output; it passes after migration.
- Passed: new Roll FSM test, `rear_soft_limit_task_test.py` (all three variants),
  `rear_imu_filter_test.py`, `up_stair_integration_contract_test.ps1`,
  `rear_imu_loss_test.cpp`, `rear_retract_request_test.cpp`, targeted diff check.
- `MDK-ARM/rear_roll_fsm_build.log`: 0 errors, 5 existing RemoteData_t warnings.
- Additional legacy `rear_soft_limit_test.cpp` fails at line 76: its retract
  velocity assertion assumes zero feedforward, but production calibration is
  already {2, -3} Nm. A temporary test with Roll disabled still fails identically;
  the same temporary test with retract feedforward zero passes. No production
  calibration or unrelated test assertion was changed to hide this mismatch.
- No firmware flashing or vehicle response validation performed.
