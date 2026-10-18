# Rear J6248 soft limit implementation plan

**Goal:** Use one configurable IMU Pitch reference; slow each rear motor near
its encoder limits, back off on an outward approach, hold inside the range and
resume attitude control only when its request permits leaving the boundary.

**Architecture:** Keep the shared Pitch cascade in `RtosTask/up_stair.cpp`.
The rear FSM owns per-motor backoff state/targets and velocity-based torque
constraints. Reuse each motor's position P with independent bounded backoff
speed P controllers during backoff, hold and a timed handoff. No entry preextension phase. Preserve the
existing NORMAL/stop/invalid-IMU zero-torque behavior, buses and front control.

**Tech Stack:** C++, existing FSM/PID, host g++, Python/PowerShell, Keil MDK.

Execute inline in this checkout, preserving pre-existing changes; no commits.

- [x] Run the existing rear FSM test as a baseline. Add a failing regression
  showing production retract targets must be inside the endpoint buffers.
- [x] Add the common `pitch_target_deg` configuration separately from the
  measured horizontal zero. Initially0; user calibration subsequently supplied
  raw level1.3 degrees and raw target-7 degrees, giving corrected target-8.3.
- [x] Add per-motor outward-approach detection, a latched inner target,
  low-speed/position dwell, safe hold and inward-request dwell before handoff.
  Exercise upper/lower/wrapped intervals, independent motors, startup at a
  stop with an inward request, and invalid feedback/IMU/velocity/control reset.
- [x] Add continuous position-to-velocity-to-torque constraints after normal
  recovery scaling/gain. Test active braking, both directions, normal startup
  scaling, and final torque bounds. Backoff has independent speed P controllers;
  protection must not vanish at scale zero.
- [x] Pass motor velocities to the FSM and update per-side backoff before
  selecting/blending position output. Run task-level tests as well as FSM tests.
- [x] Run affected behavior tests, filter/integration/clamp checks, source diff
  checks and Keil build. Review only this task's changes against its start
  snapshot. Record provisional tuning and hardware validation limits.

Verification commands:

```powershell
g++ -std=c++17 -I . tests/up_stair_behind_motor_fsm_test.cpp fsm/up_stair_behind_motor_fsm.cpp user/core/Alg/FSM/alg_fsm.cpp user/core/Alg/UtilityFunction/SlopPlanning.cpp -o tests/build/up_stair_behind_motor_fsm_test.exe
g++ -std=c++17 -I . tests/rear_soft_limit_test.cpp fsm/up_stair_behind_motor_fsm.cpp user/core/Alg/FSM/alg_fsm.cpp user/core/Alg/UtilityFunction/SlopPlanning.cpp -o tests/build/rear_soft_limit_test.exe
python tests/rear_imu_filter_test.py
powershell -NoProfile -ExecutionPolicy Bypass -File tests/up_stair_integration_contract_test.ps1
```

Keil: `D:/Keil5/Core/UV4/UV4.exe -b tank_chassis.uvprojx -j0 -o
rear_soft_limit_build.log`, working directory `MDK-ARM`; inspect its fresh log.

Calibration boundary: the user corrected the earlier stop readings. Current
mechanical bounds are left 29..92.84 degrees and right 4.77..68 degrees; the
four-wheel-flat hard-stop readings are left 92.84/right 4.77. Retraction targets
are left 84.84/right 12.77 degrees. New buffers/speeds are visible trial
settings, not measured stopping guarantees.

## Final behavior and trial parameters

- `PITCH_ZERO_DEG=1.3`, `PITCH_TARGET_DEG=-8.3`: measured raw level1.3 degrees
  and desired raw working Pitch-7 degrees. The measured working encoder angles
  left71/right26 degrees are a reference, not position-control setpoints.
  There is no automatic
  capture of the entry attitude and no entry encoder-position preextension.
- Endpoint distances: slow at5 degrees, trigger at2 degrees on outward
  request/motion, latched backoff target8 degrees inside that endpoint.
- Retraction targets are left84.84/right12.77 degrees, 8 degrees inside the
  latest calibrated left92.84/right4.77 degree hard-stop endpoints.
- Approach and backoff speed0.5rad/s. Backoff position P is reused; dedicated
  speed P has Kp8,Ki0,Kd0 and raw output cap8Nm. PID values are written directly
  in `RtosTask/up_stair.cpp`; unused header PID constants were removed.
- Settle within0.5 degrees and0.05rad/s for100ms. Hold until a request strictly
  toward the interior persists100ms; handoff takes300ms and cancels if that
  inward request ceases. Normal startup scaling applies once to the attitude
  component, while braking/backoff remain effective. Final cap stays30Nm.

## Verification evidence

The evidence below records the initial PI version. The subsequent user-requested
pure P revision is recorded separately at the end.

- Existing rear FSM baseline passed before changes. New interior-retract test
  failed on the old endpoint target, then passed after the feature was added.
- Five C++ programs passed with `-Wall -Wextra`: rear soft limits, existing
  rear FSM, IMU loss, retract requests, and front/rear keyboard command flow.
- The real extracted rear task branch with real PIDs passed sign, independent
  side takeover, hold/handoff, fault-zeroing and1Nm constant-load joint checks.
  This is a synthetic joint test, not a vehicle load model or hardware proof.
- Existing task IMU filtering test and four PowerShell contracts passed:
  stair integration, MIT clamp, CAN routing, and enable dispatch.
- Review found handoff could finish after the inward request disappeared.
  A neutral-request regression failed before the fix, then passed. Dedicated
  speed PIs also removed the old0.52Nm static backoff-effort ceiling; the1Nm
  synthetic-load regression failed before that change and passed afterward.
- Independent read-only re-review found no remaining important findings.
- Final `MDK-ARM/rear_soft_limit_build.log`: AXF/HEX generated,0 errors and5
  pre-existing warnings (`RemoteData_t` typedef linkage warnings). Keil exit1
  denotes warnings in this build; the fresh log confirms successful linking.
- Source/test/doc diff whitespace checks passed. No flash, reset or commits.

## Pure P revision

- User requested no integral in the backoff speed controller. Both controllers
  now use `{8,0,0,8,0,0}`; the original retract controllers are unchanged.
- A repeated-error regression failed on PI and passed on pure P: output does
  not accumulate, and returns to zero when speed error becomes zero.
- The synthetic1Nm constant-load check now verifies the expected P-only static
  position error `1/(5*8)=0.025rad` (about1.43 degrees), while remaining outside
  the mechanical-endpoint slow zone. This exceeds the0.5-degree settle tolerance;
  the controller continues position recovery until the settle condition holds.
- This synthetic-load result is not hardware validation or a stopping guarantee.
- Revised task/PID regression, rear soft-limit FSM and stair integration contract
  passed. `MDK-ARM/rear_soft_limit_p_only_build.log` records AXF/HEX generation,
  0 errors and5 existing warnings. No flash or commits.
