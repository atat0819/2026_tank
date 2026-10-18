# NORMAL / IMU Implementation Plan

**Goal:** Implement the approved V/C/B workflow with independent front/rear FSMs.

**Architecture:** Header-only command coordination beside the existing switch
policy; atomic publication in can_send_task and one snapshot in up_stair_task.
Explicit target requests replace independent B/V event consumption in the FSMs.

**Tech Stack:** C++, FreeRTOS, host g++, Python/PowerShell checks, Keil MDK.

## Steps (execute inline in the current checkout; no commits)

- [x] Add failing behavioral tests for debounced V/C and command transitions:
  NORMAL ignores B; V selects home/leveling; B selects target/leveling then
  home/retract; another B reverses; C wins conflicting inputs; safety resets.
- [x] Add `StairCommand` and its small edge-to-request update function in
  `fsm/stair_mode_policy.hpp`. Extend the existing keyboard command with V/C
  pulses. Publish one `StairCommand` in `RtosTask/can_send_task.cpp`, protected
  by `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()`.
- [x] Add a coupled host scenario exercising the real independent FSMs before
  changing their request interfaces. Replace front action sequence with an
  explicit target request while retaining home-first feedback recovery.
  Replace rear retract sequence with a persistent retract request, accepted
  during recovery/movement and canceled immediately when leveling is requested.
- [x] In `RtosTask/up_stair.cpp`, snapshot commands once, gate rear control by
  IMU mode, pass explicit requests to both FSMs, and reset rear filter history
  in NORMAL. Keep existing PID, direction, motor routing and torque clamps.
- [x] Update old event-specific tests/contracts for the new interfaces. Run
  host behavior tests, rear IMU filtering, switch/routing/heartbeat contracts,
  `git diff --check`, and Keil build. Inspect both build log and final diff.

Commands: compile focused C++ assertions with `g++ -std=c++17 -I .`, linking
`user/core/Alg/FSM/alg_fsm.cpp` and
`user/core/Alg/UtilityFunction/SlopPlanning.cpp` for rear tests. Execute
`python tests/rear_imu_filter_test.py` and
`powershell -NoProfile -File tests/up_stair_integration_contract_test.ps1`.
Build with `D:/Keil5/Core/UV4/UV4.exe -b tank_chassis.uvprojx -j0 -o normal_imu_build.log`
from `MDK-ARM`; inspect the fresh log for target generation and error counts.

## Verification

- 10 C++ behavioral tests passed, including independent FSM integration,
  command coordination, keyboard debounce, recovery and torque limits.
- 4 Python checks passed: interleaved command publication/safety generation,
  real task IMU preprocessing, RS485 board link and HI14 snapshot.
- 5 PowerShell integration/routing/heartbeat/enable/clamp checks passed.
- Keil fresh build: `tank_chassis.axf - 0 Error(s), 5 Warning(s)`;
  evidence: `MDK-ARM/normal_imu_build.log`.
- Source/test/doc `git diff --check` passed; generated Keil logs/dependency
  files contain tool-generated whitespace and are excluded from that check.
- Independent source review found no remaining important findings after
  adding safety-generation publication and retract soft-start regressions.
- The original HEAD rear test was independently reproduced failing because
  its configured-speed expectations were stale. Tests now use the existing
  2000 ms recovery and explicit requests; production angle/PID values unchanged.
- No flash, reset, physical vehicle validation or commits performed.

## Revision: IMU loss uses zero torque

- [x] Reproduce the previous fallback behavior with a failing all-phase IMU-loss test.
- [x] Gate all rear states on valid finite IMU, remove the obsolete basic-angle
  state/configuration/planners, and retain the existing zero-command output path.
- [x] Update task/filter contracts and regression tests for invalid/stale/NaN/Inf
  samples, independent front behavior, and soft recovery of the selected request.
- [x] Run the 20 host/integration checks and rebuild Keil after this revision.

Revision verification: 11 C++ tests, 4 Python checks and 5 PowerShell contracts
passed. The new all-phase IMU loss regression failed before the change and now
passes; it covers invalid snapshots, NaN/Inf and recovery from current feedback.
Fresh Keil log reports `0 Error(s), 5 Warning(s)`; no flash or commits performed.

## Revision: shared 100 ms IMU timeout

- [x] Add failing snapshot tests for 20/99 ms validity and exact 100 ms expiry.
- [x] Define IMU_CONTROL_TIMEOUT_MS once in remote_task.hpp; snapshot getter
  and up_stair task both consume it. Keep explicit invalid/nonfinite handling.
- [x] Verify snapshot/control boundaries, tick rollover and recovery with the
  two Python tests and two related PowerShell contracts; all passed.
- [x] Fresh Keil build: 0 errors, 5 warnings. No flash or commits.

## Revision: rear torque limited to 30 N*m

- [x] Change the saturation regression to +/-30 and observe its failure at the
  previous +/-40 cap; cover both signs, protocol endpoints and in-range values.
- [x] Set J6248_TORQUE_LIMIT_NM=30; reuse it for the Pitch torque PID output
  limit. Preserve retract +/-8 and driver protocol +/-97 scaling.
- [x] Saturation test, stair integration contract and generic MIT clamp contract
  passed; source diff check passed. Keil rear_30nm_build.log reports 0 errors,
  5 warnings. No flash or commits.

## 2026-10-18: left-front J4340 calibration

- [x] Update tests first and observe failure with the old home angle.
- [x] Set left HOME=139, TARGET=-160, LIMIT_START=124, LIMIT_END=-145 degrees.
  Preserve right-front configuration and existing wrapped-angle algorithm.
- [x] Verify +61-degree travel, continuity across +/-180 in both directions,
  target arrival, invalid feedback/range handling and V/B/C integration.
- [x] Rebuild Keil: front_left_calibration_build.log reports 0 errors and 5
  warnings; front FSM/command-flow tests and routing/integration checks passed.
  No flash or commits.

## 2026-10-18: right-front J4340 calibration

- [x] Update tests first; reproduce failure with the old right-front home.
- [x] Set right HOME=-109, TARGET=172, LIMIT_START=157, LIMIT_END=-94 degrees;
  keep left calibration unchanged. Existing positive-domain unwrapping gives
  HOME=251 and TARGET=172, producing the requested -79-degree travel.
- [x] Verify both seam-crossing directions, target/home arrival, outside-range
  pair disable and keyboard command flow. Integration contract passed.
- [x] Keil front_right_calibration_build.log: target generated, 0 errors and
  5 warnings. Routing and source diff checks passed. No flash/commits.

## 2026-10-18: rear J6248 mechanical calibration

- [x] Confirm B2 targets with user: left 91/right 6 degrees (flat wheels).
- [x] Add calibration regression, observe failure with old full-circle bounds,
  then set nominal left 29..91/right 6..68 degree bounds and B2 targets.
- [x] Confirm 0.5-degree endpoint tolerance with user; reproduce failure before
  implementing expanded feedback validity and inward-only effort at endpoints.
  Keep retract target validation inside nominal bounds and preserve soft start.
- [x] Cover opposite retract directions, inner/outer endpoint bands, fully
  extended endpoints, beyond-tolerance rejection and wrapped-interval unfolding.
- [x] Four affected C++ tests, rear IMU filter extraction test, stair integration
  and MIT clamp contracts passed; source diff check passed. Keil log
  rear_calibration_build.log reports 0 errors and 5 existing warnings.
  Read-only review found no important defects. No flash or commits.

## 2026-10-18: slower rear J6248 torque soft start

- [x] Reproduce failure of a 4-second ramp regression with the previous 2-second
  configuration; cover 0/25/50/75 percent, just before completion and completion.
- [x] Set RECOVERY_TIME_MS=4000U and update its comments. Preserve torque caps,
  retract planning speed, mechanical limits and disable behavior.
- [x] Update recovery timing in existing tests, keeping action timestamps after
  setup. Rear FSM, retract interruption/recovery, IMU-loss and command-flow tests
  passed with the 4-second ramp.
- [x] IMU preprocessing, stair integration and MIT clamp contracts passed;
  source diff check passed. Keil rear_soft_start_4s_build.log reports 0 errors
  and 5 existing warnings. No flash or commits.
