# Rear MIT speed inner loop implementation plan

**Goal:** Run rear Pitch, retraction and limit backoff with host angle PIDs and
MIT motor-speed feedback; both motor Kd values are 1.3 as requested.

**Architecture:** Preserve the mode FSM, calibrated endpoints, two-second recovery,
direction mapping, and outer PID gains. Convert the Pitch outer output from degrees/s
to rad/s; encoder position outer outputs already use rad/s. Send Kp=0, Kd=1.3 and
feedforward torque=0. Remove host speed PIDs. Keep the existing torque safety calculation
as a sampled constraint: predict Kd*(target-actual), limit it, then reconstruct a
motor speed target. This does not establish a hard limit on torque between frames;
the user confirms driver-side limiting is configured, without supplying its value.

**Tech Stack:** Existing C++ FSM/PID, extracted task host tests, g++, Keil MDK.

Execute inline in the current dirty checkout. Preserve unrelated edits. No commit,
reset, halt, flashing, or driver configuration changes.

- [x] Extend `tests/rear_soft_limit_task_test.py` to record complete MIT commands
  and evaluate their sampled speed-loop torque. Fail first on current Kd=0 commands.
  Verify the requested Kd, radian conversion, motor-speed feedback, opposite directions,
  retraction, each side's backoff/hold/handoff and zero gains on faults.
- [x] Add visible `MIT_SPEED_KD[2]={1.3f,1.3f}` and the protocol speed bound in
  `fsm/up_stair_behind_motor_fsm.hpp`. Add `Limit_Velocity()` in the rear FSM using
  `velocity + ClampJ6248Torque(Limit_Torque(id,Kd*(target-velocity)))/Kd`, then clamp
  to the protocol +/-20 rad/s range. Test startup, gain, bounds, nonfinite input,
  slow-zone braking and the actual calibrated stop positions.
- [x] In `RtosTask/up_stair.cpp`, keep only each outer angle PID, compute per-side
  speed targets, and blend backoff/attitude speed targets before the shared limiter.
  Backoff bypasses recovery scaling exactly once as before. Change limit-recovery
  requests and their threshold to rad/s: the old 0.2 Nm threshold divided by the
  former speed Kp0.8 gives 0.25 degrees/s, or 0.0043633 rad/s. This keeps the former
  static Pitch-error sensitivity rather than retaining a mismatched torque threshold.
  Use MIT speed fields in every enabled rear branch and zero gains in every disabled
  branch. Update the affected integration contract assertions.
- [x] Run `python tests/rear_soft_limit_task_test.py`, the rear filter and integration
  checks, and the rear FSM/soft-limit/IMU-loss/retract-request/command-flow host tests.
  Build Keil with `-b tank_chassis.uvprojx -j0 -o rear_mit_speed_build.log` from
  `MDK-ARM`. Review only task edits and record build output and hardware limits.

**Validation boundary:** Kd now closes motor-shaft speed, not IMU Pitch rate; the old
closed-loop dynamics are not preserved. Pitch initially remains capped at 30 degrees/s
(0.524 rad/s). At zero shaft speed, a 0.5 rad/s backoff target and Kd=1.3 request only
0.65 Nm. Use a 0.1 Nm illustrative load for the no-integral backoff equilibrium test;
do not claim the former 1 Nm load-rejection margin or measured vehicle performance.

**Completed verification:** Five C++ host programs passed with `-Wall -Wextra`
(rear FSM, soft limits, IMU loss, retract requests, keyboard command flow). Extracted
task tests passed MIT fields, shaft feedback, IMU-rate independence, both single-side
feedback failures, calibrated flat-stop entry, retraction, unloaded hold/handoff,
0.2-degree inward-error handoff, and the illustrative loaded return equilibrium.
The latter remains in position recovery; it does not prove loaded HOLD entry.
Filter/integration checks and `git diff --check` passed. Final Keil log
`MDK-ARM/rear_mit_speed_final_build.log`: 0 errors, 5 warnings. Independent review,
including the velocity-threshold follow-up, found no mandatory fixes.

Zero recovery scale matches the sampled shaft velocity while retaining Kd=1.3:
sampled torque is zero, but inter-frame speed changes still produce MIT damping.
No hardware execution, firmware download, commits or driver-limit verification occurred.
