# All stair motors MIT speed loop plan

**Goal:** Inline MIT parameters in each command and replace the two front J4340
host speed PIDs with MIT speed feedback. User confirmed Kd=1.3 for both fronts,
matching the two rears. Keep existing angle gains and all mechanical/mode logic.

**Architecture:** Front single outer PIDs retain left Kp15, right Kp8, radian
feedback and +/-10 rad/s output caps. Send Kp0, Kd1.3, feedforward torque0 directly
in `ctrl_Mit`. Disabled commands retain all zero gains. Remove the rear Kd table;
pass the actual inline Kd explicitly to `Limit_Velocity` so its sampled torque
constraint uses the same value as the MIT frame. Preserve rear recovery and limits.

Execute in the current checkout, preserving unrelated dirty files; no commits or
flashing. Front HOME/TARGET, seam handling and pair-offline interlocks remain intact.

- [x] Add an extracted real-front-task test. Run against current code to demonstrate
  failure of the requested Kd. Cover HOME, TARGET, both wrap seams, bounded speeds,
  native shaft damping, reversals, pair dropout and HOME recovery, invalid angles
  and disabled control. Verify direct Kp0/Kd1.3/torque0 fields.
- [x] Convert front PID definitions/declarations/resets to angle-only objects;
  send their outputs as speed targets. Inline all four motors' Kd fields. Remove
  the rear Kd array, add explicit Kd to its limiter API, and update affected tests
  and integration assertions. Invalid Kd must not be used as a divisor.
- [x] Run front task/FSM, rear task/FSM/soft-limit/IMU-loss/retract, command flow,
  IMU filter and integration checks. Compile Keil and review against the turn-start
  snapshot, retaining current calibration and other user edits.

**Evidence boundary:** Native speed control changes motor dynamics. Front internal
torque is no longer the old host P gain times speed error; Kd is uniformly1.3.
Rear +/-30 Nm remains only a sampled request bound, not verified drive hardware
limiting. No hardware behavior is claimed from host tests and compilation.

**Verification:** The new extracted front-task test failed on the original Kd0
commands, then passed after the speed-inner-loop change. Front/rear task tests,
front/rear FSM tests, rear soft limits, IMU loss, retract requests, keyboard command
flow, IMU filter, integration and FDCAN routing checks passed. The routing contract
was updated to check the single angle PIDs, retaining bus/ID and driver-range checks.
`git diff --check` passed. Keil `MDK-ARM/all_stair_mit_speed_build.log` reports
0 errors and 5 warnings. Independent review found no remaining mandatory fixes.
No firmware download, driver configuration changes or commits occurred.
