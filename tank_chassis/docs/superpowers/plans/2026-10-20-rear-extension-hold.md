# Rear IMU extension hold

**Goal:** Reserve 4 encoder degrees at the extended ends in IMU mode, hold
position while further extension is requested, and resume attitude control on
a sustained shortening request. Use one raw-HI14 Pitch target of 1.0 degree.

**Minimal implementation:** Keep the current FSM and MIT sending path. Add two
per-leg hold flags and one shared Pitch release timer. Use a bounded position
P calculation in the existing velocity limiter, reusing retract MIT Kd. Suppress
attitude feedforward on a held leg; preserve recovery scaling, torque limiting,
mechanical validity checks and retract behavior. No new controller or mode.

Initial tunables in the FSM header: margin 4 degrees, position Kp 8 /s,
position speed cap initially 1 rad/s, release Pitch error initially 0.5 degree for 100 ms. These
need loaded vehicle validation. Keep all mechanical endpoints unchanged.

- [x] Add a host regression for mirrored boundaries, fixed hold targets,
  small/large Pitch errors, drift, release hysteresis/time rollover and faults.
- [x] Run the regression before implementation and confirm boundary failure.
- [x] Implement hold and migrate Pitch target/feedback to raw HI14 coordinates.
- [x] Adapt related tests to the approved behavior; execute actual task output
  with real FSM/PIDs and a motor sink, including Roll and Pitch damping.
- [x] Run focused regressions and Keil build; inspect the complete diff.

No commit, flashing, or vehicle operation is part of this change.

Results: extension regression, task harness (three Kd/feedforward variants), IMU
filter, IMU-loss, retract-request, Roll FSM and integration contract pass. Keil
reports 0 errors and 5 existing RemoteData_t warnings. The legacy aggregate FSM
test fails on its hardcoded 94.7-degree endpoint assertion; the unmodified HEAD
test and sources reproduce the same failure. Leave unrelated calibration
assertions unchanged. Build log: `MDK-ARM/rear_extension_hold_build.log`.

Follow-up: select retract MIT Kd independently on each held leg, using the same
selected value for torque protection and the sent MIT command. Unheld legs keep
IMU MIT Kd. Preserve user-tuned header values; task release tests use the active
release duration. The motor-sink regression first reproduced the old Kd choice.
