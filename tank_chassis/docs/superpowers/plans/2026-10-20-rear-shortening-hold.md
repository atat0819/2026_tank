# Rear IMU shortened-end hold

Extend the approved extended-end hold to the shortened ends in the current
checkout. Preserve user calibration and reuse the active margin (2 encoder
degrees), position Kp (40 /s), speed cap (10 rad/s), release threshold (0.5 deg),
release duration (30 ms), and retract MIT Kd. Position Kp and speed cap now have
independent `SHORTENING_HOLD_KP` and `SHORTENING_HOLD_MAX_SPEED_RAD_S` parameters,
initialized to 40 /s and 10 rad/s, preserving the previous output by default.
Shortened targets are left upper
endpoint minus margin and right lower endpoint plus margin: 92.72 and 6.28 deg.

Keep the existing FSM and sending path. Represent each leg's hold as extended,
shortened, or inactive, and debounce its release independently. Extension holds
release on sustained shortening demand; shortening holds release on sustained
extension demand. Use a common hold query for Kd selection and suppression of
attitude feedforward. Guard both boundaries after release. Preserve the B-key
retract targets, feedforward, ramp, safety disabling and feedback recovery.

- [x] Add a regression using active header constants and reproduce missing
  shortened-end holding before implementation.
- [x] Extend existing FSM flags, boundary selection and velocity protection;
  update the task to recognize either held end.
- [x] Adapt actual-task tests for both end behaviors and active calibration;
  verify direction, release, mixed sides, Kd, combined feedforward and faults.
- [x] Run focused FSM/task/filter/integration regressions and Keil build.

No commit, flashing, or hardware control.

Results: the new shortened-end test first failed because the old velocity path
continued shortening at the reserved boundary. Both shortened- and extended-end
regressions now pass, including opposite ends on different legs and independent
release timing. The actual-task motor sink passes all three Kd/feedforward
variants, including shortened-end feedforward suppression and boundary guarding
after release. IMU filter, IMU loss, retract request, Roll FSM and task integration
checks pass. Keil reports 0 errors and 5 existing warnings; log is
`MDK-ARM/rear_shortening_hold_build.log`. No loaded vehicle verification.

Independent-tuning regression: a temporary build sets shortened-end Kp to 11 /s
and speed cap to 0.04 rad/s while keeping the production extended-end values.
The test reproduced shared-parameter output before the endpoint selection fix;
small-error and capped-output cases verify both shortened parameters separately.
