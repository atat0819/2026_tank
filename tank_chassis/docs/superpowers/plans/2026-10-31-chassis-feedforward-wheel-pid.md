# Chassis feedforward and wheel PID implementation plan

> Execute in the user-approved current checkout with test-driven-development; request an independent review before completion. Do not commit or flash.

**Goal:** Replace body-speed PI with the approved second-order setpoint planning, 13 kg translational inertia feedforward and four wheel-speed PID loops. Keep X/Y/yaw limits, power strategy, CAN routing and input/feedback interlocks.

**Architecture:** Use the documented second-order follower in the velocity domain: its position-like state is chassis velocity, its velocity-like state is acceleration, and its bounded second derivative is jerk. Solve the accelerate/hold/decelerate segments analytically, including reversal and exact endpoint handling; do not copy the source's zero-denominator branches. Plan X/Y only, with initial limits 4 m/s^2 and 40 m/s^3 at 1 ms; retain direct yaw target response. These are explicit commissioning values, not identified physical limits.

The wheel target and feedback use wheel-side rad/s. Each PID produces A; initial Kp=1 A/(rad/s), Ki=0.5 A/rad, Kd=0, integral output limit=2 A. They are new commissioning gains, not the old body PI coefficients. Feedforward wrench is [13*ax,13*ay,0], allocated using the existing mecanum geometry and nominal wheel torque constant. Friction feedforward is zero. Sum feedforward and PID before uniform 20 A limiting and existing power limiting; both saturation stages inhibit further integral windup.

**Source:** HERO, Harbin Institute of Technology (Weihai), author 影丨, section 2.3.2.2: https://bbs.robomaster.com/wiki/36827278/1647282?source=4 . The page declares CC BY-NC-SA 4.0 and supplies snippets rather than a complete firmware repository. Preserve attribution and describe the velocity-domain adaptation and corrections.

**Files:** Add `user/core/Alg/UtilityFunction/SecondOrderSetpointPlanner.hpp`; update `MecanumForceControl.hpp`, the focused drive/reset/VOFA portions of `RtosTask/can_send_task.cpp`, the Keil header listing, relevant host tests and `docs/chassis_force_vofa.md`. Do not edit the public PID, power, generic wheel-math, input or mechanism implementations.

- [x] Add planner tests for bounded acceleration/jerk, exact arrival, maximum acceleration, braking, mid-ramp reversal and reset. Observe failure before adding the planner.
- [x] Replace the force-controller tests with independent geometry/virtual-work checks, 13 kg feedforward, zero rotational/friction feedforward, per-wheel feedback, additive current, two-stage anti-windup and a power-limited 13 kg plant. Observe failure before changing the controller.
- [x] Implement the planner and controller; run `python tests/second_order_setpoint_test.py` and `python tests/chassis_force_control_test.py`.
- [x] Adapt task tests, then connect planning, wheel feedback and combined currents. Reset both planners and all four PID states at every existing safety/mode reset; leave the existing fault gate and zero-CAN branches intact.
- [x] Keep 15-channel USB framing and 5 ms cadence: 1-6 planned/filtered body speeds; 7-9 inertia wrench; 10-11 planned X/Y acceleration; 12 maximum absolute wheel PID request (A); 13 total current scale; 14 enable/fault state; 15 time.
- [x] Run wheel-direction, speed-limit, stair/fault, force-VOFA, USB transport and input/mechanism command regressions; verify actual task outputs contain both feedforward and PID and still satisfy the power model.
- [x] Build with `D:/Keil5/Core/UV4/UV4.exe -b tank_chassis.uvprojx -j0 -o chassis_feedforward_build.log` from `MDK-ARM`; inspect the fresh link/HEX result. Run diff checks and independent code review; fix important findings and document any hardware limitations.

Baseline: clean working tree at e506456; existing force, wheel direction, speed limit, stair/power and force-VOFA host tests all passed before edits.


Verification record:

- Planner tests failed first because the new header was absent; controller tests then failed on missing MASS_KG/four-wheel API before the controller replacement. Both passed after their implementations. Existing task tests were adapted from old three-axis assumptions to assert the new production snippets.
- Fresh successful commands: `python tests/second_order_setpoint_test.py`, `python tests/chassis_force_control_test.py`, `python tests/chassis_wheel_direction_test.py`, `python tests/chassis_x_speed_limit_test.py`, `python tests/stair_wheel_power_test.py`, `python tests/chassis_force_vofa_test.py`, `python tests/vofa_send_test.py`, `python tests/stair_command_publication_test.py`.
- Task tests verify first-cycle acceleration/13 kg feedforward, mid-motion braking, yaw wheel-feedback braking with zero yaw feedforward, input/mouse/spin limits, V/B/C four-wheel drive, summed-current power limiting, and all feedback/source/STOP/mode reset paths including planner states. USB tests verify the changed channel meanings and unchanged framing, cadence, busy handling and fault recovery.
- A source comparison against e506456 confirms all three speed limits, the power-enable switch, fitted coefficients and the comment-stripped entire power-control implementation are unchanged. Public PID, input FSM and mechanism implementations were not modified.
- Fresh Keil log `MDK-ARM/chassis_feedforward_build.log`: successful link and HEX generation, 0 errors, 5 existing RemoteData_t C-linkage warnings; process exit 1 reflects warnings. Edited source/docs/test/project diff whitespace check passed; generated Keil logs/dependency files retain their generated whitespace.
- Independent read-only control review found no actionable defect and passed an additional 20 million random retarget steps with endpoint/acceleration/jerk checks. The small float quantization error in cycle-average acceleration is within test tolerance.
- Independent final integration review also passed: per-wheel feedback, combined power limiting, all three reset sites, telemetry semantics and preserved input/speed-limit paths checked against HEAD; five relevant tests were rerun successfully.
- No commit, flash, measured inertia, friction fit or hardware validation was performed. Real task timing, wheel direction under load, drivetrain efficiency, gains, slip and braking distance remain commissioning checks.

User correction: wheel total reduction is exactly `268/17`, replacing the earlier 15.8 approximation. The shared constexpr now uses floating-point division; wheel feedback and nominal torque-per-amp conversion both derive from it. Exact-ratio and nominal torque-coefficient regression assertions failed with the prior value before this correction.

Exact-ratio verification: force-controller, actual wheel-direction and actual stair/power/reset tests passed with `268/17`. Fresh `MDK-ARM/chassis_exact_reduction_build.log` confirms link/HEX generation, 0 errors and 5 existing warnings. No flash or unrelated parameter changes.
