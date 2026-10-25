# Front J4340 speed soft start

**Goal:** Reuse the rear retract 0-to-1 speed multiplier for both front motors,
with a 1000 ms ramp and unchanged final HOME/TARGET angles.

**Architecture:** Keep one `SlopePlanning` instance in the existing front FSM.
Pass the task's sampled millisecond tick into `Update`; restart the multiplier
when enabling or changing the effective target. Reaching HOLD must not restart
it. Both existing angle PID outputs use the same multiplier before MIT sending.
Feedback loss, invalid angles and safety stopping retain immediate zero gains
and zero torque; recovery still homes first. Preserve existing tuning and all
unrelated workspace changes. No commit or flashing.

**Tech Stack:** Embedded C++, existing FSM/PID/SlopePlanning, g++, Python, Keil.

- [x] Replace stale fixed PID/Kd expectations in the existing front task harness
  with active source parameters. Add behavioral checks for the ramp at start,
  quarter/half/full duration, repeated ticks, early HOLD, feedback jumps,
  reversals, forced HOME, recovery and tick rollover. Run
  `python tests/front_mit_speed_task_test.py` and observe missing soft start.
- [x] Add `SPEED_RAMP_TIME_MS = 1000U`, shared planner/tick state and
  `Get_Speed_Scale()` to the front FSM. Append the required `now_tick` argument
  to `Update` and its callers. Reset on disable/re-enable and effective target
  changes. Advance using `min(1, delta_tick / SPEED_RAMP_TIME_MS)` and call
  `TIM_Calculate_PeriodElapsedCallback(1.0f, 0.0f)` to avoid feedback bypass.
- [x] Multiply each existing task PID result by `Get_Speed_Scale()`; retain MIT
  Kp=0, existing Kd and zero feedforward. Adapt the two existing C++ test callers
  to pass their tick; link `SlopPlanning.cpp` in host tests.
- [x] Run the front task/FSM and stair command flow tests, relevant rear task
  regressions and integration/routing contracts. Build the Keil target and read
  its log; inspect this task's diff against the pre-edit snapshot.

Host C++ checks use `g++ -std=c++17 -I .`, linking the corresponding FSM,
`user/core/Alg/FSM/alg_fsm.cpp` and
`user/core/Alg/UtilityFunction/SlopPlanning.cpp`; command flow also links the
rear FSM and `fsm/chassis_keyboard_fsm.cpp`.

Keil command, from `MDK-ARM`:
`D:/Keil5/Core/UV4/UV4.exe -b tank_chassis.uvprojx -j0 -o front_speed_ramp_build.log`.

Results: the task regression first reproduced the old immediate nonzero target
speed on a new motion, then passed with the shared ramp. Front FSM and V/B/C
command-flow host checks, rear task/filter regressions, integration contract and
DM clamp contract passed. The old routing script still fails its historical
front P=15/8 expectations; the pre-edit snapshot already had P=120 on both sides.
PID/Kd/limits/final angles are identical to that snapshot. All routing clauses
were checked separately and passed. Keil reports 0 errors and 5 existing
`RemoteData_t` typedef warnings; process exit 1 denotes the warning result.
No commit, flashing or loaded hardware verification.

Independent read-only review found no Critical/Important issues and reran the
front task regression successfully.
