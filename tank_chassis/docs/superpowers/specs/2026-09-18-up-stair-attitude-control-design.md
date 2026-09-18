# Up-Stair Attitude Control Design

## Scope

This design completes the up-stair mechanism control. Down-stair behavior and
the `V` key are explicitly out of scope.

The mechanism has two independently controlled actuator pairs:

- Two front J4310 motors move the front wheel arms between calibrated home and
  stair-hooking target positions.
- Two rear J6248 motors change the rear wheel-arm geometry. Their normal job is
  to keep the chassis level in pitch and roll, including during ordinary
  driving and while the front mechanism raises the vehicle nose.

## Operator and Mode Behavior

The DT7 switch values are `UP = 1`, `MIDDLE = 3`, and `DOWN = 2`.

| Switch condition | Front J4310 behavior | Rear J6248 behavior |
| --- | --- | --- |
| Double-down (`s1 == 2 && s2 == 2`) | Zero torque immediately | Zero torque immediately |
| Double-middle (`s1 == 3 && s2 == 3`) | Hold home or respond to the `B` key | Hold calibrated pitch and roll level |
| Any other valid active switch combination | Return to and actively hold home | Hold calibrated pitch and roll level |
| Required sensor, motor, or control-link failure | Safe zero torque for the affected mechanism | Safe zero torque for the affected mechanism |

Double-down has priority over every state transition and PID calculation. The
existing chassis drive controller also treats double-down as `CHASSIS_STOP`.

The rear attitude controller must not use `keyboard_mode` as its enable signal.
It is enabled in every valid active switch combination except double-down. The
front stair command remains available only in double-middle while keyboard
input is online.

## Front J4310 State Machine

The front mechanism uses these logical states:

```text
HOME_HOLD
    | first B press
    v
MOVING_TO_TARGET
    |
    v
TARGET_HOLD
    | second B press
    v
RETURNING_HOME
    |
    v
HOME_HOLD
```

`HOME_HOLD` and `TARGET_HOLD` are active closed-loop states. Reaching home does
not produce zero torque. The position and velocity controllers continue to
reject disturbances and keep each J4310 at its calibrated home angle.

Leaving double-middle for another active switch combination forces
`RETURNING_HOME`, regardless of whether the mechanism is moving toward or
holding the target. After reaching home, it remains in `HOME_HOLD` and
continues active angle control. A new `B` command is ignored outside
double-middle.

Double-down bypasses controlled return and immediately commands zero torque.
When leaving double-down for an active mode, the front controllers are reset,
their output is ramped in, and the commanded state is `HOME_HOLD`.

Each front motor retains its independently calibrated raw-angle range, home
angle, target angle, direction convention, and wrapped `0/2*pi` interval
handling. Invalid or missing feedback must never be used in a PID calculation.

## Rear J6248 Attitude Controller

There is no separate rear lifting state and no fixed rear lift angle. Raising
or lowering the rear of the chassis is the physical result of holding the
chassis attitude while the terrain or front mechanism changes the nose height.

The controller uses two cascaded attitude loops:

1. The pitch and roll angle loops convert attitude error into desired chassis
   angular rates.
2. The pitch-rate and roll-rate loops convert angular-rate error into pitch and
   roll torque components.

After subtracting the calibrated level offsets, the internal pitch and roll
targets are zero:

```cpp
pitch_rate_target = pitch_angle_pid.Update(0.0f, pitch_deg);
pitch_torque = pitch_rate_pid.Update(pitch_rate_target, pitch_rate_dps);

roll_rate_target = roll_angle_pid.Update(0.0f, roll_deg);
roll_torque = roll_rate_pid.Update(roll_rate_target, roll_rate_dps);
```

Pitch is the common-mode component and roll is the differential component.
Motor installation direction is applied after mixing:

```cpp
left_torque = left_direction * (pitch_torque + roll_torque);
right_torque = right_direction * (pitch_torque - roll_torque);
```

The exact signs of the pitch, roll, left-motor, and right-motor mappings are
calibration parameters verified at low torque before loaded testing.

The initial implementation intentionally has no fixed lift angle, no
integration of attitude output into a motor angle target, no static support
torque feedforward, and no requirement that the left and right linkage angles
match. On uneven ground, unequal linkage angles are valid when they produce a
level chassis.

The PID implementation may use an integral term later if testing demonstrates
persistent steady-state attitude error. Initial integral gains are zero. Any
enabled integral must have anti-windup and must stop accumulating while its
actuator is blocked by an angle or torque limit.

## Rear Mechanical Protection

Rear motor encoders do not participate in the nominal pitch/roll feedback
loop. They provide per-motor mechanical-interference protection and feedback
validity checks.

The implementation exposes calibration constants for each J6248:

```text
REAR_LIMIT_START_RAD[2]
REAR_LIMIT_END_RAD[2]
REAR_DIRECTION[2]
```

The valid raw-angle interval may cross the `0/2*pi` boundary. A motor at a
limit may still receive torque that moves it back toward the valid interval;
only torque that drives it farther outward is suppressed.

After pitch and roll mixing, the two commands are scaled together if either
would exceed the configured torque limit. Scaling preserves the requested
common/differential ratio better than clipping each command independently.

## Enable, Disable, and Recovery

At the start of every control iteration, the task evaluates the safety gate
before running any controller:

```text
double-down or control-link offline
    -> all four mechanism motors command zero torque
    -> reset all mechanism PID states

otherwise
    -> front J4310 holds or moves according to its state
    -> rear J6248 runs pitch/roll attitude control when IMU and feedback are valid
```

An invalid IMU disables the rear attitude output but does not create a rear
angle-control fallback. Invalid feedback from a mechanism motor prevents that
motor's normal closed-loop output. Recovery resets the relevant controllers
and ramps commanded torque from zero to avoid a step input.

The implementation must distinguish control-link availability from keyboard
frame availability. Rear attitude control is allowed in non-keyboard driving
modes; `B` events require double-middle and fresh keyboard data.

## Tuning and Filtering

Pitch and roll level offsets are recorded with the vehicle in its normal
mechanical posture on a level surface. The angle and angular-rate units remain
consistent with the BMI088 interface: degrees and degrees per second.

Initial tuning uses conservative proportional gains and zero integral gains.
The following protections are applied independently of PID gains:

- Small attitude deadbands to avoid reacting to measurement noise.
- Maximum desired pitch and roll rates.
- Maximum total J6248 torque.
- Torque slew-rate limiting when attitude control is enabled or recovers.
- Per-motor directional angle limits.

Longitudinal acceleration and braking can disturb an accelerometer-corrected
pitch estimate. Loaded testing must therefore verify that normal driving does
not cause excessive rear linkage motion. Filtering should be increased or
attitude gains reduced if this occurs.

## Verification

Host-side unit tests cover:

- The complete switch-mode truth table.
- Double-down overriding every front and rear state with zero torque.
- `B` commands being accepted only in double-middle with fresh keyboard data.
- Leaving double-middle forcing a controlled front return to `HOME_HOLD`.
- `HOME_HOLD` continuing active control rather than producing zero torque.
- Pitch common-mode and roll differential torque mixing.
- Proportional torque scaling when a mixed command exceeds the limit.
- Directional blocking at each wrapped and non-wrapped rear angle limit.
- PID reset and output-ramp behavior after disable or feedback recovery.

Bench verification is performed at a low torque limit before loaded stair
tests:

1. Confirm each rear motor's positive torque direction.
2. Raise the nose manually and verify that both rear motors correct pitch in
   the intended direction.
3. Lower one side manually and verify differential roll correction.
4. Exercise every rear mechanical limit and confirm reverse escape remains
   available.
5. Trigger double-down from every front state and confirm all four mechanism
   torque commands become zero in the next control cycle.
6. Switch from double-middle to another active mode while the front mechanism
   is extended and confirm controlled return followed by active home hold.

Only after these checks pass should the torque limits and gains be increased
for loaded up-stair testing.
