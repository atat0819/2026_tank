# Up-stair wrapped mechanical-limit control

## Goal

Support an independent, bounded encoder-angle interval for each front J4310,
including intervals that cross the encoder's 0/2π boundary. Position PID must
choose the path within the configured mechanical interval rather than the
shortest unrestricted circular path.

## Confirmed configuration

| Motor | Mechanical interval | Home | B-key target |
|---|---:|---:|---:|
| Left (ID 1) | 10° → 178° | 34° | 122° |
| Right (ID 2) | 300° → 70° across zero | 340° | 60° |

All configuration values are stored as radians in two-element arrays. Index 0
is the left motor and index 1 is the right motor.

## Interval rules

- `LIMIT_START_RAD[i] <= LIMIT_END_RAD[i]` denotes an ordinary interval,
  such as 10° → 178°.
- `LIMIT_START_RAD[i] > LIMIT_END_RAD[i]` denotes a wrapped interval that
  passes through 360°/0°, such as 300° → 70°.
- Ordinary interval validity is `start <= angle <= end`.
- Wrapped interval validity is `angle >= start || angle <= end`.
- Home and target must each pass their motor's validity check; invalid
  configuration or invalid feedback disables both motors.

## Continuous control coordinate

The raw J4310 feedback remains in `[0, 2π]` radians for MIT commands. The FSM
also maps it to a continuous control coordinate for position PID:

- For ordinary intervals, control angle equals raw angle.
- For wrapped intervals, raw angles below the interval start have `2π` added.

For the confirmed right motor configuration:

```text
raw 340° -> control 340°
raw 359° -> control 359°
raw   0° -> control 360°
raw  60° -> control 420°
```

Home 340° therefore maps to 340° and target 60° maps to 420°. The position
error for the B-key command is `420° - 340° = +80°`, so the motor crosses zero
inside its permitted interval. The reverse command produces `-80°`.

## FSM and task behavior

The FSM remains `DISABLED`, `HOME`, and `TARGET`. `Get_Target_Angle(id)` and a
new PID-feedback accessor return continuous control radians. A separate raw
angle accessor remains available for the MIT position field. If either motor
is offline, keyboard control is disabled, configuration is invalid, or raw
feedback is outside that motor's interval, the shared FSM becomes disabled;
the RTOS task resets both PID pairs and sends zero torque.

## Verification

1. Left motor accepts 10°–178° and rejects 9°/179°.
2. Right motor accepts 300°–360° and 0°–70°, rejecting 71°–299°.
3. Right home 340° maps to 340°; right target 60° maps to 420°.
4. Right B-key position error from home to target is +80°, not -280°.
5. Right reverse error from target to home is -80°.
6. Out-of-range feedback on either motor disables the shared FSM.
