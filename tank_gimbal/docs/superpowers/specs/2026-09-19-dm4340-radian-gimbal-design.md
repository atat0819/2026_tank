# DM4340 Radian Gimbal Design

## Goal

Replace the two LK4005 gimbal motors with two Damiao J4340 motors on FDCAN3,
retain the MCU's IMU-based cascaded control, and use radians internally.

## Hardware Mapping

| Axis | Damiao motor CAN ID | FDCAN | Control feedback |
| --- | --- | --- | --- |
| Yaw | 1 | FDCAN3 | IMU |
| Pitch | 2 | FDCAN3 | IMU |

FDCAN3 uses the existing FIFO0 configuration.  Its 16-frame depth and the
current callback's drain-until-empty behavior are sufficient for two motor
feedback streams.  FIFO1 is not part of this change.

## Damiao Driver

`DMMotorBase` and each concrete motor constructor accept a
`HAL::FDCAN::FdcanDeviceId`.  All Damiao transmit operations use that selected
bus instead of the current hard-coded FDCAN2 path.  The gimbal creates one
`J4340<2>` on FDCAN3 with receive/send IDs `{1, 2}`.

J4340 MIT position encoding is configured as `0 .. 2*pi`, matching the motor
configuration.  Position, velocity, gains, and torque are finite-value checked
and clamped to the configured MIT ranges before packet encoding.

The motors run in MIT mode but receive external torque control:

```cpp
ctrl_Mit(id, 0.0f, 0.0f, 0.0f, 0.0f, torque_nm);
```

The MCU therefore remains responsible for position and velocity control.

## Units and Coordinate Boundaries

The gimbal control layer uses `rad`, `rad/s`, and `N*m` exclusively.

- IMU UART data remains in degrees and degrees per second.  It is converted at
  the entry to the control layer.
- Vision USB data remains an absolute target in degrees in `[-180, 180]`.
  It is converted to radians before it reaches the gimbal FSM.
- Yaw feedback is unwrapped continuously in radians to avoid a discontinuity
  at `-pi/pi`.  A bounded vision target is mapped to the equivalent angle
  nearest that continuous yaw feedback.
- Pitch retains its mechanical limits, converted from degrees to radians.
- Remote and mouse gains are expressed as `rad/s` and `rad/pixel`.
- PID and feed-forward states are reset on mode/source transitions as today.
  Final torque PID and feed-forward values must be re-tuned in N*m for the
  J4340; LK4005's `+/-2048` command scale is never reused.

## Safety

Remote loss, either Damiao motor feedback loss, startup protection, and an IMU
fault command zero torque to both axes.  The old encoder-feedback control
fallback is removed.  When IMU data recovers, both FSM targets are re-anchored
to the current IMU attitude and controller states are reset before output is
restored.

## Chassis Yaw Offset

The chassis interface remains degree-based and independent of the gimbal
control layer.  `YawOffset_GetDeg()` reads the Yaw J4340 encoder (motor ID 1)
as degrees, applies the existing front-alignment calibration constant, then
normalizes to `[-180, 180]` before FDCAN2 transmission.  This preserves the
existing chassis protocol and its definition of zero as "barrel forward".

## Non-goals

- Change IMU UART format or visual USB protocol.
- Change the absolute-angle semantics of visual targeting.
- Add FDCAN FIFO1.
- Use motor-encoder feedback in the normal gimbal control loops.
