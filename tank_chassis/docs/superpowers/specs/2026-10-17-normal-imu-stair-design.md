# NORMAL / IMU mechanism commands

Keep the front J4340 FSM and rear J6248 FSM independent. A small command
coordinator translates debounced V/C/B edges into explicit persistent requests.
It does not inspect motor positions or wait for either mechanism to arrive.

| Input | Mode | Front request | Rear request |
|---|---|---|---|
| Startup / safety reset | NORMAL | Home | Zero torque |
| V | IMU | Home | Pitch leveling |
| First B in IMU | IMU | Target | Pitch leveling |
| Next B in IMU | IMU | Home | Retract and hold |
| Next B, including during motion | IMU | Target | Pitch leveling |
| C from any phase | NORMAL | Home | Zero torque |

V always restarts the IMU home/leveling phase. C takes precedence over V and B
if edges arrive together; V takes precedence over B. B in NORMAL is ignored.
Keep the existing double-middle and keyboard-online gates for keyboard edges.
Double-down, invalid switches, input loss and input-source changes override all
requests, clear the phase and require a fresh V after recovery. Leaving keyboard
control cancels the front stair request as before; selected IMU rear operation
continues with its last request until a mode/safety command changes it.

Publish the mode and both requests together under a FreeRTOS critical section;
the mechanism task reads one snapshot for both FSMs. Safety stops also latch
NORMAL and increment a shared generation; producer samples from an older
generation cannot overwrite the safety latch. Front feedback recovery
continues to return home first. Rear feedback, limits, soft start, Pitch filters,
and final torque clamps remain in the independent FSM and task. Invalid, stale
or nonfinite IMU disables both rear motors in every phase, including retract and
retracted hold; no basic-angle fallback remains. A persistent rear retract
request can interrupt recovery; clearing it immediately selects leveling. Retraction
retains an unfinished recovery ramp and restarts the ramp after any feedback
recovery, including feedback return simultaneous with B2.

Left-front J4340 calibration: HOME=139 degrees, TARGET=-160 degrees,
following the positive seam-crossing path 139 -> 180/-180 -> -160. The control
target unfolds to 200 degrees (travel +61 degrees). User-requested 15-degree
outward margins give the wrapped valid range 124 -> 180/-180 -> -145 degrees
(continuous 124..215). Right-front J4340 calibration: HOME=-109, TARGET=172 degrees, following the
negative seam-crossing path -109 -> -180/180 -> 172 (travel -79 degrees).
The existing unwrap represents this as 251 -> 172 degrees. The 15-degree
outward boundaries are -94 degrees at the home side and 157 at the target
side; the stored wrapped range is LIMIT_START=157, LIMIT_END=-94 (157..266
continuous degrees).

Rear J6248 mechanical calibration: left 29..91 degrees, right 6..68 degrees,
without seam crossing. Fully extended is left 29/right 68. B2 retracts to the
user-confirmed flat-wheel posture, left 91/right 6, so left increases and right
decreases. ANGLE_LIMIT_TOLERANCE_RAD is 0.5 degrees, as confirmed by the user:
the inner 0.5-degree band blocks outward commanded motor effort, and the outer
0.5-degree band remains valid so inward effort is allowed. Beyond that tolerance,
the existing task gate zeros both rear motors. Targets remain inside nominal
mechanical bounds; tolerance does not enlarge target validation. No automatic
pullback or active holding is added to NORMAL. Preserve existing Pitch direction
configuration; the calibrated movement paths do not establish its sign.
Do not flash or commit.

Acceptance: exercise V/B/B/B/C, commands during movement, startup/long-held
keys, mode-key priority, NORMAL torque suppression, invalid feedback, safety
reset, existing filtering/limits, host tests and the Keil build.

IMU loss does not change the selected NORMAL/IMU mode, B phase or front request.
Valid IMU automatically restores the current rear request through the existing
4000 ms soft start; retract planning restarts from current encoder feedback.

The IMU freshness timeout is now the single `IMU_CONTROL_TIMEOUT_MS = 100U`
constant in `RtosTask/remote_task.hpp`, shared by the snapshot getter and rear
control task. A 99 ms-old valid frame is accepted; age >=100 ms disables rear
control. Explicit invalid/NaN/Inf feedback still disables control immediately.

The final application torque cap is now +/-30 N*m per rear J6248, after FSM
scale/gain/limit handling and before MIT encoding. The Pitch torque PID shares
that limit; retract PID limits remain +/-8 N*m. Preserve the driver's +/-97 N*m
protocol range so encoding and feedback conversion retain their correct scale.
