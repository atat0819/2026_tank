$ErrorActionPreference = 'Stop'
$source = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\up_stair.cpp')

if ($source -notmatch '#include "../fsm/up_stair_behind_motor_fsm.hpp"') { throw 'rear FSM must be owned by up_stair.cpp' }
if ($source -notmatch 'rear_6248_pitch_pid\[2\]') { throw 'missing rear pitch cascaded PID instances' }
if ($source -notmatch 'rear_6248_roll_pid\[2\]') { throw 'missing rear roll cascaded PID instances' }
if ($source -notmatch 'EvaluateStairModePolicy') { throw 'task must evaluate the shared stair mode policy' }
if ($source -notmatch 'gimbal_switch_last_tick') { throw 'task must use switch heartbeat' }
if ($source -notmatch 'Get_Target_Pitch_Deg\(' -or $source -notmatch 'Get_Feedback_Pitch_Deg\(') { throw 'task must consume rear FSM attitude getters' }
if ($source -notmatch 'Limit_Torque\(1') { throw 'task must route left torque through rear FSM safety limit' }
if ($source -notmatch 'Limit_Torque\(2') { throw 'task must route right torque through rear FSM safety limit' }
if ($source -notmatch 'front_4340\.ctrl_Mit\(1') { throw 'front motor 1 command missing' }
if ($source -notmatch 'front_4340\.ctrl_Mit\(2') { throw 'front motor 2 command missing' }
if ($source -notmatch 'rear_6248\.ctrl_Mit\(1') { throw 'rear motor 1 command missing' }
if ($source -notmatch 'rear_6248\.ctrl_Mit\(2') { throw 'rear motor 2 command missing' }
Write-Output 'up stair integration contract passed'
