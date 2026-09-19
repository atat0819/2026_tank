$ErrorActionPreference = 'Stop'
$source = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\up_stair.cpp')

if ($source -notmatch '#include "../fsm/up_stair_behind_motor_fsm.hpp"') { throw 'rear FSM must be owned by up_stair.cpp' }
if ($source -notmatch 'rear_6248_pitch_pid\[2\]') { throw 'missing rear pitch cascaded PID instances' }
if ($source -notmatch 'rear_6248_roll_pid\[2\]') { throw 'missing rear roll cascaded PID instances' }
if ($source -notmatch 'EvaluateStairModePolicy') { throw 'task must evaluate the shared stair mode policy' }
if ($source -notmatch 'gimbal_switch_last_tick') { throw 'task must use switch heartbeat' }
if ($source -notmatch 'if \(policy\.zero_all_torque\)[\s\S]*?if \(policy\.control_fault\)[\s\S]*?continue;[\s\S]*?\}\s*else\s*\{') { throw 'task must continue only for control faults and use else for normal operation' }
if ($source -notmatch 'Get_Target_Pitch_Deg\(' -or $source -notmatch 'Get_Feedback_Pitch_Deg\(') { throw 'task must consume rear FSM attitude getters' }
if ($source -notmatch 'Limit_Torque\(\s*1') { throw 'task must route left torque through rear FSM safety limit' }
if ($source -notmatch 'Limit_Torque\(\s*2') { throw 'task must route right torque through rear FSM safety limit' }
if ($source -notmatch 'ClampJ6248Torque\(\s*up_stair_behind_motor_fsm\.Limit_Torque\(1, left_mixed_torque\)\)') { throw 'left rear torque must receive FSM gain then final J6248 clamp' }
if ($source -notmatch 'ClampJ6248Torque\(\s*up_stair_behind_motor_fsm\.Limit_Torque\(2, right_mixed_torque\)\)') { throw 'right rear torque must receive FSM gain then final J6248 clamp' }
if ($source -notmatch 'isConnected\(1, 3\)' -or $source -notmatch 'isConnected\(2, 4\)') { throw 'rear connectivity must use CAN ids 3 and 4' }
if ($source -notmatch 'now_tick = HAL_GetTick\(\);') { throw 'heartbeat snapshot must capture task time in protected snapshot' }
if ($source -notmatch 'imu_snapshot\.valid' -or $source -notmatch 'imu_snapshot\.tick < 20U') { throw 'rear control must reject stale/invalid IMU snapshots' }
if ($source -notmatch 'front_4340\.ctrl_Mit\(1') { throw 'front motor 1 command missing' }
if ($source -notmatch 'front_4340\.ctrl_Mit\(2') { throw 'front motor 2 command missing' }
if ($source -notmatch 'rear_6248\.ctrl_Mit\(1') { throw 'rear motor 1 command missing' }
if ($source -notmatch 'rear_6248\.ctrl_Mit\(2') { throw 'rear motor 2 command missing' }
Write-Output 'up stair integration contract passed'
