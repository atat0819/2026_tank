$ErrorActionPreference = 'Stop'
$source = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\up_stair.cpp')
$header = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\up_stair.hpp')
$canTask = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\can_send_task.cpp')

if ($source -notmatch '#include "../fsm/up_stair_behind_motor_fsm.hpp"') { throw 'rear FSM must be owned by up_stair.cpp' }
if ($source -notmatch 'rear_6248_pitch_pid\[2\]') { throw 'missing rear pitch cascaded PID instances' }
if ($source -notmatch 'rear_6248_roll_pid\[2\]') { throw 'missing rear roll cascaded PID instances' }
if ($source -notmatch 'rear_6248_left_retract_pid\[2\]') { throw 'missing left rear retract cascaded PID instances' }
if ($source -notmatch 'rear_6248_right_retract_pid\[2\]') { throw 'missing right rear retract cascaded PID instances' }
if ($header -notmatch 'rear_retract_action_sequence') { throw 'rear sequence must be declared for the stair task' }
if ($canTask -notmatch 'volatile uint32_t rear_retract_action_sequence') { throw 'CAN task must own rear retract sequence' }
if ($canTask -notmatch 'keyboard_cmd\.rear_retract_toggle[\s\S]*?\+\+rear_retract_action_sequence') { throw 'V edge must increment rear sequence' }
if ($source -notmatch 'EvaluateStairModePolicy') { throw 'task must evaluate the shared stair mode policy' }
if ($source -notmatch 'policy\.rear_retract_command_enabled') { throw 'rear FSM must receive policy-gated V command' }
if ($source -notmatch 'config\.angle_start_rad\[0\]\s*=\s*-3\.14159265358979323846f' -or
    $source -notmatch 'config\.angle_end_rad\[0\]\s*=\s*3\.14159265358979323846f' -or
    $source -notmatch 'config\.angle_start_rad\[1\]\s*=\s*-3\.14159265358979323846f' -or
    $source -notmatch 'config\.angle_end_rad\[1\]\s*=\s*3\.14159265358979323846f') { throw 'both rear J6248 motors must use the signed full angle range during calibration' }
if ($source -notmatch 'GetControlInputSnapshot\(now_tick\)' -or
    $source -notmatch 'input\.source != ControlInputSource::NONE') { throw 'task must use the selected input source heartbeat' }
if ($source -notmatch 'rear_action_sequence\s*=\s*rear_retract_action_sequence') { throw 'rear sequence must be snapshotted in the critical section' }
if ($source -notmatch 'if \(policy\.zero_all_torque\)[\s\S]*?if \(policy\.control_fault\)[\s\S]*?continue;[\s\S]*?\}\s*else\s*\{') { throw 'task must continue only for control faults and use else for normal operation' }
if ($source -notmatch 'if \(policy\.zero_all_torque\)[\s\S]*?front_recovery_fsm\[0\]\.Force_Enable_On_Next_Check\(\)[\s\S]*?front_recovery_fsm\[1\]\.Force_Enable_On_Next_Check\(\)[\s\S]*?rear_recovery_fsm\[0\]\.Force_Enable_On_Next_Check\(\)[\s\S]*?rear_recovery_fsm\[1\]\.Force_Enable_On_Next_Check\(\)') { throw 'safety mode must re-arm all mechanism motors for one enable request on recovery' }
if ($source -notmatch 'Get_Target_Pitch_Deg\(' -or $source -notmatch 'Get_Feedback_Pitch_Deg\(') { throw 'task must consume rear FSM attitude getters' }
if ($source -notmatch 'Uses_Position_Control') { throw 'task must select rear position control by FSM mode' }
if ($source -notmatch 'Uses_Attitude_Control') { throw 'task must preserve rear attitude control by FSM mode' }
if ($source -notmatch 'getVelocityRads\(1\)' -or $source -notmatch 'getVelocityRads\(2\)') { throw 'rear retract speed loops require motor velocity' }
if ($source -notmatch 'else if \(up_stair_behind_motor_fsm\.Uses_Position_Control\(\)\)[\s\S]*?else if \(up_stair_behind_motor_fsm\.Uses_Attitude_Control\(\)\)') { throw 'position and attitude branches must be mutually exclusive' }
if ($source -notmatch 'Get_Position_Target_Angle\(1U\)[\s\S]*?Get_Position_Feedback\(1U\)') { throw 'left position loop must use FSM target and feedback' }
if ($source -notmatch 'Get_Position_Target_Angle\(2U\)[\s\S]*?Get_Position_Feedback\(2U\)') { throw 'right position loop must use FSM target and feedback' }
if ($source -notmatch 'rear_6248_left_retract_pid\[0\]\.reset\(\)[\s\S]*?rear_6248_right_retract_pid\[1\]\.reset\(\)') { throw 'disabled/retract transitions must reset rear retract PID family' }
if ($source -notmatch 'rear_6248_pitch_pid\[0\]\.reset\(\)[\s\S]*?rear_6248_roll_pid\[1\]\.reset\(\)[\s\S]*?Uses_Position_Control') { throw 'position branch must reset inactive attitude PIDs' }
if ($source -notmatch 'Uses_Attitude_Control\(\)[\s\S]*?rear_6248_left_retract_pid\[0\]\.reset\(\)[\s\S]*?rear_6248_right_retract_pid\[1\]\.reset\(\)') { throw 'attitude branch must reset inactive retract PIDs' }
$positionBranch = [regex]::Match($source, 'else if \(up_stair_behind_motor_fsm\.Uses_Position_Control\(\)\)\s*\{(?<body>[\s\S]*?)\}\s*else if \(up_stair_behind_motor_fsm\.Uses_Attitude_Control\(\)\)')
if (-not $positionBranch.Success) { throw 'unable to isolate rear position branch' }
if ($positionBranch.Groups['body'].Value -notmatch 'left_retract_torque[\s\S]*?ClampJ6248Torque') { throw 'position torque must use the existing safety clamp' }
if ($positionBranch.Groups['body'].Value -match 'Get_Motor_Direction') { throw 'position branch must not apply motor direction correction' }
if ($source -notmatch 'retract_target_rad\[0\]\s*=\s*0\.0f' -or
    $source -notmatch 'retract_target_rad\[1\]\s*=\s*0\.0f' -or
    $source -notmatch 'retract_speed_rad_s\s*=\s*0\.0f' -or
    $source -notmatch 'retract_position_tolerance_rad\s*=\s*0\.0f') { throw 'rear retract config must be explicitly fail-safe and uncalibrated' }
if ($source -notmatch 'basic_target_rad\[0\]\s*=\s*0\.0f' -or
    $source -notmatch 'basic_target_rad\[1\]\s*=\s*0\.0f' -or
    $source -notmatch 'basic_speed_rad_s\s*=\s*0\.0f' -or
    $source -notmatch 'basic_position_tolerance_rad\s*=\s*0\.0f') { throw 'rear basic-angle config must be explicitly fail-safe and uncalibrated' }
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
if ($source -notmatch 'now_tick,\s*policy\.rear_retract_command_enabled,\s*rear_action_sequence') { throw 'rear FSM Update must receive policy and action sequence' }
Write-Output 'up stair integration contract passed'
