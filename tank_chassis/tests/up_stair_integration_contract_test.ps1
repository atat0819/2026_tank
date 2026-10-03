$ErrorActionPreference = 'Stop'
$source = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\up_stair.cpp')
$header = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\up_stair.hpp')
$rearFsmHeader = Get-Content -Raw (Join-Path $PSScriptRoot '..\fsm\up_stair_behind_motor_fsm.hpp')
$rearFsmImpl = Get-Content -Raw (Join-Path $PSScriptRoot '..\fsm\up_stair_behind_motor_fsm.cpp')
$canTask = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\can_send_task.cpp')

if ($source -notmatch '#include "../fsm/up_stair_behind_motor_fsm.hpp"') { throw 'rear FSM must be owned by up_stair.cpp' }
if ($source -notmatch 'rear_6248_pitch_pid\[2\]') { throw 'missing rear pitch cascaded PID instances' }
if ($source -match 'rear_6248_roll_pid|roll_torque|Get_Target_Roll_Deg|Get_Feedback_Roll_Deg|Get_Roll_Rate_Dps') { throw 'rear attitude control must not use roll feedback or differential torque' }
if ($source -notmatch 'Get_Motor_Direction\(1\)\)\s*\*\s*pitch_torque' -or
    $source -notmatch 'Get_Motor_Direction\(2\)\)\s*\*\s*pitch_torque') { throw 'both rear motors must receive the pitch torque with their own direction' }
if ($source -notmatch 'rear_6248_left_retract_pid\[2\]') { throw 'missing left rear retract cascaded PID instances' }
if ($source -notmatch 'rear_6248_right_retract_pid\[2\]') { throw 'missing right rear retract cascaded PID instances' }
if ($header -notmatch 'rear_retract_action_sequence') { throw 'rear sequence must be declared for the stair task' }
if ($canTask -notmatch 'volatile uint32_t rear_retract_action_sequence') { throw 'CAN task must own rear retract sequence' }
if ($canTask -notmatch 'keyboard_cmd\.rear_retract_toggle[\s\S]*?\+\+rear_retract_action_sequence') { throw 'V edge must increment rear sequence' }
if ($source -notmatch 'EvaluateStairModePolicy') { throw 'task must evaluate the shared stair mode policy' }
if ($source -notmatch 'policy\.rear_retract_command_enabled') { throw 'rear FSM must receive policy-gated V command' }
if ($rearFsmHeader -notmatch 'static constexpr float ANGLE_START_RAD\[2\]' -or
    $rearFsmHeader -notmatch 'static constexpr float ANGLE_END_RAD\[2\]' -or
    $rearFsmImpl -notmatch 'config\.angle_start_rad\[i\]\s*=\s*ANGLE_START_RAD\[i\]' -or
    $rearFsmImpl -notmatch 'config\.angle_end_rad\[i\]\s*=\s*ANGLE_END_RAD\[i\]') { throw 'rear mechanical angles must be configured in the rear FSM' }
if ($source -notmatch 'up_stair_behind_motor_fsm\(Class_Up_Stair_Behind_Motor_FSM::Config::WithAngleConstants\(\)\)' -or
    $source -match 'BuildRear6248Config|config\.(?:pitch_zero_deg|angle_start_rad|angle_end_rad|retract_target_rad|basic_target_rad|retract_position_tolerance_rad|basic_position_tolerance_rad)\s*(?:\[|=)') { throw 'task must use rear FSM angle constants without assigning angles' }
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
if ($source -notmatch 'rear_6248_pitch_pid\[0\]\.reset\(\)[\s\S]*?rear_6248_pitch_pid\[1\]\.reset\(\)[\s\S]*?Uses_Position_Control') { throw 'position branch must reset inactive pitch PIDs' }
if ($source -notmatch 'Uses_Attitude_Control\(\)[\s\S]*?rear_6248_left_retract_pid\[0\]\.reset\(\)[\s\S]*?rear_6248_right_retract_pid\[1\]\.reset\(\)') { throw 'attitude branch must reset inactive retract PIDs' }
$positionBranch = [regex]::Match($source, 'else if \(up_stair_behind_motor_fsm\.Uses_Position_Control\(\)\)\s*\{(?<body>[\s\S]*?)\}\s*(?://[^\r\n]*\r?\n\s*)*else if \(up_stair_behind_motor_fsm\.Uses_Attitude_Control\(\)\)')
if (-not $positionBranch.Success) { throw 'unable to isolate rear position branch' }
if ($positionBranch.Groups['body'].Value -notmatch 'left_retract_torque[\s\S]*?ClampJ6248Torque') { throw 'position torque must use the existing safety clamp' }
if ($positionBranch.Groups['body'].Value -match 'Get_Motor_Direction') { throw 'position branch must not apply motor direction correction' }
if ($rearFsmHeader -notmatch 'static constexpr float PITCH_ZERO_DEG' -or
    $rearFsmHeader -notmatch 'static constexpr float RETRACT_TARGET_RAD\[2\]' -or
    $rearFsmHeader -notmatch 'static constexpr float RETRACT_POSITION_TOLERANCE_RAD' -or
    $rearFsmHeader -notmatch 'static constexpr float BASIC_TARGET_RAD\[2\]' -or
    $rearFsmHeader -notmatch 'static constexpr float BASIC_POSITION_TOLERANCE_RAD' -or
    $rearFsmImpl -notmatch 'config\.pitch_zero_deg\s*=\s*PITCH_ZERO_DEG' -or
    $rearFsmImpl -notmatch 'config\.retract_target_rad\[i\]\s*=\s*RETRACT_TARGET_RAD\[i\]' -or
    $rearFsmImpl -notmatch 'config\.basic_target_rad\[i\]\s*=\s*BASIC_TARGET_RAD\[i\]' -or
    $rearFsmImpl -notmatch 'config\.retract_position_tolerance_rad\s*=\s*RETRACT_POSITION_TOLERANCE_RAD' -or
    $rearFsmImpl -notmatch 'config\.basic_position_tolerance_rad\s*=\s*BASIC_POSITION_TOLERANCE_RAD') { throw 'rear angle targets and tolerances must be configured in the rear FSM header' }
if ($rearFsmImpl -notmatch 'retract_speed_rad_s\(0\.0f\)' -or
    $rearFsmImpl -notmatch 'basic_speed_rad_s\(0\.0f\)') { throw 'uncalibrated rear position control speeds must remain zero' }
if ($source -notmatch 'Limit_Torque\(\s*1') { throw 'task must route left torque through rear FSM safety limit' }
if ($source -notmatch 'Limit_Torque\(\s*2') { throw 'task must route right torque through rear FSM safety limit' }
if ($source -notmatch 'ClampJ6248Torque\(\s*up_stair_behind_motor_fsm\.Limit_Torque\(1, left_pitch_torque\)\)') { throw 'left rear pitch torque must receive FSM gain then final J6248 clamp' }
if ($source -notmatch 'ClampJ6248Torque\(\s*up_stair_behind_motor_fsm\.Limit_Torque\(2, right_pitch_torque\)\)') { throw 'right rear pitch torque must receive FSM gain then final J6248 clamp' }
if ($source -notmatch 'isConnected\(1, 7\)' -or $source -notmatch 'isConnected\(2, 8\)') { throw 'rear connectivity alarm IDs must use CAN ids 7 and 8' }
if ($source -notmatch 'now_tick = HAL_GetTick\(\);') { throw 'heartbeat snapshot must capture task time in protected snapshot' }
if ($source -notmatch 'imu_snapshot\.valid' -or $source -notmatch 'imu_snapshot\.tick < 20U') { throw 'rear control must reject stale/invalid IMU snapshots' }
if ($source -notmatch 'front_4340\.ctrl_Mit\(1') { throw 'front motor 1 command missing' }
if ($source -notmatch 'front_4340\.ctrl_Mit\(2') { throw 'front motor 2 command missing' }
if ($source -notmatch 'rear_6248\.ctrl_Mit\(1') { throw 'rear motor 1 command missing' }
if ($source -notmatch 'rear_6248\.ctrl_Mit\(2') { throw 'rear motor 2 command missing' }
if ($source -notmatch 'now_tick,\s*policy\.rear_retract_command_enabled,\s*rear_action_sequence') { throw 'rear FSM Update must receive policy and action sequence' }
Write-Output 'up stair integration contract passed'
