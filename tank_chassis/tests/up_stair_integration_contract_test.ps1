$ErrorActionPreference = 'Stop'
$source = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\up_stair.cpp')
$header = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\up_stair.hpp')
$rearFsmHeader = Get-Content -Raw (Join-Path $PSScriptRoot '..\fsm\up_stair_behind_motor_fsm.hpp')
$rearFsmImpl = Get-Content -Raw (Join-Path $PSScriptRoot '..\fsm\up_stair_behind_motor_fsm.cpp')
$canTask = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\can_send_task.cpp')

if ($source -notmatch '#include "../fsm/up_stair_behind_motor_fsm.hpp"') { throw 'rear FSM must be owned by up_stair.cpp' }
if ($source -match 'front_4340_(?:left|right)_pid\[' -or
    $header -match 'front_4340_(?:left|right)_pid\[') { throw 'front control must keep only angle outer PID objects' }
if ($source -notmatch 'front_4340\.ctrl_Mit\(1, up_stair_fsm\.Get_Current_Angle\(1\), front_left_target_velocity,\s*0\.0f, 1\.3f, 0\.0f\)' -or
    $source -notmatch 'front_4340\.ctrl_Mit\(2, up_stair_fsm\.Get_Current_Angle\(2\), target_velocity,\s*0\.0f, 1\.3f, 0\.0f\)') { throw 'front angle outputs must feed MIT speed targets with inline Kp=0, Kd=1.3, torque=0' }
if ($source -notmatch 'ALG::PID::PID rear_6248_pitch_pid =') { throw 'missing rear pitch outer PID' }
if ($source -match 'rear_6248_(?:pitch|left_retract|right_retract)_pid\[|rear_6248_limit_speed_pid') { throw 'rear shaft speed feedback must be closed by MIT, not host speed PIDs' }
if ($source -match 'rear_6248_roll_pid|Get_Roll_Rate_Dps') { throw 'weak Roll compensation must not introduce another cascade or Roll rate loop' }
if ($source -match 'REAR_IMU_ROLL_|const float roll_torque_nm') { throw 'Roll parameters and proportional compensation must be owned by the rear FSM' }
if ($source -notmatch 'Get_Raw_Roll_Torque\(1U\)' -or
    $source -notmatch 'Get_Raw_Roll_Torque\(2U\)' -or
    $source -notmatch 'rear_right_velocity,\s*filtered_roll_deg\);') { throw 'task must pass filtered Roll to FSM and use its per-motor correction' }
foreach ($field in @('roll_target_deg', 'roll_kp', 'roll_max_torque_nm')) {
    if ($rearFsmImpl -notmatch "config\.$field\s*=") { throw "FSM must load Roll configuration: $field" }
}
if ($source -notmatch 'Get_Motor_Direction\(1\)\)\s*\*\s*pitch_target_velocity_rad_s' -or
    $source -notmatch 'Get_Motor_Direction\(2\)\)\s*\*\s*pitch_target_velocity_rad_s') { throw 'both rear motors must receive the pitch speed with their own direction' }
if ($source -notmatch 'ALG::PID::PID rear_6248_left_retract_pid =') { throw 'missing left rear retract outer PID' }
if ($source -notmatch 'ALG::PID::PID rear_6248_right_retract_pid =') { throw 'missing right rear retract outer PID' }
if ($header -notmatch 'extern StairCommand stair_command') { throw 'shared explicit mechanism requests must be declared' }
if ($canTask -notmatch 'StairCommand stair_command;') { throw 'CAN task must own mechanism requests' }
if ($canTask -notmatch 'taskENTER_CRITICAL\(\);\s*if \(stair_sample_generation == stair_command_generation\)\s*\{\s*UpdateStairCommand\(stair_command,[\s\S]*?keyboard_cmd\.imu_mode_request,[\s\S]*?keyboard_cmd\.normal_mode_request,[\s\S]*?keyboard_cmd\.stair_toggle\)[\s\S]*?taskEXIT_CRITICAL') { throw 'V/C/B requests must be published atomically' }
if ($source -notmatch 'EvaluateStairModePolicy') { throw 'task must evaluate the shared stair mode policy' }
if ($source -notmatch 'policy\.rear_attitude_enabled && command\.mode == StairMode::IMU') { throw 'NORMAL must gate off rear control, in all rear phases' }
if ($rearFsmHeader -notmatch 'static constexpr float ANGLE_START_RAD\[2\]' -or
    $rearFsmHeader -notmatch 'static constexpr float ANGLE_END_RAD\[2\]' -or
    $rearFsmImpl -notmatch 'config\.angle_start_rad\[i\]\s*=\s*ANGLE_START_RAD\[i\]' -or
    $rearFsmImpl -notmatch 'config\.angle_end_rad\[i\]\s*=\s*ANGLE_END_RAD\[i\]') { throw 'rear mechanical angles must be configured in the rear FSM' }
if ($source -notmatch 'up_stair_behind_motor_fsm\(Class_Up_Stair_Behind_Motor_FSM::Config::WithAngleConstants\(\)\)' -or
    $source -match 'BuildRear6248Config|config\.(?:pitch_zero_deg|angle_start_rad|angle_end_rad|retract_target_rad|retract_position_tolerance_rad)\s*(?:\[|=)') { throw 'task must use rear FSM angle constants without assigning angles' }
if ($source -notmatch 'GetControlInputSnapshot\(now_tick\)' -or
    $source -notmatch 'input\.source != ControlInputSource::NONE') { throw 'task must use the selected input source heartbeat' }
if ($source -notmatch 'taskENTER_CRITICAL\(\);\s*command = stair_command;[\s\S]*?taskEXIT_CRITICAL') { throw 'front/rear requests must use one protected snapshot' }
if ($source -notmatch 'if \(policy\.zero_all_torque\)[\s\S]*?if \(policy\.control_fault\)[\s\S]*?continue;[\s\S]*?\}\s*else\s*\{') { throw 'task must continue only for control faults and use else for normal operation' }
if ($source -notmatch 'if \(policy\.zero_all_torque\)[\s\S]*?front_recovery_fsm\[0\]\.Force_Enable_On_Next_Check\(\)[\s\S]*?front_recovery_fsm\[1\]\.Force_Enable_On_Next_Check\(\)[\s\S]*?rear_recovery_fsm\[0\]\.Force_Enable_On_Next_Check\(\)[\s\S]*?rear_recovery_fsm\[1\]\.Force_Enable_On_Next_Check\(\)') { throw 'safety mode must re-arm all mechanism motors for one enable request on recovery' }
if ($source -notmatch 'rear_6248_pitch_pid\.UpDate\(\s*up_stair_behind_motor_fsm\.Get_Target_Pitch_Rad\(\),\s*up_stair_behind_motor_fsm\.Get_Feedback_Pitch_Rad\(\)\);') { throw 'Pitch outer PID must consume radians and output rad/s directly' }
if ($source -notmatch 'Uses_Position_Control') { throw 'task must select rear position control by FSM mode' }
if ($source -notmatch 'Uses_Attitude_Control') { throw 'task must preserve rear attitude control by FSM mode' }
if ($source -notmatch 'getVelocityRads\(1\)' -or $source -notmatch 'getVelocityRads\(2\)') { throw 'rear retract speed loops require motor velocity' }
if ($source -notmatch 'else if \(up_stair_behind_motor_fsm\.Uses_Position_Control\(\)\)[\s\S]*?else if \(up_stair_behind_motor_fsm\.Uses_Attitude_Control\(\)\)') { throw 'position and attitude branches must be mutually exclusive' }
if ($source -notmatch 'Get_Position_Target_Angle\(1U\)[\s\S]*?Get_Position_Feedback\(1U\)') { throw 'left position loop must use FSM target and feedback' }
if ($source -notmatch 'Get_Position_Target_Angle\(2U\)[\s\S]*?Get_Position_Feedback\(2U\)') { throw 'right position loop must use FSM target and feedback' }
if ($source -notmatch 'rear_6248_left_retract_pid\.reset\(\)[\s\S]*?rear_6248_right_retract_pid\.reset\(\)') { throw 'disabled transitions must reset rear outer PIDs' }
if ($source -match 'Update_Limit_Recovery|Uses_Limit_Position_Control|Get_Limit_Attitude_Blend|rear_limit_position_active' -or
    $rearFsmImpl -match 'LIMIT_SLOW_ZONE|LIMIT_BACKOFF|LIMIT_RETURNING|LIMIT_BRAKE_GAIN') { throw 'rear endpoints must not decelerate, back off, latch or hand off position control' }
$positionBranch = [regex]::Match($source, 'else if \(up_stair_behind_motor_fsm\.Uses_Position_Control\(\)\)\s*\{(?<body>[\s\S]*?)\}\s*(?://[^\r\n]*\r?\n\s*)*else if \(up_stair_behind_motor_fsm\.Uses_Attitude_Control\(\)\)')
if (-not $positionBranch.Success) { throw 'unable to isolate rear position branch' }
if ($positionBranch.Groups['body'].Value -notmatch 'rear_6248_pitch_pid\.reset\(\)') { throw 'position branch must reset inactive pitch PID' }
if ($positionBranch.Groups['body'].Value -notmatch 'Limit_Velocity\(1U, left_target_velocity, REAR_RETRACT_MIT_KD\)' -or
    $positionBranch.Groups['body'].Value -notmatch 'Limit_Velocity\(2U, right_target_velocity, REAR_RETRACT_MIT_KD\)') { throw 'position speeds must use rear FSM constraints with the retract Kd' }
if ($positionBranch.Groups['body'].Value -match 'Get_Motor_Direction') { throw 'position branch must not apply motor direction correction' }
if ($rearFsmHeader -notmatch 'static constexpr float PITCH_ZERO_DEG' -or
    $rearFsmHeader -notmatch 'static constexpr float RETRACT_TARGET_RAD\[2\]' -or
    $rearFsmHeader -notmatch 'static constexpr float RETRACT_POSITION_TOLERANCE_RAD' -or
    $rearFsmImpl -notmatch 'config\.retract_target_rad\[i\]\s*=\s*RETRACT_TARGET_RAD\[i\]') { throw 'rear retract angles must remain in the rear FSM header' }
if ($rearFsmHeader -match 'BASIC_ANGLE_CONTROL|BASIC_TARGET_RAD' -or
    $rearFsmImpl -match 'Start_Basic_Angle_Control') { throw 'removed IMU fallback must not return' }
if ($rearFsmImpl -notmatch 'if \(!control_enabled \|\| !config_valid_ \|\| !imu_ready\)\s*\{\s*Disable\(\);\s*return;') { throw 'invalid IMU must disable all rear control before retract selection' }
if ($source -notmatch 'Limit_Velocity\(1U, left_pitch_velocity, REAR_IMU_MIT_KD, left_attitude_feedforward_nm\)' -or
    $source -notmatch 'Limit_Velocity\(2U, right_pitch_velocity, REAR_IMU_MIT_KD, right_attitude_feedforward_nm\)') { throw 'both pitch speeds must protect combined Pitch/Roll feedforward with the matching Kd' }
if ($rearFsmImpl -notmatch 'Limit_Torque\(id, kd \* \(target - velocity\) \+ raw_feedforward\)' -or
    $rearFsmImpl -notmatch 'velocity \+ \(torque - feedforward\) / kd') { throw 'MIT speed constraints must protect the total torque and subtract the sent feedforward' }
foreach ($kd in @('REAR_IMU_MIT_KD', 'REAR_RETRACT_MIT_KD')) {
    if ([regex]::Matches($source, "(?m)^float $kd\s*=").Count -ne 1) { throw "rear MIT damping must have one definition per mode: $kd" }
    if ([regex]::Matches($source, "Limit_Velocity\([^\r\n]+, $kd(?:, [^\r\n)]+)?\),\s*0\.0f, $kd,").Count -ne 2) { throw "both motors must use $kd consistently between limiting and sending" }
}
foreach ($id in @('1', '2')) {
    if ($positionBranch.Groups['body'].Value -notmatch "0\.0f, REAR_RETRACT_MIT_KD, up_stair_behind_motor_fsm\.Get_Retract_Feedforward_Torque\(${id}U\)\)") { throw "retract motor $id must send the FSM feedforward directly" }
}
if ($source -notmatch 'isConnected\(1, 7\)' -or $source -notmatch 'isConnected\(2, 8\)') { throw 'rear connectivity alarm IDs must use CAN ids 7 and 8' }
if ($source -notmatch 'now_tick = HAL_GetTick\(\);') { throw 'heartbeat snapshot must capture task time in protected snapshot' }
if ($source -notmatch 'imu_snapshot\.valid' -or $source -notmatch 'imu_snapshot\.tick < IMU_CONTROL_TIMEOUT_MS') { throw 'rear control must reject stale/invalid IMU snapshots' }
if ($source -notmatch 'front_4340\.ctrl_Mit\(1') { throw 'front motor 1 command missing' }
if ($source -notmatch 'front_4340\.ctrl_Mit\(2') { throw 'front motor 2 command missing' }
if ($source -notmatch 'rear_6248\.ctrl_Mit\(1') { throw 'rear motor 1 command missing' }
if ($source -notmatch 'rear_6248\.ctrl_Mit\(2') { throw 'rear motor 2 command missing' }
if ($source -notmatch 'now_tick,\s*command\.rear_retract_requested') { throw 'rear FSM must receive the explicit retract request' }
if ($source -notmatch 'command\.rear_retract_requested,\s*rear_left_velocity,\s*rear_right_velocity') { throw 'soft limits require actual motor speeds' }
if ($rearFsmImpl -notmatch 'at_lower_margin && torque < 0\.0f' -or
    $rearFsmImpl -notmatch 'at_upper_margin && torque > 0\.0f') { throw 'rear endpoints must block only outward torque' }
Write-Output 'up stair integration contract passed'
