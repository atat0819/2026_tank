$ErrorActionPreference = 'Stop'

$upStair = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\up_stair.cpp')
$canTask = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\can_send_task.cpp')
$dmMotor = Get-Content -Raw (Join-Path $PSScriptRoot '..\user\core\BSP\Motor\DM\DmMotor.hpp')

if ($upStair -notmatch 'J4340<2>[\s\S]*?HAL::FDCAN::FdcanDeviceId::HAL_Fdcan1') {
    throw 'front_4340 is not bound to FDCAN1.'
}

if ($dmMotor -notmatch 'class J4340[\s\S]*?Parameters\(-3\.14159f,\s*3\.14159f,\s*-10\.0f,\s*10\.0f,\s*-27\.0f,\s*27\.0f,\s*0\.0f,\s*500\.0f,\s*0\.0f,\s*5\.0f\)') {
    throw 'J4340 MIT ranges must match P=-3.14159..3.14159, V=+/-10, T=+/-27.'
}

$leftPid = [regex]::Match($upStair, 'front_4340_left_pid\[2\]\s*=\s*\{(?<body>[\s\S]*?)\};')
$rightPid = [regex]::Match($upStair, 'front_4340_right_pid\[2\]\s*=\s*\{(?<body>[\s\S]*?)\};')
if (-not $leftPid.Success -or -not $rightPid.Success) { throw 'front 4340 PID arrays are missing.' }
if ($leftPid.Groups['body'].Value -notmatch '\{15\.0f,\s*0\.0f,\s*0\.0f,\s*10\.0f[\s\S]*?\{3\.0f,\s*0\.0f,\s*0\.0f,\s*27\.0f') { throw 'left front PID limits must be velocity 10 and torque 27.' }
if ($rightPid.Groups['body'].Value -notmatch '\{8\.0f,\s*0\.0f,\s*0\.0f,\s*10\.0f[\s\S]*?\{0\.8f,\s*0\.0f,\s*0\.0f,\s*27\.0f') { throw 'right front PID limits must be velocity 10 and torque 27.' }

if ($canTask -notmatch 'fdcan1\.register_rx_callback\(\[\]\(const HAL::FDCAN::Frame &frame\)[\s\S]*?frame\.id >= 0x05 && frame\.id <= 0x06[\s\S]*?front_4340\.Parse\(frame\)') {
    throw 'FDCAN1 does not route DM feedback IDs 0x05-0x06 to front_4340.'
}

if ($canTask -notmatch 'frame\.id >= 0x201 && frame\.id <= 0x204[\s\S]*?chassis_motor\.Parse\(frame\)') {
    throw 'FDCAN1 no longer routes 3508 feedback IDs 0x201-0x204 to chassis_motor.'
}
