$ErrorActionPreference = 'Stop'

$upStair = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\up_stair.cpp')
$canTask = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\can_send_task.cpp')
$dmMotor = Get-Content -Raw (Join-Path $PSScriptRoot '..\user\core\BSP\Motor\DM\DmMotor.hpp')

if ($upStair -notmatch 'J4340<2> front_4340\(\s*0x00, \{0x05, 0x06\}, \{0x05, 0x06\}, HAL::FDCAN::FdcanDeviceId::HAL_Fdcan2\)') {
    throw 'front_4340 must use FDCAN2 with TX/RX IDs 0x05-0x06.'
}

if ($dmMotor -notmatch 'class J4340[\s\S]*?Parameters\(-3\.14159f,\s*3\.14159f,\s*-10\.0f,\s*10\.0f,\s*-27\.0f,\s*27\.0f,\s*0\.0f,\s*500\.0f,\s*0\.0f,\s*5\.0f\)') {
    throw 'J4340 MIT ranges must match P=-3.14159..3.14159, V=+/-10, T=+/-27.'
}

$leftPid = [regex]::Match($upStair, 'front_4340_left_pid\s*=\s*\{(?<body>[\s\S]*?)\};')
$rightPid = [regex]::Match($upStair, 'front_4340_right_pid\s*=\s*\{(?<body>[\s\S]*?)\};')
if (-not $leftPid.Success -or -not $rightPid.Success) { throw 'front 4340 angle outer PIDs are missing.' }
if ($leftPid.Groups['body'].Value -notmatch '15\.0f,\s*0\.0f,\s*0\.0f,\s*10\.0f') { throw 'left front angle Kp and velocity limit must remain 15 and 10.' }
if ($rightPid.Groups['body'].Value -notmatch '8\.0f,\s*0\.0f,\s*0\.0f,\s*10\.0f') { throw 'right front angle Kp and velocity limit must remain 8 and 10.' }

$fdcan1Rx = [regex]::Match($canTask, 'fdcan1\.register_rx_callback\([\s\S]*?\n\s*\}\);').Value
$fdcan2Rx = [regex]::Match($canTask, 'fdcan2\.register_rx_callback\([\s\S]*?\n\s*\}\);').Value
$fdcan3Rx = [regex]::Match($canTask, 'fdcan3\.register_rx_callback\([\s\S]*?\n\s*\}\);').Value
if ($canTask -notmatch 'auto &fdcan2 = [^;]*HAL::FDCAN::FdcanDeviceId::HAL_Fdcan2') {
    throw 'FDCAN2 receive device is missing.'
}
if ($fdcan2Rx -notmatch 'frame\.id >= 0x05 && frame\.id <= 0x06[\s\S]*?front_4340\.Parse\(frame\)') {
    throw 'FDCAN2 does not route DM feedback IDs 0x05-0x06 to front_4340.'
}
if ($fdcan1Rx -match 'front_4340\.Parse') {
    throw 'FDCAN1 must no longer dispatch front motor feedback.'
}

if ($fdcan1Rx -notmatch 'frame\.id >= 0x201 && frame\.id <= 0x204[\s\S]*?chassis_motor\.Parse\(frame\)') {
    throw 'FDCAN1 no longer routes 3508 feedback IDs 0x201-0x204 to chassis_motor.'
}
if ($fdcan1Rx -notmatch 'frame\.id == 0x777[\s\S]*?supercap\.parse\(frame\)') {
    throw 'FDCAN1 supercap feedback routing is missing.'
}
if ($upStair -notmatch 'J6248<2> rear_6248\(\s*0x00, \{0x07, 0x08\}, \{0x07, 0x08\}, HAL::FDCAN::FdcanDeviceId::HAL_Fdcan3\)' -or
    $fdcan3Rx -notmatch 'frame\.id >= 0x07 && frame\.id <= 0x08[\s\S]*?rear_6248\.Parse\(frame\)') {
    throw 'Rear motors must retain FDCAN3 TX/RX IDs 0x07-0x08.'
}
Write-Host 'FDCAN motor routing contracts passed.'
