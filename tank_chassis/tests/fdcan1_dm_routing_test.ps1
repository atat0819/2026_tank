$ErrorActionPreference = 'Stop'

$upStair = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\up_stair.cpp')
$canTask = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\can_send_task.cpp')

if ($upStair -notmatch 'J4310<2>[\s\S]*?HAL::FDCAN::FdcanDeviceId::HAL_Fdcan1') {
    throw 'front_4340 is not bound to FDCAN1.'
}

if ($canTask -notmatch 'fdcan1\.register_rx_callback\(\[\]\(const HAL::FDCAN::Frame &frame\)[\s\S]*?frame\.id >= 0x01 && frame\.id <= 0x02[\s\S]*?front_4340\.Parse\(frame\)') {
    throw 'FDCAN1 does not route DM feedback IDs 0x01-0x02 to front_4340.'
}

if ($canTask -notmatch 'frame\.id >= 0x201 && frame\.id <= 0x204[\s\S]*?chassis_motor\.Parse\(frame\)') {
    throw 'FDCAN1 no longer routes 3508 feedback IDs 0x201-0x204 to chassis_motor.'
}
