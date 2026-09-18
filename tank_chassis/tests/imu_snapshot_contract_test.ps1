$ErrorActionPreference = 'Stop'
$source = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\imu_task.cpp')
$header = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\imu_task.hpp')
if ($source -notmatch 'taskENTER_CRITICAL\(\);') { throw 'IMU publication must be protected' }
if ($source -notmatch 'imu_control_snapshot\.valid = false') { throw 'failed IMU reads must invalidate snapshot' }
if ($source -notmatch 'bmi088_init_status == HAL_OK &&') { throw 'only successful IMU reads may publish valid data' }
if ($header -notmatch 'GetImuControlSnapshot') { throw 'IMU snapshot getter missing' }
Write-Output 'imu snapshot contract passed'
