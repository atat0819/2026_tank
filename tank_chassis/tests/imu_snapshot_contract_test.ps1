$ErrorActionPreference = 'Stop'
$source = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\remote_task.cpp')
$header = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\remote_task.hpp')
if ($source -notmatch 'taskENTER_CRITICAL\(\);') { throw 'IMU publication must be protected' }
if ($source -notmatch 'imu_control_snapshot\.valid = false') { throw 'failed IMU reads must invalidate snapshot' }
if ($source -match 'bmi088|BMI088|hspi2') { throw 'active feedback must no longer depend on BMI088' }
if ($source -notmatch 'PublishHi14ImuFromISR' -or $source -notmatch 'if \(valid && finite\)') { throw 'only valid finite HI14 frames may publish control feedback' }
if ($source -notmatch 'taskENTER_CRITICAL_FROM_ISR\(\)') { throw 'UART ISR must publish coherently' }
if ($source -match 'xImuHandle|vTaskNotifyGiveFromISR') { throw 'removed IMU task must no longer receive notifications' }
if ($source -notmatch 'HAL_GetTick\(\) - snapshot\.tick < IMU_CONTROL_TIMEOUT_MS') { throw 'old HI14 frames must expire at the shared timeout' }
if ($header -notmatch 'GetImuControlSnapshot') { throw 'IMU snapshot getter missing' }
if ($source -notmatch 'std::isfinite\(pitch\)\s*&&\s*std::isfinite\(pitch_rate\)') { throw 'rear control snapshot validity must depend on pitch data only' }
$rtos = Get-Content -Raw (Join-Path $PSScriptRoot '..\Core\Src\freertos.c')
if ($rtos -match 'xTaskCreate\(imu_task|xImuHandle|start_imu') { throw 'IMU telemetry task must no longer be created' }

if ($header -notmatch 'constexpr uint32_t IMU_CONTROL_TIMEOUT_MS = 100U') { throw 'IMU timeout must be configured once as 100ms' }
Write-Output 'imu snapshot contract passed'
