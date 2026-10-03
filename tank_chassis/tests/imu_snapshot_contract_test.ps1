$ErrorActionPreference = 'Stop'
$source = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\imu_task.cpp')
$header = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\imu_task.hpp')
if ($source -notmatch 'taskENTER_CRITICAL\(\);') { throw 'IMU publication must be protected' }
if ($source -notmatch 'imu_control_snapshot\.valid = false') { throw 'failed IMU reads must invalidate snapshot' }
if ($source -notmatch 'bmi088_init_status == HAL_OK &&') { throw 'only successful IMU reads may publish valid data' }
if ($header -notmatch 'GetImuControlSnapshot') { throw 'IMU snapshot getter missing' }
if ($header -notmatch 'float roll_deg;' -or $header -notmatch 'float roll_rate_dps;') { throw 'IMU snapshot must include roll data for VOFA observation' }
if ($source -notmatch 'imu_control_snapshot\.roll_deg = bmi088\.GetRollAngleDeg\(\)' -or
    $source -notmatch 'imu_control_snapshot\.roll_rate_dps = bmi088\.GetGyroRateXDps\(\)') { throw 'roll observation must be published with the pitch snapshot' }
if ($source -notmatch 'std::isfinite\(pitch\)\s*&&\s*std::isfinite\(pitch_rate\)') { throw 'rear control snapshot validity must depend on pitch data only' }
foreach ($axis in @('x', 'y', 'z')) {
    $getterAxis = $axis.ToUpperInvariant()
    if ($header -notmatch "float accel_${axis}_mps2;" -or
        $source -notmatch "imu_control_snapshot\.accel_${axis}_mps2 = bmi088\.GetAccel${getterAxis}Mps2\(\)") { throw "IMU snapshot must include acceleration $axis for VOFA" }
}
if ($header -notmatch 'float yaw_rate_dps;' -or
    $source -notmatch 'imu_control_snapshot\.yaw_rate_dps = bmi088\.GetGyroRateZDps\(\)') { throw 'IMU snapshot must include gyro Z for VOFA' }
Write-Output 'imu snapshot contract passed'
