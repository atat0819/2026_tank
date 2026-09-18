$ErrorActionPreference = 'Stop'

$source = Get-Content -Raw "$PSScriptRoot/../user/core/BSP/Motor/DM/DmMotor.hpp"

if ($source -notmatch 'ClampMitCommandValue') {
    throw 'Missing generic MIT command clamp helper.'
}

if ($source -notmatch 'x\s*=\s*ClampMitCommandValue\(x,\s*x_min,\s*x_max\)') {
    throw 'float_to_uint must clamp its input before encoding.'
}

Write-Output 'dm motor clamp contract passed'
