$ErrorActionPreference = 'Stop'

$source = Get-Content -Raw (Join-Path $PSScriptRoot '..\RtosTask\up_stair.cpp')
$enableIndex = $source.IndexOf('if (left_enable_requested)')
$controlIndex = $source.IndexOf('if (left_feedback_valid && up_stair_fsm.Is_Enabled())')

if ($enableIndex -lt 0) {
    throw 'ID1 enable dispatch is missing.'
}

if ($controlIndex -lt 0) {
    throw 'ID1 control branch is missing.'
}

if ($enableIndex -gt $controlIndex) {
    throw 'ID1 enable dispatch is inside or after the control branch and misses online recovery.'
}
