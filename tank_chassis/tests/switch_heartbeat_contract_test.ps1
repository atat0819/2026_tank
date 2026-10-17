$header = Get-Content -Raw 'RtosTask/can_send_task.hpp'
$source = Get-Content -Raw 'RtosTask/can_send_task.cpp'

if ($header -notmatch 'extern volatile uint32_t gimbal_switch_last_tick') { exit 1 }
if ($header -notmatch 'extern volatile bool gimbal_switch_received') { exit 1 }
if ($source -notmatch 'void Chassis485_ApplyData') { exit 1 }
if ($source -notmatch 'const uint32_t now\s*=\s*HAL_GetTick\(\)') { exit 1 }
if ($source -notmatch 'gimbal_switch_last_tick\s*=\s*now') { exit 1 }
if ($source -notmatch 'gimbal_switch_received\s*=\s*input_online') { exit 1 }
if ($source -notmatch 'data\.s1 != 0U && data\.s2 != 0U') { exit 1 }
exit 0
