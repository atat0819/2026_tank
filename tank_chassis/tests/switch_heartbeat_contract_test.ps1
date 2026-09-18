$header = Get-Content -Raw 'RtosTask/can_send_task.hpp'
$source = Get-Content -Raw 'RtosTask/can_send_task.cpp'

if ($header -notmatch 'extern volatile uint32_t gimbal_switch_last_tick') { exit 1 }
if ($header -notmatch 'extern volatile bool gimbal_switch_received') { exit 1 }
if ($source -notmatch 'gimbal_switch_last_tick\s*=\s*HAL_GetTick\(\)') { exit 1 }
if ($source -notmatch 'gimbal_switch_received\s*=\s*true') { exit 1 }
if ($source -notmatch 'frame\.id\s*==\s*0x303\s*&&\s*frame\.dlc\s*>=\s*2U') { exit 1 }
exit 0
