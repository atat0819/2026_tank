#include "UI_task.hpp"

#include "remote_task.hpp"
#include "../user/core/APP/Referee/RM_RefereeSystem.h"
#include "../user/core/APP/Referee/RMRefereeSystemUI.hpp"

extern "C" void ui_task(void *argument)
{
    (void)argument;

    // Wait for the referee UART task to receive the first valid frame.
    osDelay(500);

    for (;;)
    {
        if (!RM_RefereeSystemDirFlag &&
            ext_power_heat_data_0x0201.robot_id != 0)
        {
            RMRefereeSystemUI::UpdatePowerStatus(
                PowerData.power,
                ext_power_heat_data_0x0201.chassis_power_limit,
                ext_power_heat_data_0x0202.chassis_power_buffer);
            RMRefereeSystemUI::UpdateHealthStatus(
                ext_power_heat_data_0x0201.current_HP,
                ext_power_heat_data_0x0201.maximum_HP);
            RMRefereeSystemUI::UpdateAimGuides();
        }
        else
        {
            RMRefereeSystemUI::ResetPowerStatus();
            RMRefereeSystemUI::ResetHealthStatus();
            RMRefereeSystemUI::ResetAimGuides();
        }

        osDelay(200);
    }
}
