#ifndef GANTRY_TASK_HPP
#define GANTRY_TASK_HPP

#ifdef __cplusplus
#include "../user/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../user/core/BSP/Motor/DM/DmMotor.hpp"
#include "../feeder_fsm/gantry_fsm.hpp"

extern BSP::Motor::Dji::GM3508<2> gantry_3508_motor;
extern BSP::Motor::DM::J4310<1> gantry_dm4310_motor;

struct GantryWatch
{
    double angle_deg[3]; // Keil Watch：累计角度 deg，ID5、ID6 为输出轴端，ID9 为 DM 轴
    float velocity_rad_s[3]; // rad/s，轴端与 angle_deg 一致
    bool online[3];
    bool limit[5];      // PA0..4 消抖状态
    bool configured;
    uint8_t dm_status;  // 达妙反馈高四位：1=使能，0=失能，8..E=故障
    GantryState state;
    GantryFault fault;
    float progress;
    float scale;
};
extern volatile GantryWatch gantry_watch;

extern "C" {
#endif

void gantry_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // GANTRY_TASK_HPP
