#ifndef UP_STAIR_HPP
#define UP_STAIR_HPP

#include "cmsis_os.h"

#ifdef __cplusplus
#include "../user/core/BSP/Motor/DM/DmMotor.hpp"
#include "../user/core/Alg/PID/pid.hpp"

extern BSP::Motor::DM::J4310<2> front_4340;
extern BSP::Motor::DM::J6248<2> rear_6248;
extern ALG::PID::PID front_4340_left_pid[2];
extern ALG::PID::PID front_4340_right_pid[2];
extern volatile bool dm_motor_control_ready;
extern volatile uint32_t stair_action_sequence;

extern "C" {
#endif

void up_stair_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // UP_STAIR_HPP
