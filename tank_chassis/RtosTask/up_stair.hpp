#ifndef UP_STAIR_HPP
#define UP_STAIR_HPP

#include "cmsis_os.h"

#ifdef __cplusplus
#include "../user/core/BSP/Motor/DM/DmMotor.hpp"
#include "../user/core/Alg/PID/pid.hpp"

// 前部两台 J4310 和后部两台 J6248 的达妙电机对象。
extern BSP::Motor::DM::J4310<2> front_4340;
extern BSP::Motor::DM::J6248<2> rear_6248;
// 前左右位置/速度串级 PID，数组下标 0=位置环、1=速度环。
extern ALG::PID::PID front_4340_left_pid[2];
extern ALG::PID::PID front_4340_right_pid[2];
// CAN 接收回调完成初始化后置 true，允许上台阶任务访问电机反馈。
extern volatile bool dm_motor_control_ready;
// 键盘 B 每次有效按下后递增，供前部状态机检测动作边沿。
extern volatile uint32_t stair_action_sequence;

extern "C" {
#endif

// 上台阶机构控制任务：执行档位安全判断、状态机更新和四电机闭环输出。
void up_stair_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // UP_STAIR_HPP
