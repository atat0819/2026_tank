#ifndef UP_STAIR_HPP
#define UP_STAIR_HPP

#include "cmsis_os.h"

#ifdef __cplusplus
#include "../user/core/BSP/Motor/DM/DmMotor.hpp"
#include "../user/core/Alg/PID/pid.hpp"
#include "../fsm/stair_mode_policy.hpp"

// 前部两台 J4340 和后部两台 J6248 的达妙电机对象。
extern BSP::Motor::DM::J4340<2> front_4340;
extern BSP::Motor::DM::J6248<2> rear_6248;
// 前左右角度外环，输出 rad/s；内环使用各自 MIT 命令的 Kd。
extern ALG::PID::PID front_4340_left_pid;
extern ALG::PID::PID front_4340_right_pid;
// CAN 接收回调完成初始化后置 true，允许上台阶任务访问电机反馈。
extern volatile bool dm_motor_control_ready;
// CAN 任务发布的模式和前后目标；读写须在同一临界区内完成。
extern StairCommand stair_command;
extern uint32_t stair_command_generation; // 安全停车代次，丢弃停车前采样的命令

extern "C" {
#endif

// 上台阶机构控制任务：执行档位安全判断、状态机更新和四电机闭环输出。
void up_stair_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // UP_STAIR_HPP
