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
    float lift_current_a[2]; // C620 实际转矩电流 A，依次 ID5、ID6，保留正负号
    bool online[3];
    bool limit[4];      // PE13、PE9、PA2、PA3 消抖状态；下降原点由反馈堵转判定
    bool configured;
    uint8_t dm_status;  // 达妙反馈高四位：1=使能，0=失能，8..E=故障
    GantryState state;
    GantryFault fault;
    float progress;
    float scale;
    bool limit_raw[4]; // PE13、PE9、PA2、PA3 原始电平，1=高；与 limit 对照检查接线/消抖
    bool control_enabled; // 参数、遥控在线、双中挡及键鼠输入源条件
    bool feedback_ok; // 三台在线、速度有效且 DM 已使能；FSM 还会校验角度/电流
    bool right_raw; // 遥控反馈的鼠标右键原始状态
    bool right_pressed; // 输入调度器消抖后的右键状态
    float target_velocity_rad_s[3]; // 各轴控制目标，包含位置保持外环输出，非 CAN 回读
    int16_t lift_command[2]; // 实际提交 setCAN 的 ID5/ID6 原始电流指令，非实测电流
    float dm_torque_command_nm; // MIT 编码前计算并限幅的阻尼力矩，非反馈力矩
};
extern volatile GantryWatch gantry_watch;

extern "C" {
#endif

void gantry_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // GANTRY_TASK_HPP
