# Chassis Nine-Switch Rotation Implementation Plan

> **For agentic workers:** 在当前 checkout 执行；保留已有未提交改动，不提交、不烧录。

**Goal:** 九种拨杆组合直接选停车/键鼠/遥控，删除双层状态与小陀螺。

**Architecture:** 一个 current_mode、一个 StateUpdate、一个 Get_wz_cmd。双下停车、双中键鼠（无效则停车）、其余七种遥控；原跟随 PID 留存但不激活。

**Tech Stack:** C++17 / Class_FSM / Python + g++ 主机回归 / Keil ARMCLANG。

- [x] 更新 tests/chassis_rotation_fsm_test.py，先确认旧双层状态触发失败；覆盖全部九种组合、非法档位/失联/键鼠无效、遥控模式间不切状态、全部鼠标取值及保留 PID。
- [x] 精简 fsm/chassis_fsm.hpp/.cpp：单一状态，九种组合逐项列出；删除小陀螺和双层状态/重载。任务改为四参数 StateUpdate，读取统一角速度输出。
- [x] 更新真实键鼠链路测试，先确认 gyro_enabled 残留导致失败，再删除键盘自旋逻辑；验证 CTRL 不影响鼠标输出。调整引用已删除状态/字段的相关测试。
- [x] 运行九组合、输入缓存、限幅、轮向、键盘独立测试、键盘斜坡、爬楼和 RS485 回归。
- [x] 独立只读审查；Keil 重新构建并核对新日志与配置哈希。
- [x] 本次源码 git diff --check；记录测试及未烧录状态。

## 验证结果

2026-11-02：八项主机回归通过：chassis_rotation_fsm_test.py、gimbal_yaw_input_test.py、chassis_x_speed_limit_test.py、chassis_wheel_direction_test.py、chassis_keyboard_ramp_test.py、stair_wheel_power_test.py、rs485_board_link_test.py，以及 g++ 编译运行的 chassis_keyboard_fsm_test.cpp。

新增验收先检测到旧 rotation_mode 和键盘 gyro_enabled 残留而失败；移除后通过。真实键盘 FSM 与输入发布/缓存链路验证 CTRL 按下、保持、释放及二次按下都不覆盖鼠标输出。

独立只读审查无可操作问题。Keil MDK-ARM/chassis_nine_switch_build.log 完成链接和 HEX 生成，0 Error(s)、5 个原有 RemoteData_t 警告（UV4 进程码 1 为带警告构建）。三份 Keil 配置、MecanumForceControl.hpp 和 freertos.c 构建前后 SHA-256 一致。本次修改源码差异检查通过。

保留已有工作区改动；未提交、未烧录、未进行硬件验证。
