# Chassis Force Control Implementation Plan

> **For agentic workers:** Execute inline with superpowers:executing-plans and test-driven-development. Use requesting-code-review for the control math and final diff.

**Goal:** 将底盘统一为四轮车体速度闭环力控，移除上台阶仅后轮驱动策略。

**当前参数要求（同日后续更新）：** 用户要求 PID 参数全部为 0，不再使用旧参数换算。已删除 `WHEEL_PID_TO_FORCE`、`OLD_INTEGRAL_FORCE_LIMIT`，三轴 `kp`、`ki`、`integral_limit` 全部显式为 0。当前类为 PI，没有 D 项；使用积分前需要自行设置积分限幅。动态测试显式使用独立测试增益，固件默认不输出驱动电流。

本次参数更新的零默认断言先在旧非零配置上失败，修改后控制器/方向/实际任务三项回归通过。`MDK-ARM/chassis_force_zero_pid_build.log` 记录链接和 HEX 生成完成，0 Error(s), 5 Warning(s)；未提交、未刷写。

**Architecture:** 使用当前车专用的一个头文件统一正解、力分配与三轴 PI，保留输入/FSM/功率/CAN 链。标准麦轮内部单位配合入口旧倍率，保留既有操作速度；电流和功率缩放均反馈到积分处理。

**Tech Stack:** STM32H723 / FreeRTOS / header-only C++17 / g++ host tests / Python / Keil ARMClang.

- [x] 添加 `tests/chassis_force_control_test.cpp` 及其 Python 编译入口，随后更新 `tests/chassis_wheel_direction_test.py`：先因缺少车体力控实现失败；覆盖正反解、力重构、方向、增益、限流、抗饱和及带滤波的速度跟踪。
- [x] 实现 `user/core/Alg/ChassisCalculation/MecanumForceControl.hpp`，使用设计文档给出的同一 B 矩阵。运行上述主机测试直到通过。
- [x] 更新 `tests/stair_wheel_power_test.py` 为所有机构阶段四驱的实际任务片段测试；先在旧任务上观察失败。
- [x] 修改 `RtosTask/can_send_task.cpp`：实例化车体控制器，删除后驱判断和前轮清零；在各旧复位点重置新控制器；四轮在线/有限反馈作为使能条件；正解 -> 原滤波/斜坡 -> 车体 PI -> 原功率限幅，并回传 eta；轮目标仅作观测。
- [x] 运行四轮/任务测试、命令发布、机构集成、输入心跳、VOFA 和功率策略测试；测试从当前配置取上限，不写死历史值。记录与本次无关的既有失败。
- [x] `D:/Keil5/Core/UV4/UV4.exe -b MDK-ARM/tank_chassis.uvprojx -j0 -o MDK-ARM/chassis_force_control_build.log`；读取新日志的编译/链接结果。审查源码 diff，完成独立代码审查并处理重要问题。

不新建分支/提交，不刷写；用户已批准在当前工程实施上述范围。

验证记录：新力控测试先因实现缺失失败；更新的机构阶段测试先因旧后驱条件仍存在失败，完成实现后均通过。方向/倍率、正反解、力和功率一致性、限流/限功率抗积分饱和、带 8 Hz 滤波的速度保持/减速、实际任务 V/B/C 四驱及全部复位分支、原子命令发布、VOFA、机构集成、心跳、输入选择和机构模式/命令主机测试通过。受控制结构替换影响的旧方向测试改为调用实际新任务片段，鼠标倍率断言对齐当前源码的 1（未修改生产鼠标输入）。VOFA 测试仅更新新增轮反馈故障门限的定位条件。

独立只读审查确认几何/力分配、L² 增益、两级限幅和各复位路径；修正测试比较函数对 NaN 漏检，并记录三轴积分叠加约束及悬空/打滑限制。未改原通用 Omni 数学库，其旧 FK/ID 不再用于当前控制链。

Keil 新日志 `MDK-ARM/chassis_force_control_build.log`：链接、HEX 生成完成，0 Error(s), 5 Warning(s)；进程返回 1 对应带警告构建，已核对实际链接结果。AXF/HEX 和相应构建产物已更新。警告来自原有 RemoteData_t 默认成员初始化与 C 链接兼容性，本次未改该声明。

只验证本次相关检查，未声称全仓历史测试均通过；原 `gimbal_yaw_input_test.py` 仍有写死历史倍率 2 的断言，`power_strategy_startup_test.cpp` 仍有历史 50 W 断言，本次未执行或修改这两个无关旧配置测试。全部改动保留在当前工作区，未提交、未刷写。

