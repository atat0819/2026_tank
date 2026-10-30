# Stair wheel power implementation plan

**Goal:** IMU 模式第一次 B 至第二次 B 收腿之前前轮零电流；初次进入 IMU、收腿及保持阶段四驱；其他模式维持原控制。

**Architecture:** 在 CAN 任务已有命令发布临界区中读取 `mode == IMU && front_target_requested`，在四轮 PID 后、功率限幅前清除前轮 ID 1/2 的电流和 PID 历史。第一次 B 将前机构目标请求置 true，第二次 B 收腿时置 false，直接复用该阶段状态。后续 B 交替时也按同一阶段切换；V 重新进入初始阶段，恢复四驱。保留四轮速度反馈、功率模型基础损耗、总功率限制、后轮 PID 和停车链。

**Tech Stack:** STM32H723 / FreeRTOS / C++ / Python host tests / Keil.

- [x] 新增 `tests/stair_wheel_power_test.py`，提取实际命令发布、PID 和限功率代码；先运行并确认调平阶段前轮零电流断言失败。
- [x] 修改 `RtosTask/can_send_task.cpp`，在发布临界区保存上述模式判断，在功率分配前对前轮执行 `motor_pid[i].reset(); motor_output[i] = 0.0f;`。
- [x] 验证 NORMAL、V、第一次 B、第二次 B 收腿、保持、第三次 B 返回调平、C、停车；验证前轮无积分残留、带转时无主动制动、后轮预算增加和总功率受限。
- [x] 运行轮速方向、命令发布、VOFA、上台阶集成回归，Keil 编译后读取错误/警告与链接结果，检查最终 diff。

在当前用户指定工程内修改，保留已有未提交改动；不提交、不刷写。硬件效果需后续实车验证。

验证结果：新增测试经历预期失败后通过，覆盖实际前进/后退电机符号、功率限幅启用与旁路、前轮被动带转、退出调平无积分残留。`stair_command_publication_test.py`、`vofa_send_test.py`、`up_stair_integration_contract_test.ps1` 通过。

`chassis_wheel_direction_test.py` 原脚本失败于既有鼠标倍率 2 的断言，HEAD 与当前源码倍率均为 1；未修改该无关参数和测试。临时排除鼠标断言、单独执行原轮速方向与运动跟踪部分通过。`power_strategy_startup_test.cpp` 仍写死历史默认 50 W，与用户当前 100 W 不符，本次未运行或改动它。

Keil 使用 `D:/Keil5/Core/UV4/UV4.exe -b` 编译，`MDK-ARM/stair_wheel_power_build.log` 记录链接和 HEX 生成成功，0 Error(s), 5 Warning(s)。本次源文件 diff 检查通过；全仓 diff 检查另有原有 JLink 日志及构建产物尾随空格，不属于本次业务代码修改。

后续阶段范围修正：仅 `IMU && front_target_requested` 时后驱；V 初次进入和再次进入均四驱，B 抬前机构后后驱，下次 B 收腿即恢复四驱。更新测试先在旧条件下失败，修改后通过；命令发布和上台阶集成合同通过。`MDK-ARM/stair_first_b_rear_drive_build.log` 记录 0 Error(s), 5 Warning(s)，HEX 已生成，未刷写。

这次额外执行采集回归，USB/UART10/四轮采样部分通过，但配置检查仍要求115200，而当前 `Core/Src/usart.c` 的UART10为230400、`.ioc`为115200，最终配置断言失败。本次未改该串口配置或相关测试。
