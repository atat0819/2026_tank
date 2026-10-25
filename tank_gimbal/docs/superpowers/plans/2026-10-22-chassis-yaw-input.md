# Chassis Yaw Input Implementation Plan

当前最终方案已切换为“鼠标 X 最新状态保持”，详见 `tank_gimbal/docs/superpowers/plans/2026-10-22-mouse-state-hold.md`。以下 X 累计/单次消费内容是此前阶段的历史记录，已被最新方案替代；Pitch Y 仍单次消费。

**Goal:** 删除云台 Yaw 电机，将原 RS485 偏角四字节替换为用户确认的两个 int16_t。

**Architecture:** 仅在现有任务中移除 Yaw 链，Pitch 单槽对应物理 ID8；摇杆发送最新值，鼠标发送一次性增量，复用现有 RS485 调度。仅修改云台，不提交、不刷写。

**Tech Stack:** STM32H723 / C++ / FreeRTOS / Keil / Python + g++ 主机测试。

- [x] 修改 `tests/boards_communication_uart_485_test.py`，验证两个有符号字段及输入门控；修改电机路由测试，验证 ID7 被忽略、ID8 进入唯一 Pitch 槽。先运行确认旧实现不满足。
- [x] 修改 `RtosTask/can_send_task.cpp/.hpp`：删去 Yaw 专属控制和恢复，Pitch 改为单槽；保留既有 Pitch 控制及龙门架路由。
- [x] 修改 `communication_between_boards/boards_communication_uart_485.cpp/.hpp` 的参数和打包，去除电机依赖；旧 CAN 偏角接口固定返回 0。
- [x] 将原双轴任务集成测试调整为单 Pitch 合同，运行 RS485、反馈切换、掉线恢复、Pitch 限位和龙门架测试。
- [x] Keil 完整编译并检查实际日志，审查增量差异，交付协议和底盘配套点。

验证记录：相关测试通过；与修改前工作区快照比较，Pitch PID/FSM 参数和完整控制/补偿计算未变。Keil 日志 `MDK-ARM/yaw_motor_removal_build.log`：0 Error(s), 16 Warning(s)，已生成 AXF/HEX。未提交、未刷写。

扩展检查：`tests/hi12_dma_receive_test.py` 因旧断言要求 UART7=115200 而失败，当前未改动的 `Core/Src/usart.c` 为 921600。未为本任务调整 IMU 配置或此旧断言。

用户确认的鼠标增量修正：

- [x] 补充实际 DT7 消费函数、实际任务发送片段与 RS485 打包的主机回归，复现同帧位移重复发送。
- [x] 仅在 `can_send_task.cpp` 增加待发累计量；忙/启动失败保留，HAL_OK 后扣除；保留 int16_t 分包余量，切模式/失联清零。
- [x] 更新协议注释和底盘配套点，不改 RS485 打包函数、帧长、等待回复规则及任务延时。
- [x] 运行相关回归并核对 Keil 新构建日志：鼠标增量/忙与错误保留/分包/发送时序/切模式与失联测试通过；Pitch 反馈切换、恢复、限位及龙门架输入回归通过。`MDK-ARM/mouse_delta_once_build.log` 为 0 Error(s), 16 Warning(s)。

用户确认的纯编码器 Pitch 修正：

- [x] 先用 `encoder_only_pitch_test.py` 复现活动 IMU 依赖；确认禁用视觉接管、停止姿态发送和 UART7 IMU 接收，保留 RS485 字段填 0。
- [x] `can_send_task.cpp/.hpp` 删除 IMU 来源选择、等待、PID 与视觉控制分支，直接使用逻辑槽 1 编码器角度/速度；保留编码器参数、限位、恢复与鼠标增量发送。
- [x] `remote_control_task.cpp/.hpp` 移除 IMU 实例及接收/恢复调用，停止视觉姿态发送；保留 DT7、RS485 与 USB 接收接口。
- [x] 更新原双来源测试为编码器合同，验证 UART7 无接收而 DT7/RS485 保留；补充协议保留字段与底盘配套说明。
- [x] 最终相关回归、参数差异检查和 Keil 构建日志核对：编码器反馈/PID、重力参考、恢复、限位、鼠标增量、RS485 接收和 UART7 停用测试通过；编码器 PID/FSM 参数、鼠标发送分支（除原 IMU 参数填 0）与 RS485 打包/时序保持一致。`MDK-ARM/pitch_encoder_only_build.log`：0 Error(s), 16 Warning(s)。只读复核未发现重要问题。未提交、未刷写。

旧验证记录中的“反馈切换”和 UART7=115200 断言仅对应此前阶段；当前已停用 IMU 接收，相应集成测试验证编码器固定反馈与 UART7 停用，不再要求 IMU 波特率。

- [x] 按用户要求合并反馈函数：`ControlTask()` 改为 `UpdatePitchMotorFeedback()`，移除单次 for 循环与 `UpdateGimbalFeedback()`；统一更新槽 0 快照及控制反馈，启动调用一次、每个控制周期调用一次。编码器、重力参考、限位和鼠标增量回归通过；`MDK-ARM/pitch_feedback_merge_build.log`：0 Error(s), 16 Warning(s)。

用户确认的 Pitch MIT 速度控制：

- [x] 先补实际任务控制片段测试，确认旧实现缺少角度 PID 的 rad/s 输出限幅及 MIT 速度控制。
- [x] 遥控与键鼠分别保留原角度 PID；max 直接设为 95°/s 换算的 rad/s，删除主控三个速度 PID，两个独立 MIT Kd 初值均为 1.3。
- [x] 正常发送 Kp=0、目标速度及匹配 Kd；保留重力前馈。按反馈估算合成力矩，原幅值保护改变估算值或方向限位生效时改为 Kd=0 的受限力矩命令；STOP/失联发送零 Kd、零前馈。复核发现下限估算为零/正时仍需关闭 Kd，已补先失败后通过的回归；上限允许正力矩支撑的原配置保留。FSM 与发送保护统一使用本周期编码器 rad 快照。
- [x] 验证角度/速度模式、独立参数、速度限幅、STOP/恢复、合成力矩限位、RS485/鼠标及完整 Keil 编译。VOFA 第六通道注明为估算力矩，非实测力矩。

最终验证：`python -m unittest discover -s tests -p '*test.py'` 为 13 tests OK，另执行了模块内 RS485/鼠标、龙门架、UART 路由等回归；发送累计量片段与修改前快照一致。实际任务控制片段测试还单独把键鼠 Kd 改为 0.9 重编，验证两个 Kd 的独立路由。只读复核未发现剩余重要问题。最终日志 `MDK-ARM/pitch_mit_velocity_clean_reports_build.log` 为 0 Error(s), 16 Warning(s)，已生成 AXF/HEX。期间两次链接因旧 map/htm 文件打开失败；仅将这两个生成报告移到临时快照备份后重建成功，未修改工程配置。未提交、未刷写或带载验证。

用户后续纠正 J4340 参数：位置 +/-3.14 rad、速度 +/-10 rad/s、力矩 +/-27 N*m。当前 `DmMotor.hpp` 已有用户给定值，保持文件逐字节不变；仅同步任务内 `J4340_TORQUE_LIMIT_NM` 为 27 并更正相关注释。95°/s 目标速度和两个独立 Kd=1.3 保持不变。`pitch_mit_velocity_test.py` 先复现旧 9 N*m 限幅错误，再验证 +/-15 N*m 不触发幅值回退、合成超过 +/-27 时受限；`gantry_motor_definition_test.py` 验证真实 J4340 的 TX/RX 位置、速度、力矩两端映射，四组龙门架用例通过。`MDK-ARM/pitch_27nm_verified_build.log`：0 Error(s), 16 Warning(s)，生成 AXF/HEX；同样在备份旧生成报告后重建成功。未提交、未刷写。
