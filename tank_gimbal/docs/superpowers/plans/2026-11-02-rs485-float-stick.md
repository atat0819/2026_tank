# RS485 Float Right Stick Implementation Plan

**Goal:** 仅将现有右摇杆 X 改为归一化 float，移除云台 IMU 数据，双端同步为 22 字节协议。

**Architecture:** 保留 USART3 DMA 请求/回复、CRC16、鼠标最新状态、输入源选择及失联保护。云台使用现有 `get_right_x()`，底盘直接使用收到的归一化值，不再除以 660。

**Tech Stack:** STM32H723 / C++ / FreeRTOS / Keil / Python + g++。

## 已确认协议与范围

- 用户确认只发送右摇杆 X，不增加 Y。
- 0..1=78 91；2..5=右摇杆 X float，[-1,1]；6..7=鼠标 X int16；8..11=Vx float；12..15=Vy float；16=S1；17=S2；18..19=键盘 uint16；20..21=CRC16。
- 多字节字段小端，CRC16/MCRF4XX 覆盖 0..19。删除原两项 IMU 参数、打包、解码和通信缓存，不修改底盘自身 IMU。
- 保持摇杆方向、归一化量、模式清零、失联全零、DMA 忙保护、2 ms 回复超时和 12 字节裁判回复。
- 双端需配套更新固件；不兼容旧 28 字节请求。本次不提交、不刷写。

## 实施与验收

- [x] 更新 `tank_gimbal/tests/boards_communication_uart_485_test.py`：使用 7 参数发送接口，校验 22 字节布局、-0.5f 的浮点字节序、鼠标正负极值、CRC、模式清零、失联及 DMA/超时行为。
- [x] 更新 `tank_chassis/tests/rs485_board_link_test.py`：真实双端往返验证 22 字节拆帧、粘帧、噪声、CRC、归一化转向、NaN/Inf/超范围摇杆拒绝、失联和本地输入优先。先运行两项新测试，确认旧实现不满足新接口。
- [x] 修改云台 `communication_between_boards/boards_communication_uart_485.hpp/.cpp`：签名首参数为 float，删除两个 IMU 参数，按上述偏移打包 22 字节。修改 `RtosTask/can_send_task.cpp`：调用 `get_right_x()`，移除两项 IMU 实参，保留模式及失联门控。
- [x] 修改底盘 `communication/boards_communication_uart_485.hpp/.cpp`：接收 22 字节，删除 IMU 字段，解码 float 摇杆并校验有限且位于 [-1,1]。修改 `RtosTask/can_send_task.hpp/.cpp`：通信缓存改为 float、删除两项 IMU 缓存和赋值，`input.wz = gimbalChassis_communicate.right_stick_x`。
- [x] 更新底盘 `communication/README_rs485.md`，记录新协议。调整直接受影响的 `tests/gimbal_yaw_input_test.py` 的 float 输入，并从当前源码提取转向常量，修复基线已存在的缺少 `MAX_WZ_RAD_S` 编译依赖；不修改控制参数。
- [x] 运行两端通信、UART 恢复、编码器 Pitch 回归；Keil 分别编译两个工程，检查新日志的错误数和 AXF/HEX 时间戳。
- [x] 与本轮修改前快照逐文件比较，确认底盘现有未提交控制改动全部保留。

## 基线

- 原 28 字节云台发送测试、双端往返测试通过。
- 原 `gimbal_yaw_input_test.py` 缺少当前 `MAX_WZ_RAD_S` 常量而编译失败；还保留旧鼠标增益/限幅期望，测试须随当前实际源码同步。

## 验证结果

- 先失败：两端新通信测试均因旧发送接口仍要求 9 参数、旧任务仍读取计数接口而失败。
- 修改后通过：云台 `boards_communication_uart_485_test.py`、`rs485_uart_test.py`、`encoder_only_pitch_test.py`、`pitch_mit_velocity_test.py`；底盘 `rs485_board_link_test.py`、`gimbal_yaw_input_test.py`、`rs485_error_recovery_test.py`。
- 云台 `MDK-ARM/rs485_float_stick_build.log`：0 Error(s), 16 Warning(s)，AXF/HEX 更新于 2026-11-02 21:11:15。
- 底盘 `MDK-ARM/rs485_float_stick_build.log`：0 Error(s), 5 Warning(s)，AXF/HEX 更新于 2026-11-02 21:11:46。
- 两端 Keil 进程返回 1，日志均确认链接及 HEX 生成成功；按实际日志与产物判断通过。
- 与本轮临时快照比对：底盘 `StairCommand stair_command;` 之后及云台发送调用之后的控制源码全文一致；原文件换行格式和 BOM 状态保留。
- 未提交、未刷写，实板收发和带载行为未验证。
