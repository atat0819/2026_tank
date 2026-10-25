# 两工程鼠标 X 最新状态保持

用户最终确认：云台重复发送最新 DT7 鼠标 X，底盘覆盖保存并在每个控制周期使用；Pitch 鼠标 Y 继续一次性更新角度目标。该方案替代此前 X 单次增量、忙时累计、成功扣除的实现。

验收：不改 28 字节布局、两个 int16 的偏移、CRC、DMA 忙保护及回复门控；两种接收机接法使用同一个 `-input.mouse_x * 2*pi/180` 转向公式及 +/-4 rad/s 限幅。无新 DT7 帧时保持 X；新 DT7 零值覆盖旧值；发送忙/启动失败后重试最新状态，不排队旧鼠标值。保持失联/100ms 超时/切源首周期停车、LOCAL 优先、Z 停用及固定相对偏角 0。

- [x] 备份两工程本次涉及的源文件、测试及说明；保留当前工作区其他改动。
- [x] 更新云台 sender 回归和底盘 cache/turn 回归，先确认旧累计实现不满足最新状态合同。
- [x] 云台 `RtosTask/can_send_task.cpp` 删除 X 待发累计/分包/成功扣除，改发 `RemoteData.mouse_x`；更新 RS485 头部说明。保持 `ConsumeMouseDelta()` 及 Pitch Y 使用路径。
- [x] 底盘 `communication/boards_communication_uart_485.hpp/.cpp` 的字段改名为 `mouse_x`；`RtosTask/can_send_task.hpp/.cpp` 直接缓存到通信状态和公共输入快照，删除 ISR 累计及行驶任务取走清零，共用现有鼠标转向表达式。
- [x] 主机测试通过：同一采样连续发送、同值新帧覆盖而不相加、新零值、在途缓冲区不变、busy/error 后最新值胜出、Pitch Y 同帧只消费一次；真实两端编解码、LOCAL/GIMBAL 稳态同输入同输出、失联/过期/切源/gyro/STOP 门控、轮速方向、爬楼命令原子发布及 UART 错误恢复。
- [x] 只读审查未发现重要问题。
- [x] 核对两个工程 Keil 构建日志并确认 AXF/HEX 生成：云台 `MDK-ARM/mouse_state_hold_build.log` 为 0 Error(s), 16 Warning(s)；底盘同名日志为 0 Error(s), 5 Warning(s)。云台完整 Python 回归为 13 tests OK，底盘协议、缓存保持、两来源同值转向、轮速方向、爬楼命令发布和 UART 错误恢复测试通过。

没有改动 DT7 驱动、Pitch PID/FSM 参数、底盘轮速 PID/功率控制、后腿参数或输入优先规则。未提交、未刷写；该方案对齐采样保持方式，仍增加 RS485 转发时延，实际手感需要实机确认。
