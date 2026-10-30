# 四轮 3508 功率辨识采集

**Goal:** 外接功率计只测四个 3508 的供电总功率，经 UART10 接收；用 USB CDC / VOFA JustFloat 采集功率、四轮电流和转速。

**Architecture:** 复用历史 12 字节三个 float 协议和现有 DMA+Idle 回调。有效功率帧更新数据、接收时刻、序号；CAN 任务在临界区复制该帧和四轮反馈，在临界区外用 `PowerControlBase<4>` 计算预测功率。每个新功率帧最多发送一次，不按任务周期重复旧测量。USB 忙/未连接直接跳过。保留现有遥控和功率限制。

**Tech Stack:** STM32H723, FreeRTOS, UART10 RX DMA, USB CDC, C++, Python host tests, Keil.

- [x] 先补测试验证 USB 静态缓冲区、未枚举、忙时不覆盖、15 通道和 JustFloat 帧尾。
- [x] UART10 设置 115200 / 8N1（同步 `.ioc`），恢复 12 字节接收，验证完整帧、无效长度/非有限数、HT、错误恢复和接收先解析后重启。
- [x] CAN 任务在公共循环发送 15 通道，覆盖 STOP 仍采集、只发送新测量、四轮带符号电流与转子 rad/s、反馈状态和模型求和。
- [x] 关闭 up_stair 中原 IMU 遥测调用，避免 UART10 向功率计发送无关帧；保留滤波及控制，更新对应测试。
- [x] 主机回归、Keil 编译、独立代码审查，记录通道表与接线/拟合说明；不刷写、不提交。

用户已确认 UART10 可接、功率计只测四个 3508。使用历史协议；硬件协议和数据质量仍需接线采样验证。没有实测记录前不生成新拟合系数。

验证：`power_identification_test.py` 先因 USB 缺少持久缓冲区按预期失败，修改后全部通过；`hi14_uart7_test.py`、`rear_imu_filter_test.py`、`rs485_error_recovery_test.py`、`stair_wheel_power_test.py`、`stair_command_publication_test.py`、`vofa_send_test.py` 和上台阶集成合同通过。RS485 回归调整为只提取目标 UART 分支，避免新 UART10 分支要求无关 stub。

Keil 最终链接并生成 HEX，0 Error(s), 5 Warning(s)。独立只读审查确认 USB 64 字节包的 ZLP/TxState、优先级 5 临界区、UART 普通 DMA TC/Idle 行为及单位转换；未发现必须修改的问题。接线、15 通道和总功率拟合公式见 `docs/power_identification_usb.md`。
