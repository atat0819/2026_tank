# UART10 IMU VOFA 调试发送

**Goal:** 通过 UART10 的现有 JustFloat DMA 接口发送用户选择的六个 IMU 通道。

**Architecture:** 在 `up_stair.cpp` 现有有效 IMU 新帧分支恢复 `vofa_send()` 调用。
通道按 Pitch 原始/滤波、Pitch 角速度原始/滤波、Roll 原始/滤波排列。
沿用后腿 IMU 模式门控、滤波历史和超时保护；UART10 保持 115200、8N1。
保留功率计 RX 与 USB 功率遥测；本任务不调整滤波系数或电机控制。

**Tech Stack:** STM32H723、C++、HAL UART DMA、Python/g++ 主机测试、Keil。

## 实施与验收

- [x] 修改 `tests/rear_imu_filter_test.py`：首帧和新帧发送六通道；重复、无效、超时及关闭后腿模式时不发送；检查 Roll 原始/滤波顺序。
- [x] 调整 `tests/power_identification_test.py` 的协议隔离断言：允许 UART10 IMU 发送，禁止 IMU 混入 USB 功率流。
- [x] 运行 IMU 测试，确认因未恢复发送而失败（`vofa_frames == 1`）。
- [x] 在 `RtosTask/up_stair.cpp` 原遥测位置添加 `if (imu_new_frame)` 和六通道 `vofa_send()`。
- [x] 运行 `python tests/rear_imu_filter_test.py`、`python tests/vofa_send_test.py`、`python tests/power_identification_test.py`，全部通过。
- [x] Keil 编译，实际链接结果为 0 errors、5 warnings，详见 `MDK-ARM/uart10_imu_vofa_build.log`。
- [x] 检查本次差异及 `git diff --check`；UART DMA 缓冲区位于 AXI SRAM（`0x240017f8`），仅有一处 UART10 IMU 发送调用；未提交或烧录。

## 使用

PE3（UART10 TX）接 USB 转串口 RX，共地。VOFA 选择串口、115200、8N1、JustFloat。
I0/I1 为 Pitch（deg），I2/I3 为 Pitch 角速度（deg/s），I4/I5 为 Roll（deg）。
当前发送沿用后腿 IMU 模式门控，双下/NORMAL 不发送；先按现有操作进入后腿 IMU 模式。
28 字节帧在 115200、8N1 下约需 2.43 ms；UART 忙时跳过本帧，不改写在发缓冲区。
主机测试与编译不能证明板端已烧录或电脑已收到数据。
