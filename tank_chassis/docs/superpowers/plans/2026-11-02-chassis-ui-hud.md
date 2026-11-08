# Chassis UI HUD Implementation Plan

**Goal:** 保留现有功率显示，增加实时血量数字和血条、固定准星、可实射标定的距离刻度。

**Architecture:** 沿用现有 UI 图形与 UART DMA 接口。`UI_task.cpp` 负责在线判断和显示调度，`RMRefereeSystemUI.hpp/.cpp` 提供血量和固定瞄准图形。准星中心、尺寸及三条距离线的标签、相对坐标和长度集中在头文件。默认 3/5/7 m 及坐标仅为占位，未标定。全部 UI 发送共用 40 ms 最小包间隔，不改裁判协议或控制逻辑。

**Tech Stack:** STM32H723 / FreeRTOS / C++ / Python + g++ 宿主测试 / Keil ARMClang。

用户已授权直接实现；沿用当前工作区，保留已有未提交修改。无须新工作树，不提交或烧录。

- [x] 新增 `tests/chassis_ui_test.py`，编译真实 `UI_task.cpp`、UI 和裁判协议源码，验证血量/血条比例、零血量后恢复、未知血量上限、功率保留、固定图形仅初始化发送、失败重试、断线恢复、跨 tick 回绕的包间隔，以及修改坐标参数后的实际线坐标。
- [x] 执行 `python tests/chassis_ui_test.py`，确认原任务因缺少血量和准星而出现行为断言失败。
- [x] 在 `RMRefereeSystemUI.hpp` 添加固定瞄准参数和血量/准星更新及复位声明；在 `.cpp` 实现图形、发送节奏和初始化状态；`UI_task.cpp` 接入两个新增显示和断线复位。零血量删除填充线，恢复非零血量时重新添加。
- [x] 执行新增 HUD 测试和原 `tests/referee_protocol_test.py`，检查真实线上数据、CRC、原功率值与发送失败重试。
- [x] 补充 `docs/chassis_ui.md` 标定参数、初始布局和测试说明；Keil 构建，核对日志、AXF/HEX 与工程配置哈希。
- [x] 相对修改前快照检查本次差异并完成只读审查，修复实际问题后记录验证结果。

验证结果：新增真实 UI 任务宿主测试先确认缺少 HP/瞄准图形和统一包间隔的行为断言失败，接入后 6 组全部通过；原裁判协议 11 组回归全部通过。测试在临时副本中修改准星中心与第一条刻度偏移，重新编译验证发送坐标随参数变化。

Keil `MDK-ARM/chassis_ui_hud_build.log` 完成链接及 HEX 生成，0 Error(s)、5 个既有 RemoteData_t 警告；进程退出码 1 对应带警告构建。独立只读审查指出 UI 栈预算紧张，故仅将 `Core/Src/freertos.c` 的 UI_Task 从 256 words 增至 384 words，并再次构建通过。静态调用图深度约 1004 字节；实际高水位需上板确认。修正后审查无新增问题。

本次源码差异检查通过；相对修改前快照，底盘控制源码和三份 Keil 配置 SHA256 均未变化。保留原有未提交修改，未提交、未烧录、未进行客户端或实射验证。参数和标定方式见 `docs/chassis_ui.md`。
