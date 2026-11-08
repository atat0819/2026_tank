# Keyboard Start Ramp Implementation Plan

**Goal:** 仅在有效键鼠模式下为底盘 vx/vy 增加启动斜坡；松键目标立即归零，遥控器直接使用目标，旋转保持原响应。

**Architecture:** 复用 SlopePlanning，按 1 ms 周期设置 0.008 m/s 的加速步长（约 8 m/s²），下降步长保留 100 m/s 使减速目标直接到位。每次反向先清零该轴规划，再向新方向加速。规划反馈使用上次规划输出，避免实测速度跳过斜坡；闭环仍使用原实测反馈。退出键鼠、STOP、输入掉线/切源或轮速失效时清空规划，重新进入从零启动。

**Tech Stack:** STM32H723 / FreeRTOS / C++17 / Python + g++ 宿主测试 / Keil ARMClang。

用户已确认：只限制启动加速，松键立即把目标置零。沿用当前工作区，不提交已有未提交修改，不烧录。

- [x] 新增 tests/chassis_keyboard_ramp_test.py：提取实际任务目标及安全分支，编译真实 SlopePlanning；验证正负方向每周期 0.008、到达目标不超调、反馈不跳过斜坡、减速/松键立即到位、反向重新启动、遥控器直通、旋转不变及安全复位。
- [x] 执行 `python tests/chassis_keyboard_ramp_test.py`，确认原实现因键鼠首周期直接满速而失败。
- [x] 修改 RtosTask/can_send_task.cpp：`KEYBOARD_SPEED_RAMP_STEP = 0.008f`；仅 `keyboard_cmd.valid` 分支使用 ramp；反向先调用 `TIM_Calculate_PeriodElapsedCallback(0.0f, 0.0f)`；规划调用的 feedback 为 `GetOut()`；非键鼠及安全分支清零规划；force_target 使用分支选择后的 vx_body/vy_body。
- [x] 更新两个直接相关旧测试夹具：速度限幅用当前源码常量作期望，轮向测试显式选择遥控器直通；不修改限速、控制增益、功率限制或遥测。
- [x] 执行新增斜坡测试及速度限幅、轮向、键鼠/鼠标、机构功率、力控回归；Keil 构建并核对日志，确认项目配置未变。
- [x] 审查相对本次修改前快照的差异，确认只作用于键鼠启动；记录验证结果。

验证结果：新增斜坡测试先出现 `actual=-3.75 expected=-0.008` 的预期行为失败；接入后通过。chassis_keyboard_ramp_test.py、chassis_x_speed_limit_test.py、chassis_wheel_direction_test.py、stair_wheel_power_test.py、rs485_board_link_test.py、chassis_force_control_test.py、stair_command_publication_test.py 及真实 chassis_keyboard_fsm_test.cpp 共 8 项通过。键盘 FSM 宿主测试保留既有聚合初始化字段缺省警告。

Keil 日志 MDK-ARM/keyboard_start_ramp_build.log：完成链接及 HEX 生成，0 Error(s)、5 Warning(s)；警告均为原有 RemoteData_t C 链接兼容性问题，进程退出码 1 对应带警告构建。uvprojx/uvoptx/uvguix 文件在构建前后 SHA-256 一致。独立只读审查未发现可操作问题；本次差异检查通过。

已有测试问题：gimbal_yaw_input_test.py 的夹具缺少 MAX_WZ_RAD_S 声明，修改前快照和修改后源码均复现同一编译失败；本次未修改该测试。鼠标来源、限幅和旋转直通由限速/轮向测试及 RS485 测试覆盖。

未烧录或进行实车验证；0.47/0.31 秒仅为 1 ms 周期下 X/Y 静止至满速的理论规划时间。保留原有键盘 20 ms 去抖，立即归零指稳定松键命令生效的控制周期。
