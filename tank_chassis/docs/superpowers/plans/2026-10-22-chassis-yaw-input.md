# 底盘接入云台转向输入

当前最终方案已切换为“鼠标 X 最新状态保持”，详见 `tank_gimbal/docs/superpowers/plans/2026-10-22-mouse-state-hold.md`。以下 X 累计/单次消费内容是此前阶段的历史记录，已被最新方案替代；Pitch Y 仍单次消费。

用户授权工程：`C:/Users/hp/Desktop/2026_tank/tank_chassis`。使用当前 checkout；保留后腿参数等已有改动，不提交、不刷写。

验收：28 字节协议和 CRC/回复不变；2..3 为 int16 摇杆 X，4..5 为 int16 鼠标 X 增量。遥控按最新摇杆连续转向；鼠标由行驶任务一次消费，爬楼读取公共快照不能消费。固定偏角 0，IMU 保留字段填 0；禁用 Z 跟随及相对旋转相位补偿。复用原鼠标角速度公式 -count*2*pi/180 与 +/-4 rad/s 限幅，不新增 Yaw 角度闭环。云台累计增量不同于本地 DBUS 保持采样，初始系数相同不保证手感相同，需实车调节。

- [x] 先补 `tests/gimbal_yaw_input_test.py` 验证旧工程缺少摇杆/单次消费；更新实际两端协议回归。
- [x] 修改 `communication/boards_communication_uart_485.hpp/.cpp` 的字段解码、摇杆范围和明确失联帧校验。
- [x] 修改 `RtosTask/can_send_task.hpp/.cpp`：增加摇杆存储和 int32 待消费量，ISR 每新包累加，失联/过期/切模式/本地接管清除。公共快照只读，主任务在临界区消费并清零；摇杆/鼠标转向接入 GIMBAL 来源。
- [x] 修改 `fsm/chassis_keyboard_fsm.hpp/.cpp`：Z 无动作，兼容保留输出字段恒 false；CTRL 陀螺、V/C/B 和平移不变。主任务移除相对旋转相位补偿。
- [x] 运行协议、增量/来源/停车、键盘、轮速方向、爬楼命令发布回归；Keil 编译并读取真实日志；只读代码复核。

验证记录：新增转向回归先以缺少摇杆字段失败，修改后通过。实际两端 RS485 请求/回复、拆粘包、CRC、范围与失联帧校验、DMA 忙/错误保持回归通过。鼠标真实发布/快照/单次消费/转向分支覆盖正负累加、超过 int16、相同连续新包、零增量、无新数据归零、LOCAL 优先、切源/切模式/100ms 过期清除、过期后新包先于控制任务、tick 回绕和小陀螺丢弃；Z 无动作，CTRL 陀螺及既有 V/C/B 测试通过。轮速方向、键盘爬楼命令发布、前后 FSM 命令流程、来源选择和停车策略回归通过。

主机命令：`python tests/gimbal_yaw_input_test.py`、`python tests/rs485_board_link_test.py`、`python tests/chassis_wheel_direction_test.py`、`python tests/stair_command_publication_test.py`、`python tests/rs485_error_recovery_test.py`。C++ 测试包含 `chassis_keyboard_fsm_test`、`control_input_selector_test`、`stair_mode_policy_test`、`stair_command_test`、`stair_command_flow_test`；后者链接键盘 FSM、前后机构 FSM、alg_fsm.cpp 与 `user/core/Alg/UtilityFunction/SlopPlanning.cpp`。

Keil：`MDK-ARM/chassis_yaw_input_build.log` 为 0 Error(s), 5 Warning(s)，生成 AXF/HEX。只读审查未发现重要问题；后续仅修正相关注释，已对比去注释后的 C++ token 相同。原后腿 ROLL_KP=2、ROLL_MAX_TORQUE_NM=6 等已有改动保留；未修改 up_stair 任务、功率控制、轮速 PID、UART 错误恢复或云台工程。未提交、未刷写；方向、灵敏度及负载效果尚需实机验证。
