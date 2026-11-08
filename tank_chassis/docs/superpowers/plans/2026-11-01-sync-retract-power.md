# Sync Force Chassis Retract Power Plan

用户要求恢复收腿阶段前75%/后25%固定功率预算，不互借；沿用本聊天已确认的预算语义、键鼠退出门控和状态范围。

仅 `keyboard_cmd.valid && IMU && rear_retract_requested` 生效。有效命令发布后在同一临界区保存判断，前ID1/2、后ID3/4分别用二轮限幅器，组内同比缩放；其他阶段继续原四轮统一限幅器。按用户后续明确要求，分组期间关闭四轮同步校正，当周期清零同步目标、误差、电流；退出后重新计算同步。车体力控始终保留，两组最小 eta 反馈车体 PI；预算为最终约束，不强行放大电流。

- [x] 扩展真实任务片段测试：第二次B后前/后预测功率上限、保持、下次B/V/C恢复统一eta、键鼠失效/拨杆退出，先观察旧实现失败。
- [x] CAN任务增加比例常量、两个二轮限幅对象、临界区门控和分组分支。不修改同步控制头文件及用户当前4/10A等调参。
- [x] 验证前后零需求不借功率、组内电流比例、不同方向/预算/速度、同步+分组组合、STOP和反馈故障清零；运行相关回归。
- [x] Keil编译并读取新日志、独立只读审查、核对受保护参数及工程配置。

不提交、不刷写；模型预测预算不能等同实际功率严格3:1，零电流基础损耗超过预算时无法靠衰减解决。

验证：新测试先在旧统一分配的后25%预算断言处失败，分组后通过。覆盖126组电流/方向/速度/预算组合、真实同步输出进入分组、真实键盘FSM全部非双中档位、键盘掉线、V/B/B/保持/B/V/C和既有安全门。stair_wheel_power_test.py、chassis_force_control_test.py、stair_command_publication_test.py、vofa_send_test.py、up_stair_integration_contract_test.ps1 通过；独立只读审查通过并复跑分组测试。

三项旧回归未通过：chassis_wheel_direction_test.py、chassis_x_speed_limit_test.py 硬编码旧2.56/1.7/1.5速度限值，与用户当前3.75/2.5/2.5不一致；chassis_force_vofa_test.py 要求USB 15通道调用保持启用，而当前用户已注释该调用并改为UART RPM采集。保留用户当前配置，未改这些旧测试或开启USB发送。

Keil 日志 MDK-ARM/sync_retract_power_build.log 完成链接及HEX生成，0 Error(s)、5个既有RemoteData_t警告，进程码1对应带警告构建。源码diff检查通过。同步控制头文件SHA256保持E13CAC73564A796A8BE4380494F0752585DACF51C3ED3849D24574AADDC34F47，Keil工程/选项/窗口配置哈希与本次开始时一致。未修改tank_gimbal，未烧录或实车验证。

退出双中仅关闭本周期功率分组，保留机构IMU/收腿请求原行为；回到有效双中且仍有收腿请求时可恢复分组。

## 分组期间关闭同步的后续修正

- [x] 使用非零差速/同步电流进入收腿，旧代码在同步观察量应清零断言处失败。
- [x] 在实际任务同一阶段门控下清零三组同步数组，不改变同步P/上限/车体PI，不执行整个控制器Reset；关闭功率限幅的调试旁路也保持收腿期间同步关闭。
- [x] 验证保持20周期无残留，下一次B、V、C、拨杆离开双中、键盘失效后当周期恢复同步及原统一限功率；已有安全保护和126组预算测试仍通过。

本次修正验证：stair_wheel_power_test.py、chassis_force_control_test.py、stair_command_publication_test.py、vofa_send_test.py、up_stair_integration_contract_test.ps1 全部通过。独立只读审查通过并复跑任务测试。方向和限速两个片段测试增加普通模式门控变量以适配新条件，编译正常，仍在之前记录的旧速度数值断言处失败，未改动用户当前速度参数。

MDK-ARM/retract_sync_gate_build.log 记录链接及HEX生成完成，0 Error(s)、5个既有RemoteData_t警告；进程退出码1对应带警告构建。同步控制头文件及三份Keil配置哈希与本轮开始一致。未刷写。
