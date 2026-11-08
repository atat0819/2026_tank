# Force Wheel Synchronization Implementation Plan

**Goal:** 用户已批准基于 e506456(save likong) 恢复力控，加入四轮仅 P 同步校正，统一限流/限功率，不恢复 75%/25% 分配。

**Architecture:** 使用未滤波轮端速度 FK -> IK 得到协调轮速，四轮 P 输出投影至 n=(1,-1,1,-1)。同步电流先按单一 q 限至 ±2 A，再与力控基础电流相加，整体统一限流和限功率。P 初试为四轮各 0.1 A/(rad/s)，无 I/D 状态；停机/失效/模式切换统一 Reset。保留 e506456 的参数、速度限制和 USB 协议，不修改其他数学库。

- [x] 新测试复现 6000/2000 分布与 4000/4000 车速反馈相同；同步必须降低前轮驱动、提高后轮驱动，且不改变名义 Fx/Fy/Mz。
- [x] 在 MecanumForceControl.hpp 增加同步 P 参数、目标/误差/电流观察量和同步计算，合流位置在统一限流之前；CAN 任务每周期先采样同步再执行车体力控。
- [x] 检查正反向、侧移/旋转/组合运动兼容轮速不产生同步输出、非均匀 P 的投影、同步峰值限制、统一限流/功率比例、复位及无新增积分残留。
- [x] 悬空轮组差异数值模型验证偏差下降；运行既有力控、方向、机构阶段、VOFA、输入/停车测试；Keil 编译及独立只读审查。

保留已验证备份，不提交、不刷写。仅 P 时不同负载可存在稳态差，实车需整定；不承诺严格零误差。

验证记录：控制器新接口测试先在旧头文件缺少同步接口时失败，补充后通过；实际任务同步调用断言先失败，接入任务后通过。不同初始轮速的简化悬空模型在 5 s 内差速降至初始值的 0.1% 以下，此模型无摩擦偏差，不代表实车不同阻力下零稳态差。

chassis_force_control_test.py、stair_wheel_power_test.py、chassis_wheel_direction_test.py、chassis_force_vofa_test.py、chassis_x_speed_limit_test.py、stair_command_publication_test.py、vofa_send_test.py、up_stair_integration_contract_test.ps1 均通过。独立只读审查通过，并独立复跑控制器及任务测试。源文件 diff 检查通过。

MDK-ARM/force_wheel_sync_build.log 记录链接及 HEX 生成完成，0 Error(s)、5 个现有 RemoteData_t 警告；进程退出码 1 对应带警告构建。恢复 e506456 后的 Keil 工程/选项/窗口配置哈希在本次实现和编译期间保持一致。tank_gimbal 未修改。

调试观察：chassis_force.sync_kp[4] 为四轮 P，sync_current_limit 为校正峰值；sync_target_speed[4]、sync_error[4] 单位均为轮端 rad/s；sync_current[4] 为限流前同步电流 A，最终有效同步电流还需乘 output_scale。原 motor_target_speed 保留摇杆目标对应轮速，不能当作同步目标。同步计算用未滤波轮速，当前仅 P，无新增 I/D 历史；Reset 清除目标、误差和同步电流。全部与原力控统一限流，最后由单个四轮功率控制器缩放。
