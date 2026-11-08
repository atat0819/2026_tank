# 底盘九种拨杆组合与旋转输出

用户已确认的最终要求：九种拨杆组合直接判断；双下停车、双中键鼠、其余七种遥控。取消小陀螺状态及 CTRL 自旋功能。本方案替代此前的双层模式方案。

## 实现约束

- 只保存一个 current_mode，移除 rotation_mode、Get_Rotation_Mode 和状态更新/旋转输出的重载。
- StateUpdate(s1,s2,online,keyboard_valid) 逐项列出九种组合。双下、失联、非法拨杆及双中键鼠无效进入 STOP；双中有效进入 MOUSE_ROTATE；其余七种进入 JOYSTICK_ROTATE。
- Get_wz_cmd(yaw,stick,mouse,max_wz) 只按 current_mode 输出：遥控为 -stick*max_wz，鼠标为 -count*(1.0f*M_PI/180.0f)，停车为 0。任务只传入参数、读取结果并保留车体目标统一 ±2.5 rad/s 限幅。
- 删除 CHASSIS_GYRO_SPIN、延时变量、陀螺速度参数和键盘 gyro_enabled/CTRL 翻转逻辑。KEY_CTRL 按键码保留，当前不产生动作。
- 保留 FOLLOW_GIMBAL、NOT_FOLLOW 和既有跟随 PID，当前九种组合均不进入这两个保留状态；Z 仍无动作。
- 积分复位跟随实际模式变化：遥控七种档位间切换不复位，遥控/键鼠/停车之间切换复位。不再保存旧的虚拟模式来决定积分复位。
- 输入仲裁、缓存/过期、切源与轮反馈安全门控、平移、V/C/B 机构功能、力控参数及功率策略不变。

## 验收

九种有效组合、非法档位/失联、双中键鼠失效、正负摇杆、全部 int16 鼠标输入、CTRL 无旋转动作均有回归。相关输入/限幅/轮向/键盘/爬楼/RS485 回归及 Keil 编译通过。不烧录、不提交已有工作区改动。
