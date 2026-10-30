# 全向轮、舵轮解算约定

`OmniCalculation.hpp` 用于固定驱动方向的全向轮，`StringWheel.hpp` 用于独立转向/驱动的舵轮。当前工程麦轮力控使用独立的 `MecanumForceControl.hpp`，不调用这两套解算。

## 单位与方向

| 量 | 单位/约定 |
|---|---|
| 轮半径、轮心坐标、中心到轮距离 | m |
| 车体 X/Y 速度 | m/s，右手车体坐标系 |
| 车体旋转角速度 | rad/s，逆时针为正 |
| 所有角度 | rad，逆时针为正 |
| 轮子输入/输出转速 | **轮端 rad/s**，不含减速比 |
| 车体 Fx/Fy、Mz | N、N*m |
| `GetMotorTorque()` | **轮端 N*m**，不含减速比和电流换算 |

电机反馈若为转子 RPM，应由调用层先计算：

```text
轮端 rad/s = 转子 RPM × 2π / 60 / 总减速比
转子目标 RPM = 轮端目标 rad/s × 总减速比 × 60 / 2π
```

轮端力矩除以实车标定的轮端力矩/电流系数，才是目标电流。安装方向的正负号必须同时用于反馈和输出。旧 `String_IK` 的固定 `40×60/(2π)` 已移除，旧调用方不能继续把输出直接与电机 RPM 比较。

## 全向轮几何

- `Omni_IK(r, s, wheel_azimuth, wheel_coordinate)`：`r` 为几何尺度，`s` 为轮半径；坐标数组为四轮车体坐标 `(x_i,y_i)`。
- `Omni_FK(r, s, n, wheel_azimuth, wheel_direction)`：旧接口按各轮心到中心相同距离 `r` 建模；`wheel_direction[i]` 是轮心位置极角，不是电机正反转系数。
- `Omni_ID(r, s, n, wheel_azimuth, wheel_coordinate)`：按实际坐标分配轮端力矩。
- `wheel_azimuth[i]` 均指轮子正转驱动方向角，不是麦轮小辊子角度；`n` 为 3 或 4，使用数组前 `n` 个轮。IK 返回四个位置的结果，三轮调用者忽略第四个。
- 正逆解使用相同的右手 X/Y 符号。以轮心极角 `phi_i` 构造坐标时，坐标必须是 `r*[cos(phi_i),sin(phi_i)]`。

设 `a_i` 为驱动方向，归一化几何行向量为：

```text
h_i = [cos(a_i), sin(a_i), (x_i*sin(a_i)-y_i*cos(a_i))/r]
u = [vx, vy, r*wz]
轮端转速_i = h_i*u / s
```

正解用几何矩阵的最小二乘逆，不能将各轮投影直接平均。逆动力学求最小范数轮缘力，使其合成所请求的力/转矩；再乘轮半径得到轮端力矩。

## 舵轮角度与零位

三套类都使用同一组每轮零位 `phase_offset[4]`：

```text
物理轮朝向 = 原始编码器角度 - phase_offset[i]
舵向编码器目标 = 物理目标朝向 + phase_offset[i]，再就近转位
```

- `String_IK(r,s,wheel_azimuth,phase_offset)`。
- `String_FK(r,s,wheel_azimuth,phase_offset)` 和 `String_ID(...)` 可省略第四参数，省略表示零位全 0。
- `wheel_azimuth[i]` 在舵轮类中是轮心位置极角，各轮心到车体原点距离均为 `r`。
- 三者的 `Set_current_steer_angles(angle,i)` 都接收原始编码器 rad；不能只在 FK/ID 额外减一次偏移。
- `GetMotor_direction(i)` 返回最接近当前编码器角度的目标 rad；转向超过 90° 时改用反向轮速，零速保持当前舵角。
- `StringInvKinematics(...,phase,speed_gain,rotate_gain)` 中的单个 `phase` 是输入平移速度的坐标变换角，区别于每轮编码器零位数组。

舵轮正解由四轮实际轮速、实际舵角构造八个轮心速度分量，每次独立最小二乘计算，不累加历史输出。`String_ID::OmniInvDynamics()` 保留原有公开方法名。

## 秩不足与使用边界

四轮平行时，例如都朝 X，驱动力不能直接产生任意 Fy。分配器将旋转项按几何尺度归一化，对称矩阵特征值低于最大值 `1e-6` 的方向舍弃，输出当前舵角能产生的请求分量对应的最小范数轮端力矩；不可实现的请求不保证被跟踪。归一化空间为 `[Fx,Fy,Mz/r]`。

参数要求：正轮半径、正几何尺度，合法索引和轮数，有限角度/速度。接地模型要求轮子满足滚动约束；悬空或打滑时，轮速推算的车速不等于实际车速。轮端转速、力矩和功率限制由调用层负责。

## 验证

运行 `python tests/omni_swerve_calculation_test.py`。测试包含独立刚体轮速断言、三/四轮全向正逆往返、组合运动、重复调用、舵角零位/整圈偏移、跨圈和反转、零速保持、力/转矩重构、虚功以及平行/近似平行舵角。

公式参考：[Modern Robotics 13.4](https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-4-odometry/)、[WPILib 舵轮运动学](https://docs.wpilib.org/en/latest/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html)。具体调用见同目录 `README详细调用.md`。
