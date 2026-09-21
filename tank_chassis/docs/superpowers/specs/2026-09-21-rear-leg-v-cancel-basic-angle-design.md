# 后腿 V 取消收腿与 IMU 失效基础角度设计

## 目标

后部两个 J6248 在正常模式下继续使用 IMU pitch/roll 姿态控制，在上台阶收腿模式下继续使用编码器位置—速度串级控制。本次改动解决两个问题：

1. 收腿目标因负载、机械限位或 PID 参数而长时间无法到达时，操作手可以再次按 V，立即取消继续收腿。
2. 正常姿态控制所需的 IMU 失效时，不给固定开环力矩，而是让左右后腿缓慢移动到各自经过标定的基础角度并保持。

前部 J4310 状态机、B 键流程和底盘轮毂前进命令不属于本次修改范围。

## 控制模式

后部状态机新增：

- `UP_STAIR_BEHIND_MOTOR_BASIC_ANGLE_CONTROL`：IMU 暂时不可用于姿态控制时，通过编码器位置—速度串级 PID 缓慢回到基础角度并保持。

已有状态保持：

- `DISABLED`：后部控制关闭，输出零力矩。
- `RECOVERING`：IMU 姿态控制以现有 300 ms 斜坡恢复。
- `ATTITUDE_HOLD`：使用 pitch/roll 姿态串级 PID。
- `RETRACTING`：使用编码器位置控制执行收腿。
- `RETRACTED_HOLD`：使用编码器位置控制保持收腿目标。

## 状态转移

### 正常控制中的 IMU 失效

```text
RECOVERING 或 ATTITUDE_HOLD
    -- IMU 失效且基础角度配置有效 --> BASIC_ANGLE_CONTROL

BASIC_ANGLE_CONTROL
    -- IMU 恢复 --> RECOVERING
    -- 300 ms 恢复完成 --> ATTITUDE_HOLD
```

进入 `BASIC_ANGLE_CONTROL` 时，以左右当前编码器连续角度作为斜坡起点，按配置速度分别移动到左右基础角度。到达目标后继续使用同一位置闭环保持。

### 收腿过程中再次按 V

```text
RETRACTING 或 RETRACTED_HOLD
    -- 新 V 且 IMU 有效 --> RECOVERING
    -- 新 V 且 IMU 无效、基础角度配置有效 --> BASIC_ANGLE_CONTROL
```

因此 V 在收腿期间不再被无条件忽略。新的 V 动作序号会立即取消收腿位置目标，并根据 IMU 是否可用选择姿态恢复或基础角度控制。

### 收腿期间单纯 IMU 失效

`RETRACTING` 和 `RETRACTED_HOLD` 本身不依赖 IMU。没有新的 V 取消请求时，IMU 失效不会打断正在进行的收腿或收腿保持。

## 基础角度配置

后腿配置增加：

```cpp
float basic_target_rad[2];
float basic_speed_rad_s;
float basic_position_tolerance_rad;
```

约束如下：

- 左右基础目标必须为有限值并位于各自机械安全角度范围内。
- 跨 0° 的机械区间使用与收腿目标相同的连续角度展开规则。
- `basic_speed_rad_s` 必须大于 0。
- `basic_position_tolerance_rad` 必须大于 0。
- 生产默认值全部为 `0.0f`，在实车标定前基础角度配置无效，不允许产生未经确认的自动动作。

基础角度配置与收腿配置分别校验；某一组无效不应破坏另一组已经标定的功能。

## 位置控制接口

基础角度控制复用现有左右后腿位置环和速度环，不新增 PID 对象。状态机增加通用位置控制接口：

```cpp
bool Uses_Position_Control() const;
float Get_Position_Target_Angle(uint8_t id) const;
```

`Uses_Position_Control()` 在 `RETRACTING`、`RETRACTED_HOLD` 和 `BASIC_ANGLE_CONTROL` 中返回 `true`。任务循环使用通用目标接口执行：

```text
位置目标 - 连续编码器角度
          ↓
       位置 PID
          ↓ 目标速度
       速度 PID
          ↓ 力矩
       J6248
```

已有 `Uses_Retract_Position_Control()` 和 `Get_Retract_Target_Angle()` 保留，避免破坏现有测试或其他调用者；任务循环改用通用接口。

位置控制力矩不乘 `Get_Motor_Direction()`，继续经过：

- FSM 机械限位方向保护；
- 左右电机独立力矩增益；
- J6248 最终力矩钳位。

## IMU 有效性与安全处理

IMU 是否可用统一定义为：IMU 新鲜有效，并且 pitch、roll、pitch rate、roll rate 均为有限值。

- 姿态状态发现 IMU 不可用时，不再直接因为 IMU 原因清零；基础角度配置和双侧编码器反馈有效时进入 `BASIC_ANGLE_CONTROL`。
- `BASIC_ANGLE_CONTROL` 中 IMU 数据不参与位置 PID，只用于判断是否可以回到 `RECOVERING`。
- `RETRACTING` 和 `RETRACTED_HOLD` 中 IMU 数据不参与控制。
- 任一位置控制状态中，只要一个 J6248 掉线、编码器无效或机械角越界，两侧位置控制同时停止并进入 `DISABLED`。
- 控制权限关闭、机械总配置无效或基础角度配置无效时，不允许进入基础角度控制。
- 基础角度配置无效且姿态模式丢失 IMU 时，保持安全的 `DISABLED` 零力矩行为。

## V 动作序号

继续使用现有 `rear_retract_action_sequence`。每个新序号只处理一次：

- `ATTITUDE_HOLD` 中的新序号开始收腿。
- `RETRACTING` 或 `RETRACTED_HOLD` 中的新序号取消收腿。
- `BASIC_ANGLE_CONTROL`、`RECOVERING` 和 `DISABLED` 同步消费不能执行的旧序号，防止 IMU 恢复后重放旧按键。

## 任务循环

`up_stair_task` 保留三类互斥输出：

1. 后腿禁用或双侧不可控：两侧零力矩，复位姿态与位置 PID。
2. `Uses_Position_Control()`：复位姿态 PID，使用现有左右收腿位置—速度 PID 控制 FSM 给出的通用目标。
3. `Uses_Attitude_Control()`：复位位置 PID，执行现有 pitch/roll 姿态串级控制。

基础角度控制和收腿控制使用同一位置 PID，但由状态机提供不同的斜坡目标。同一周期不允许同时输出位置控制力矩和姿态控制力矩。

## 验证

状态机单元测试必须覆盖：

- 姿态保持时 IMU 失效，进入基础角度控制。
- 基础角度目标从当前编码器角度按速度限制渐变，而不是瞬间跳变。
- 到达基础角度后继续位置保持。
- 基础角度控制期间 IMU 恢复，进入 `RECOVERING`，完成后进入 `ATTITUDE_HOLD`。
- 收腿期间 IMU 失效但没有新 V，继续收腿。
- 收腿期间再次按 V 且 IMU 有效，立即进入 `RECOVERING`。
- 收腿期间再次按 V 且 IMU 无效，立即进入基础角度控制。
- 基础角度配置为零、越界、非有限值、速度非正或容差非正时被拒绝。
- 跨 0° 的基础目标被正确展开。
- 基础角度控制中任一后电机反馈失效，两侧进入 `DISABLED` 且力矩限制返回零。

任务集成契约必须覆盖：

- 任务循环使用通用位置控制状态和通用位置目标接口。
- 基础角度生产配置显式保持零值安全默认。
- 位置分支继续复用左右位置—速度 PID，且不乘电机方向系数。
- 姿态和位置 PID 在模式切换时互相复位。
