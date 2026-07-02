# 7. 速度平滑器（VelocitySmoother）

`VelocitySmoother` 移植自 Nav2 `nav2_velocity_smoother`，在控制器输出与底盘之间施加**运动学约束**，防止加速度突变、消除 deadband 以下的抖动。

## 7.1 定位

| 维度 | 说明 |
|------|------|
| 输入 | 原始 `cmd_vel` 命令 |
| 输出 | 平滑后的 `cmd_vel` |
| 约束 | 最大/最小速度、最大加速度/减速度 |
| 反馈 | OPEN_LOOP（用上次输出）或 CLOSED_LOOP（用 OdomSmoother） |
| 状态 | 算法已实现；pub/sub 定时器待接线 |

## 7.2 向量约定

三轴向量索引 $(v_x, v_y, \omega_z)$ 对应 `[0], [1], [2]`：

| 索引 | 分量 | 差速机器人典型值 |
|------|------|------------------|
| 0 | $v_x$ | max 0.50, min -0.50 m/s |
| 1 | $v_y$ | 0（非完整约束） |
| 2 | $\omega_z$ | max 2.5, min -2.5 rad/s |

## 7.3 算法流程

```
inputCommandCallback(cmd_vel_raw)
    │
    ▼
smootherTimer()  @ smoothing_frequency
    │
    ├─ 1. 超时检测 → 无新命令超过 velocity_timeout → 发零速
    │
    ├─ 2. 获取当前速度 v_curr
    │      OPEN_LOOP: 上次输出 last_cmd_
    │      CLOSED_LOOP: OdomSmoother.getTwistStamped()
    │
    ├─ 3. Clamp 命令到 [min_velocity, max_velocity]
    │
    ├─ 4. 计算同步缩放因子 η（若 scale_velocities=true）
    │
    ├─ 5. 逐轴 applyConstraints → v_out
    │
    ├─ 6. Deadband 过滤
    │
    └─ 7. 保存 last_cmd_，发布 v_out
```

## 7.4 约束增量计算

对单个分量，设当前速度 $v_{curr}$、目标命令 $v_{cmd}$、控制频率 $f$、加速度 $a > 0$、减速度 $d < 0$：

**判定加速还是减速**（加速 $\Leftrightarrow |v_{cmd}| \geq |v_{curr}| \land v_{curr} \cdot v_{cmd} \geq 0$）：

**加速模式**：

$$
\Delta v_{\max} = \frac{a}{f}, \quad \Delta v_{\min} = -\frac{a}{f}
$$

**减速/反向模式**：

$$
\Delta v_{\max} = -\frac{d}{f}, \quad \Delta v_{\min} = \frac{d}{f}
$$

> 减速度 $d$ 必须为**负数**（如 -2.5），加速度 $a$ 必须为**正数**（如 2.5）。构造函数会校验符号。

## 7.5 同步缩放（scale_velocities）

当 `scale_velocities = true` 时，三轴共用同一缩放因子 $\eta$，保证 $(v_x, v_y, \omega_z)$ 同步变化，避免轨迹扭曲。

**单轴 eta 计算**（`findEtaConstraint`）：

$$
\Delta v = v_{cmd} - v_{curr}
$$

$$
\eta_i = \begin{cases}
\Delta v_{\max} / \Delta v & \mathrm{if}\ \Delta v > \Delta v_{\max} \\
\Delta v_{\min} / \Delta v & \mathrm{if}\ \Delta v < \Delta v_{\min} \\
-1 & \mathrm{otherwise}
\end{cases}
$$

`otherwise` 表示该轴无需缩放。

**全局 eta**：取所有 $\eta_i > 0$ 中使 $|1 - \eta_i|$ 最小的值；若无则 $\eta = 1.0$。

**应用约束**（`applyConstraints`）：

$$
v_{out} = v_{curr} + \operatorname{clamp}(\eta \cdot \Delta v,\; \Delta v_{\min},\; \Delta v_{\max})
$$

### 数值示例

设 $f = 20$ Hz，$a = 2.5$ m/s²，$v_{curr} = 0$，$v_{cmd} = 1.0$ m/s：

$$
\Delta v_{\max} = 2.5 / 20 = 0.125\,\mathrm{m/s}
$$

每周期最多增加 0.125 m/s，从 0 到 1.0 m/s 需约 8 个周期（0.4 s）。

## 7.6 Deadband 过滤

$$
v_{out} = \begin{cases}
0 & |v_{out}| < v_{\mathrm{deadband}} \\
v_{out} & \mathrm{otherwise}
\end{cases}
$$

默认 deadband 为 $(0, 0, 0)$，即不过滤。可设 $(0.01, 0, 0.01)$ 消除微小抖动。

## 7.7 超时保护

若距上次命令时间超过 `velocity_timeout`（默认 1.0 s）：

1. 第一次超时：将命令置零，继续平滑减速
2. 已停止（`stopped_ = true`）：不再发布

防止控制器崩溃后机器人持续执行旧命令。

## 7.8 反馈模式

| 模式 | `open_loop_` | 当前速度来源 | 适用 |
|------|-------------|-------------|------|
| OPEN_LOOP | true | 上次平滑输出 `last_cmd_` | 低延迟、无 odom |
| CLOSED_LOOP | false | `OdomSmoother` 滑动平均 | 精确跟踪实际速度 |

CLOSED_LOOP 模式下，`OdomSmoother` 对 `odom_duration`（默认 0.1 s）窗口内的里程计做算术平均：

$$
\bar{v}_x = \frac{1}{N}\sum_{i=1}^{N} v_{x,i}
$$

## 7.9 配置参数

**Proto**（`smoother_options.proto`）：

| 字段 | 默认 | 说明 |
|------|------|------|
| `smoothing_frequency` | 20.0 Hz | 平滑定时器频率 |
| `feedback` | `"OPEN_LOOP"` | 或 `"CLOSED_LOOP"` |
| `scale_velocities` | false | 三轴同步缩放 |
| `max_velocity` | [0.50, 0.0, 2.5] | 速度上界 |
| `min_velocity` | [-0.50, 0.0, -2.5] | 速度下界 |
| `max_accel` | [2.5, 0.0, 3.2] | 加速度上界 |
| `max_decel` | [-2.5, 0.0, -3.2] | 减速度（负值） |
| `deadband_velocity` | [0, 0, 0] | Deadband |
| `velocity_timeout` | 1.0 s | 命令超时 |
| `odom_topic` | `"odom"` | 闭环反馈话题 |
| `odom_duration` | 0.1 s | 里程计平滑窗口 |

## 7.10 与 ControllerServer 的关系

```
ControllerServer                    VelocitySmoother
     │                                    │
     │ ComputeVelocityCommands()          │
     │──────── raw cmd_vel ──────────────→│ inputCommandCallback()
     │                                    │ smootherTimer()
     │                                    │──────── smooth cmd_vel ──→ 底盘
```

VelocitySmoother 可部署为：

1. **ControllerServer 内部组件**：在 `ComputeAndPublishVelocity()` 中调用
2. **独立节点**：订阅 `cmd_vel_raw`，发布 `cmd_vel`（Nav2 默认方式）

当前 Autonomy 两种方式的 pub/sub 均未接线。

## 7.11 调参建议

| 场景 | max_accel | scale_velocities | feedback |
|------|-----------|------------------|----------|
| 室内慢速 | [1.0, 0, 2.0] | true | CLOSED_LOOP |
| 高速 AGV | [3.0, 0, 4.0] | true | CLOSED_LOOP |
| 仿真调试 | [5.0, 0, 5.0] | false | OPEN_LOOP |
| 消除抖动 | deadband [0.01, 0, 0.02] | — | — |

**原则**：`smoothing_frequency` 应 ≥ `controller_frequency`；加速度限制应匹配机器人硬件规格。
