(velocity-smoother-impl)=
# 20. VelocitySmoother 实现

> 归属 [§7 速度平滑器](../07_velocity_smoother.md)
>
> `autonomy::control::utils::VelocitySmoother` 移植自 Nav2 `nav2_velocity_smoother`，对 `cmd_vel` 施加加速度约束与 deadband 过滤。  
> 公式见 [03_math.md §3.11](../03_math.md)。

| 维度 | 说明 |
|------|------|
| 源码 | `autonomy/control/utils/velocity_smoother.*` |
| 状态 | ✅ 算法已实现；pub/sub 定时器待接线 |
| 配置 Proto | `smoother_options.proto`（Lua 接线待完成） |

---

## 1. 架构

```
Controller / 上层
    │ cmd_vel_raw
    ▼
VelocitySmoother
├── inputCommandCallback()    # 接收命令
├── smootherTimer()           # @ smoothing_frequency
│   ├── 超时检测
│   ├── 获取 v_curr (OPEN/CLOSED_LOOP)
│   ├── clamp + η 缩放 + applyConstraints
│   └── deadband
└── OdomSmoother (CLOSED_LOOP)
    │
    ▼
cmd_vel → 底盘
```

---

## 2. 数学原理（Step-by-Step）

### Step 1：输入 clamp

$$
v_{cmd} \leftarrow \mathrm{clamp}(v_{cmd},\; v_{min},\; v_{max})
$$

三轴 $(v_x, v_y, \omega_z)$ 独立 clamp。

### Step 2：判定加速/减速模式

加速 $\Leftrightarrow |v_{cmd}| \geq |v_{curr}| \land v_{curr}\cdot v_{cmd} \geq 0$

| 模式 | $\Delta v_{max}$ | $\Delta v_{min}$ |
|------|------------------|------------------|
| 加速 | $a/f$ | $-a/f$ |
| 减速 | $-d/f$ | $d/f$（$d<0$） |

### Step 3：同步缩放 η

`scale_velocities=true` 时，三轴取使 $|1-\eta|$ 最小的 $\eta$（`findEtaConstraint`）。

### Step 4：应用约束

$$
v_{out} = v_{curr} + \mathrm{clamp}(\eta \Delta v,\; \Delta v_{min},\; \Delta v_{max})
$$

### Step 5：Deadband

$|v_{out}| < v_{deadband} \Rightarrow 0$

### Step 6：超时

超过 `velocity_timeout` 无新命令 → 发零速。

---

## 3. 反馈模式

| 模式 | `v_curr` 来源 |
|------|---------------|
| OPEN_LOOP | 上次输出 `last_cmd_` |
| CLOSED_LOOP | `OdomSmoother` 滑动平均 |

---

## 4. 默认参数

| 参数 | 默认 |
|------|------|
| `smoothing_frequency` | 20 Hz |
| `max_velocity` | [0.50, 0, 2.5] |
| `max_accel` | [2.5, 0, 3.2] |
| `max_decel` | [-2.5, 0, -3.2] |
| `velocity_timeout` | 1.0 s |
| `deadband_velocity` | [0, 0, 0] |

---

## 5. 源码对照

| 方法 | 文件 | 职责 |
|------|------|------|
| `findEtaConstraint()` | `velocity_smoother.cpp` | 单轴 η |
| `applyConstraints()` | `velocity_smoother.cpp` | 输出速度 |
| `smootherTimer()` | `velocity_smoother.cpp` | 主循环 |
| `OdomSmoother::updateState()` | `odometry_utils.cpp` | 滑动平均 |

---

## 6. 数值示例

$f=20$ Hz，$a=2.5$，$v_{curr}=0.2$，$v_{cmd}=1.0$：

| 步 | 结果 |
|----|------|
| $\Delta v_{max}=0.125$ | 每周期最大增量 |
| $\eta=0.125/0.8=0.156$ | 缩放因子 |
| $v_{out}=0.325$ m/s | 第一周期输出 |

约 8 周期（0.4 s）达到 1.0 m/s。

---

## 7. 集成待办

```
□ 在 ControllerServer 或独立节点实例化
□ 订阅 cmd_vel_raw，发布 cmd_vel
□ 定时调用 smootherTimer() @ smoothing_frequency
□ Lua 加载 VelocitySmootherOptions
```

---

## 8. 参考文献

1. Nav2 velocity_smoother: [configuration](https://navigation.ros.org/configuration/packages/configuring-velocity-smoother.html)
