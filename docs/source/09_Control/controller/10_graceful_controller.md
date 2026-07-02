(graceful-controller)=
# 10. Graceful Controller

> 归属 [§8 局部控制器 · §8.2](../08_controller_algorithms.md#82-graceful-controller)
>
> **Graceful Controller** 是 Nav2 `nav2_graceful_controller`，基于 Park & Kuipers 的**平滑控制律**，通过指数稳定 motion target 生成舒适、可预测的差速机器人轨迹。  
> Autonomy 配置见 `config/control/controller.lua` 中 `graceful_controller` 段。

| 维度 | 说明 |
|------|------|
| 类型 | 几何 + Lyapunov 平滑控制 |
| 输入 | 路径、位姿、速度、costmap（可选碰撞检测） |
| 输出 | $(v_x^{cmd}, \omega_z^{cmd})$ |
| Autonomy 状态 | ⏳ 配置预留，插件未实现 |

---

## 1. 核心论文清单

#### ① 《A Smooth Control Law for Graceful Motion…》(2011)

| 字段 | 内容 |
|------|------|
| **作者** | Jong Jin Park, Benjamin Kuipers |
| **出处** | *IEEE ICRA*, 2011, [DOI:10.1109/ICRA.2011.5980167](https://doi.org/10.1109/ICRA.2011.5980167) |
| **核心价值** | **Graceful 控制律理论**。基于奇异摄动 Lyapunov 设计，快慢子系统分离，全局收敛到目标位姿且无奇异点。 |
| **对应本文** | [§3](#3-数学原理-step-by-step) |

#### ② 《Graceful Navigation for Mobile Robots in Dynamic and Uncertain Environments》(2016)

| 字段 | 内容 |
|------|------|
| **作者** | Jong Jin Park (PhD Thesis) |
| **核心价值** | 扩展到动态不确定环境；Nav2 实现引用此工作的 motion_target 思想。 |

#### ③ Nav2 Graceful Controller 文档

| 字段 | 内容 |
|------|------|
| **链接** | [configuring-graceful-motion-controller](https://docs.nav2.org/configuration/packages/configuring-graceful-motion-controller.html) |
| **核心价值** | 工程参数：lookahead、减速区、初始旋转、碰撞检测。 |

---

## 2. 架构

```
路径 → 机器人坐标系
    │
    ├─ initial_rotation? → 原地对齐路径航向
    │
    ├─ 计算 motion_target（指数稳定 lookahead 点）
    │
    ├─ 平滑控制律 → (v_x, ω_z)
    │
    ├─ slowdown_radius 减速
    │
    └─ use_collision_detection? → costmap 弧采样
         │
         ▼
      cmd_vel
```

---

## 3. 数学原理（Step-by-Step）

### Step 1：自我中心极坐标

相对 motion target 的极坐标 $(r, \phi)$，$r$ 为距离，$\phi$ 为方位角。

### Step 2：快慢子系统（Park & Kuipers 2011）

- **慢子系统**（位置）：参考航向 $\theta_d$ 由状态反馈得到
- **快子系统**（转向）：实际航向 $\theta$ 指数收敛到 $\theta_d$

保证轨迹**平滑、无奇异**，全局收敛到目标位姿。

### Step 3：Motion Target

在路径前方距离 $L_d \in [L_{min}, L_{max}]$ 处设置目标点，Nav2 实现用指数稳定距离生成平滑曲线。

### Step 4：线速度

$$
v_x \in [v_{linear\_min},\; v_{linear\_max}]
$$

接近目标时（$d_{goal} < r_{slowdown}$）：

$$
v_x \leftarrow v_{min} + (v_{max} - v_{min}) \cdot \frac{d_{goal}}{r_{slowdown}}
$$

### Step 5：角速度

由控制律输出，约束 $|\omega_z| \leq \omega_{max}$。

### Step 6：碰撞检测（可选）

沿预测弧采样 $p(s) = (x+s\cos\theta,\; y+s\sin\theta)$，若 costmap 代价 ≥ INSCRIBED 则拒绝命令。

---

## 4. Autonomy 配置

```lua
graceful_controller = {
    max_lookahead = 0.55,
    min_lookahead = 0.25,
    v_linear_max = 0.5,
    v_linear_min = 0.05,
    v_angular_max = 1.0,
    slowdown_radius = 0.5,
    initial_rotation = true,
    allow_backward = false,
    use_collision_detection = true,
},
```

---

## 5. 调参要点

| 现象 | 调整 |
|------|------|
| 切弯穿墙 | `use_collision_detection=true`，减小 max_lookahead |
| 接近目标冲过头 | 增大 slowdown_radius |
| 起步摆头 | 启用 initial_rotation |
| 窄道不稳 | 减小 max_lookahead，降低 v_linear_max |

---

## 6. 与 RPP / DWB 对比

| 维度 | Graceful | RPP | DWB |
|------|----------|-----|-----|
| 平滑性 | ✅ 最优 | 中 | 中 |
| 算力 | 低 | 低 | 中 |
| 动态避障 | 中（可选碰撞检测） | 弱 | 强 |
| 适用 | 室内服务机器人 | 通用默认 | 复杂障碍 |

---

## 7. 参考文献

1. Park, J. J., & Kuipers, B. (2011). *A Smooth Control Law for Graceful Motion of Differential Wheeled Mobile Robots in 2D Environment*. ICRA.
2. Park, J. J. (2016). *Graceful Navigation for Mobile Robots in Dynamic and Uncertain Environments*. PhD thesis, Univ. of Michigan.
