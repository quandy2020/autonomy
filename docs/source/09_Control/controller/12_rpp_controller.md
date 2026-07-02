(rpp-controller)=
# 12. Regulated Pure Pursuit (RPP) Controller

> 归属 [§8 局部控制器 · §8.4](../08_controller_algorithms.md#84-regulated-pure-pursuitrpp)
>
> **Regulated Pure Pursuit** 是 Nav2 默认局部控制器 `nav2_regulated_pure_pursuit_controller`，在经典 Pure Pursuit 几何跟踪上增加曲率/障碍/目标三项速度调节。  
> 几何推导见 [03_math.md §3.3–3.4](../03_math.md)；Autonomy 工具函数见 `controller_utils.*`。

| 维度 | 说明 |
|------|------|
| 类型 | 几何路径跟踪 |
| 输入 | 全局路径、位姿、速度、costmap（障碍调节） |
| 输出 | $(v_x^{cmd}, \omega_z^{cmd})$ |
| Autonomy 状态 | ⏳ 工具函数就绪，插件未实现 |

---

## 1. 核心论文清单

#### ① 《Implementation of the Pure Pursuit Path Tracking Algorithm》(1992)

| 字段 | 内容 |
|------|------|
| **作者** | R. Coulter |
| **出处** | CMU-RI-TR-92-01 |
| **核心价值** | **Pure Pursuit 经典文献**。lookahead 点 + 曲率 $\kappa = 2y/L_d^2$。 |
| **对应本文** | [§3 Step 1–4](#step-1-pure-pursuit) |

#### ② Nav2 Regulated Pure Pursuit（工程实现）

| 字段 | 内容 |
|------|------|
| **来源** | `nav2_regulated_pure_pursuit_controller` |
| **核心价值** | 在 PP 上叠加**曲率调节、障碍减速、接近目标减速**，成为 Nav2 默认控制器。 |
| **对应本文** | [§3 Step 5](#step-5-regulated) |

#### ③ 《The Marathon 2》(2020)

| 字段 | 内容 |
|------|------|
| **核心价值** | Nav2 架构中 RPP 作为低算力、高可靠默认选项。 |

---

## 2. 架构

```
SetPlan(path)
    │
    ▼
变换路径到机器人坐标系
    │
    ▼
GetLookAheadPoint(L_d, path)     ← autonomy/control/utils
    │
    ▼
κ = 2y* / L_d²  →  ω = κ · v_x
    │
    ▼
Regulated 速度调节（曲率 / 障碍 / 目标）
    │
    ▼
cmd_vel
```

---

## 3. 数学原理（Step-by-Step）

(step-1-pure-pursuit)=
### Step 1：Pure Pursuit 几何

机器人坐标系下 lookahead 点 $P=(x_l, y_l)$，圆弧曲率：

$$
\kappa = \frac{2 y_l}{x_l^2 + y_l^2} \approx \frac{2 y_l}{L_d^2}
$$

角速度：$\omega_z = \kappa \cdot v_x$。

### Step 2：Lookahead 距离

Nav2 动态调节：

$$
L_d = \mathrm{clamp}(\alpha |v_x| + L_{base},\; L_{min},\; L_{max})
$$

### Step 3：圆-线段交点（可选）

`CircleSegmentIntersection` 在路径段上求距原点 $L_d$ 的交点（见 `controller_utils.cpp`）。

### Step 4：沿路径弧长插值

`GetLookAheadPoint` 累积段长 $S_k$，在线段上 `LinearInterpolation` 得 $p_{la}$。

(step-5-regulated)=
### Step 5：Regulated 速度调节

$$
v_x \leftarrow \min\!\left(v_{desired},\;
\frac{\omega_{max}}{|\kappa|},\;
\alpha \cdot d_{obs},\;
\beta \cdot d_{goal}\right)
$$

| 调节项 | 目的 |
|--------|------|
| 曲率 | 急弯降速，防侧翻 |
| 障碍 | 近障降速 |
| 目标 | 接近终点平滑减速 |

### Step 6：输出

$$
v_x^{cmd} = v_x,\quad \omega_z^{cmd} = \kappa \cdot v_x
$$

---

## 4. Autonomy 已有工具

| 函数 | 文件 |
|------|------|
| `GetLookAheadPoint()` | `controller_utils.cpp` |
| `CircleSegmentIntersection()` | `controller_utils.cpp` |
| `LinearInterpolation()` | `controller_utils.cpp` |

---

## 5. 配置参考（Nav2）

| 参数 | 典型值 | 说明 |
|------|--------|------|
| `lookahead_dist` | 0.6 m | 基础前瞻 |
| `min_lookahead_dist` | 0.3 m | 下限 |
| `max_lookahead_dist` | 0.9 m | 上限 |
| `use_velocity_scaled_lookahead` | true | 速度缩放 |
| `use_regulated_linear_velocity_scaling` | true | 曲率调节 |
| `use_cost_regulated_linear_velocity_scaling` | true | 障碍调节 |
| `regulated_linear_scaling_min_radius` | 0.9 m | 曲率半径阈值 |

---

## 6. 调参要点

| 场景 | 建议 |
|------|------|
| 窄通道 | 减小 lookahead，启用 cost regulated |
| 开阔地 | 增大 lookahead，提高速度 |
| 振荡 | 增大 min_lookahead，降低 v_x |

---

## 7. Autonomy 移植清单

```
□ 继承 ControllerInterface
□ 复用 controller_utils 几何函数
□ 实现 Regulated 三项速度调节
□ 接入 costmap 障碍距离查询
□ 作为首个控制器插件（Phase 1 推荐）
```

---

## 8. 参考文献

1. Coulter, R. (1992). *Implementation of the Pure Pursuit Path Tracking Algorithm*. CMU-RI-TR-92-01.
2. Nav2 RPP: [configuring-rpp-controller](https://navigation.ros.org/configuration/packages/configuring-regulated-pp-controller.html)
