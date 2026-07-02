# 8. 障碍感知与代价地图

本文描述 Autonomy 当前**实际承担障碍感知职能**的 `map/costmap_2d` 模块：ObstacleLayer、VoxelLayer 与膨胀层。

> Perception 模块完整实现后，障碍几何可经 Perception 插件输出并写入同一 costmap 管线。当前导航栈依赖本章所述路径。

---

## 8.1 架构位置

```
传感器观测
    │
    ▼
ObservationBuffer（时间缓冲）
    │
    ▼
ObstacleLayer / VoxelLayer（标记/清除）
    │
    ▼
InflationLayer（安全膨胀）
    │
    ▼
Costmap2DWrapper → Planning / Control
```

源码：`autonomy/map/costmap_2d/layers/`。

---

## 8.2 ObstacleLayer

### 8.2.1 职责

| 功能 | 说明 |
|------|------|
| **Marking** | 将障碍点投影到 costmap 栅格，设为 `LETHAL_OBSTACLE` |
| **Clearing** | 沿射线追踪，将自由空间栅格设为 `FREE_SPACE` |
| **高度过滤** | 忽略 `z < min_obstacle_height` 或 `z > max_obstacle_height` 的点 |

### 8.2.2 激光处理流程

1. 接收 `LaserScan`，遍历每个有效 range $r_i$
2. TF 变换：传感器系 → `global_frame`
3. 计算终点 $p_{end} = o + r_i \cdot \hat{d}_i$
4. **清除**：从 $o$ 到 $p_{end}$ 沿射线标记自由
5. **标记**：$p_{end}$ 处标记障碍

### 8.2.3 配置示例

```lua
obstacle_layer = {
    enabled = true,
    observation_sources = "scan",
    scan = {
        topic = "/scan",
        sensor_frame = "laser_link",
        data_type = "LaserScan",
        marking = true,
        clearing = true,
        inf_is_valid = false,
        obstacle_range = 2.5,
        raytrace_range = 3.0,
        max_obstacle_height = 2.0,
        min_obstacle_height = 0.0,
    },
}
```

---

## 8.3 VoxelLayer

3D 点云障碍处理，适用于深度相机或 3D 激光：

| 步骤 | 说明 |
|------|------|
| 体素化 | 将点云划分为 3D 体素网格 |
| 高度投影 | 将 3D 障碍投影到 2D costmap |
| 清除/标记 | 同 ObstacleLayer 逻辑 |

源码：`voxel_layer.hpp` / `voxel_layer.cpp`。

---

## 8.4 膨胀层（InflationLayer）

### 8.4.1 目的

在障碍周围生成**代价梯度**，使规划器保持安全距离。

### 8.4.2 数学

对障碍栅格 $p$，膨胀半径 $r_{infl}$ 内栅格 $q$ 的代价：

$$
\mathrm{cost}(q) = \max_{p \in \mathcal{O}} f(\|q - p\|)
$$

常用分段函数：

$$
f(d) = \begin{cases}
\mathrm{LETHAL} & d \leq r_{robot} \\
\mathrm{INSCRIBED} \cdot e^{-\lambda(d - r_{robot})} & r_{robot} < d \leq r_{infl} \\
0 & d > r_{infl}
\end{cases}
$$

详见 [03_math.md §3.6](03_math.md#36-代价地图障碍标记)。

### 8.4.3 配置

```lua
inflation_layer = {
    enabled = true,
    inflation_radius = 0.55,       -- 膨胀半径 (m)
    cost_scaling_factor = 10.0,    -- 衰减系数 λ
}
```

---

## 8.5 代价地图代价值

| 值 | 宏 | 含义 |
|----|-----|------|
| 0 | `FREE_SPACE` | 自由 |
| 1–252 | 梯度代价 | 距障碍越近代价越高 |
| 253 | `INSCRIBED_INFLATED_OBSTACLE` | 内切圆膨胀边界 |
| 254 | `LETHAL_OBSTACLE` | 致命障碍 |
| 255 | `NO_INFORMATION` | 未知 |

---

## 8.6 ObservationBuffer

```cpp
class ObservationBuffer {
    void bufferObservation(const Observation& obs);
    void getObservations(std::vector<Observation>& observations);
    void resetLastUpdated();
};
```

| 参数 | 说明 |
|------|------|
| `observation_keep_time` | 观测保留时间窗口 |
| `expected_update_rate` | 预期传感器更新频率 |

---

## 8.7 与 Planning / Control 的接口

```cpp
// Costmap2DWrapper 提供
bool getCost(x, y);           // 查询单栅格代价
bool isCollisionFree(pose);   // 碰撞检测
void updateMap();             // 触发图层更新
```

Planning 全局规划器与 Control 局部 costmap 均通过 `Costmap2DWrapper` 访问障碍信息。

---

## 8.8 与 Perception 的融合（规划）

完整 Perception 实现后，障碍融合策略：

| 策略 | 说明 |
|------|------|
| **叠加** | Perception 障碍与激光障碍取并集（max cost） |
| **优先级** | 视觉检测优先于激光（语义障碍） |
| **独立层** | 新增 `SemanticLayer` 插件，与 ObstacleLayer 并列 |

```lua
-- 规划
plugins = { "static_layer", "obstacle_layer", "semantic_layer", "inflation_layer" }
```

---

## 8.9 调试建议

| 工具 | 用途 |
|------|------|
| RViz costmap 显示 | 可视化障碍/膨胀区域 |
| `autonomy_planning_test` | 离线验证路径与障碍关系 |
| 日志 `obstacle_range` | 确认激光数据是否到达 |

| 现象 | 处理 |
|------|------|
| costmap 全未知 | 检查 static_layer 是否加载地图 |
| 障碍不更新 | 确认 `/scan` topic 与 `sensor_frame` TF |
| 虚假障碍 | 调整 `min_obstacle_height`、过滤地面点 |
