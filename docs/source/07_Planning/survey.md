# 路径规划算法综述（Survey）

本文从学术与工程视角综述移动机器人路径规划的主要方法，并说明 Autonomy `planning` 模块在其中的定位与选型依据。

> 各算法的详细公式推导见 [Planning 指南 · 数学公式](guide.md#planning-math)。

## 1. 问题定义

### 1.1 基本形式

给定配置空间 $\mathcal{C} \subseteq \mathbb{R}^n$（平面机器人常用 $SE(2)$），障碍区域 $\mathcal{C}_{\text{obs}}$，自由空间：

$$
\mathcal{C}_{\text{free}} = \mathcal{C} \setminus \mathcal{C}_{\text{obs}}
$$

起点 $q_s \in \mathcal{C}_{\text{free}}$，终点 $q_g \in \mathcal{C}_{\text{free}}$。求连续曲线 $\tau: [0,1] \to \mathcal{C}_{\text{free}}$，使 $\tau(0)=q_s$，$\tau(1)=q_g$。

### 1.2 优化目标

$$
J(\tau) = \int_0^1 \Big( w_l \|\tau'(s)\| + w_c \cdot c(\tau(s)) + w_k \|\tau''(s)\|^2 \Big) ds
$$

| 项 | 含义 |
|----|------|
| $w_l \|\tau'\|$ | 路径长度 |
| $w_c \cdot c(\cdot)$ | 障碍/代价惩罚 |
| $w_k \|\tau''\|^2$ | 曲率/平滑度 |

### 1.3 离散化

世界坐标到栅格索引：

$$
m_x = \left\lfloor \frac{x - x_0}{\Delta} \right\rfloor, \quad
m_y = \left\lfloor \frac{y - y_0}{\Delta} \right\rfloor
$$

每个栅格存储代价值 $c_{ij} \in [0,255]$，规划在栅格图上进行后转回世界坐标。

## 2. 算法分类体系

```
路径规划
├── 按规划范围
│   ├── 全局规划（Global Planning）    ← Autonomy planning 模块
│   └── 局部规划（Local Planning）     ← control 模块
│
├── 按搜索空间
│   ├── 构型空间（C-space）精确方法
│   ├── 栅格/图搜索方法               ← NavFn, Dijkstra, Theta*
│   ├── 采样-based 方法
│   └── 优化-based 方法
│
├── 按完备性
│   ├── 完备（Completeness）
│   ├── 概率完备（Probabilistically Complete）
│   └── 不完备
│
└── 按最优性
    ├── 最优（Optimal）
    ├── 渐近最优（Asymptotically Optimal）
    └── 启发式/近似最优
```

## 3. 图搜索方法

### 3.1 基础算法

| 算法 | 最优性 | 完备性 | 时间复杂度 | 空间复杂度 |
|------|--------|--------|-----------|-----------|
| BFS | 等权图最优 | 完备 | O(V+E) | O(V) |
| Dijkstra | 非负权最优 | 完备 | O((V+E) log V) | O(V) |
| A* | 可采纳启发式下最优 | 完备 | O(b^d) 最坏 | O(b^d) |
| Greedy Best-First | 不保证 | 不完备 | O(b^d) | O(b^d) |
| D* / D* Lite | 动态环境最优 | 完备 | 取决于变化量 | O(V) |

### 3.2 Autonomy 中的实现

| 算法 | 实现 | 搜索空间 | 特点 |
|------|------|----------|------|
| Dijkstra | `DijkstraPlanner` / NavFn | 8-隐式连通栅格 | 桶队列，反向传播 |
| A* | `NavfnPlanner` (`use_astar=true`) | 同上 | 欧氏启发式 |
| Theta* | `ThetaStarPlanner` | 4/8 连通 + LOS | 任意角路径 |

### 3.3 Any-Angle 方法

标准栅格搜索只能沿栅格方向移动，路径长度比真实最短路径长。Any-angle 方法在栅格搜索基础上允许任意方向直线连接：

| 方法 | 思路 | Autonomy 支持 |
|------|------|---------------|
| **Theta*** | A* + 视线检测捷径 | ✅ `ThetaStarPlanner` |
| **Lazy Theta*** | 延迟 LOS 验证 | ❌ |
| **Block A*** | 分块 LOS | ❌ |
| **Field D*** | 插值场引导 | ❌ |
| **Anya*** | 压缩栅格上的任意角 | ❌ |
| 后处理字符串拉直 | 先栅格搜索再 LOS 简化 | 部分（`PathSimplifier`） |

## 4. 采样-based 方法

### 4.1 主要算法

| 算法 | 策略 | 最优性 | 适用维度 |
|------|------|--------|----------|
| PRM | 随机采样构建路线图 | 概率完备 | 高维 |
| RRT | 随机扩展树 | 概率完备 | 高维 |
| RRT* | RRT + 重连优化 | 渐近最优 | 高维 |
| RRT-Connect | 双向 RRT | 概率完备，更快 | 高维 |
| BIT* | 批量隐式树 | 渐近最优 | 中-高维 |
| FMT* | 快速行进树 | 渐近最优 | 中-高维 |

### 4.2 与 Autonomy 的关系

Autonomy `planning` 模块**当前未实现**采样-based 全局规划器。对于高维构型空间（机械臂、多关节）或复杂非凸障碍环境，采样方法更合适。未来可通过插件机制集成。

## 5. 优化-based 方法

### 5.1 主要方法

| 方法 | 描述 | 优点 | 缺点 |
|------|------|------|------|
| CHOMP | 协变梯度优化轨迹 | 平滑 | 易陷局部最优 |
| STOMP | 随机轨迹优化 | 鲁棒 | 计算量大 |
| TrajOpt | 序列凸优化 | 处理约束 | 实现复杂 |
| GPMP2 | 高斯过程运动规划 | 概率性 | 理论要求高 |

### 5.2 Autonomy 中的优化元素

`SimpleSmoother` 最小化：

$$
E = w_d \sum_i \|p_i - o_i\|^2 + w_s \sum_i \|p_{i-1} - 2p_i + p_{i+1}\|^2
$$

这是路径**后处理**优化，不是全局搜索。详见 [Planning 指南 · 数学公式](guide.md#planning-math) §8。

## 6. 势场与导航函数方法

### 6.1 经典方法

| 方法 | 原理 | 问题 |
|------|------|------|
| 人工势场 (APF) | 引力+斥力 | 局部极小 |
| 调和势场 | 解拉普拉斯方程 | 计算量大 |
| **NavFn** | 平面波传播近似 | 网格依赖 |
| Fast Marching Method | 精确 Eikonal 方程求解 | 比 NavFn 精确但更慢 |

### 6.2 NavFn 的定位

NavFn 是导航函数方法的工程近似：

- 使用平面波模型代替精确 Eikonal 求解
- 桶队列 Dijkstra 传播
- 梯度跟踪提取路径
- 计算效率适合实时系统

## 7. 分层规划架构

实际机器人导航系统通常采用分层架构：

```
┌─────────────────────────────────────────┐
│  行为层 (Behavior Tree / State Machine)  │
│  决策：何时规划、重规划、恢复             │
├─────────────────────────────────────────┤
│  全局规划 (Global Planner)  ← planning   │
│  粗粒度路径，低频更新 (1-5 Hz)           │
├─────────────────────────────────────────┤
│  局部规划 (Local Planner)   ← control    │
│  细粒度轨迹，高频更新 (10-50 Hz)         │
├─────────────────────────────────────────┤
│  运动控制 (Controller)                   │
│  速度/角速度指令                         │
└─────────────────────────────────────────┘
```

Autonomy 完整导航栈：

| 层级 | 模块 | 频率 |
|------|------|------|
| 行为 | `navigator` | 事件驱动 |
| 全局规划 | `planning` | 1–5 Hz |
| 局部规划 | `control` | 10–50 Hz |
| 感知/地图 | `map`, `perception` | 5–20 Hz |

## 8. 算法对比总表

### 8.1 Autonomy 内置规划器

| 维度 | NavFn (Dijkstra) | NavFn (A*) | DijkstraPlanner | ThetaStarPlanner |
|------|------------------|------------|-----------------|------------------|
| 算法基础 | 导航势场 + Dijkstra | 导航势场 + A* | 同 NavFn Dijkstra | A* + LOS |
| 路径角度 | 网格 | 网格 | 网格 | 任意角 |
| 路径长度 | 较长 | 较长 | 较长 | 较短 |
| 平滑度 | 中 | 中 | 中 | 高 |
| 计算速度 | 快 | 中 | 快 | 较慢 |
| 确定性 | 高 | 高 | 高 | 高 |
| 窄通道 | 好 | 好 | 好 | 4-连通好，8-连通注意 |
| 默认推荐 | ★★★★ | ★★★ | ★★★★ | ★★★★（开阔环境） |

### 8.2 与业界方案对比

| 方案 | 代表实现 | 与 Autonomy 关系 |
|------|----------|------------------|
| NavFn | nav2_navfn_planner | 直接移植，行为一致 |
| Dijkstra | nav2_navfn_planner (Dijkstra mode) | 独立插件封装 |
| Theta* | nav2_theta_star_planner | 移植实现 |
| Smac Planner | nav2_smac_planner (Hybrid A*, State Lattice) | 未实现，可插件扩展 |
| OMPL | ROS ompl_interface | 未集成 |
| Nav2 Route | 拓扑路由 | 未实现 |

## 9. 代价地图与规划的关系

### 9.1 代价层级

| 值 | 名称 | 规划行为 |
|----|------|----------|
| 0 | FREE_SPACE | 自由通行 |
| 1–252 | 梯度代价 | 按代价加权（prefer 远离障碍） |
| 253 | INSCRIBED_INFLATED | Theta* 视为阻塞 |
| 254 | LETHAL_OBSTACLE | 不可通行 |
| 255 | NO_INFORMATION | 取决于 `allow_unknown` |

### 9.2 膨胀层的影响

膨胀层（`inflation_layer`）将障碍物向外扩展：

- `inflation_radius`：物理膨胀半径（应 ≥ 机器人半径）
- `cost_scaling_factor`：代价衰减速度

规划器不直接处理机器人几何形状（全局规划假设点机器人或已膨胀），膨胀半径设置不当是路径穿墙的最常见原因。

## 10. 选型决策树

```
需要全局路径？
├── 是 → 地图类型？
│   ├── 已知静态地图
│   │   ├── 需要最短/平滑路径？
│   │   │   ├── 是 → theta_star_planner
│   │   │   └── 否 → navfn_planner（默认）
│   │   ├── 需要确定性？
│   │   │   └── 是 → dijkstra_planner
│   │   └── 大地图？
│   │       └── navfn_planner + use_astar=true
│   └── 部分未知环境
│       └── 任意规划器 + allow_unknown=true
└── 否 → 使用 control 模块局部规划
```

## 11. 开放问题与未来方向

| 方向 | 描述 | 优先级 |
|------|------|--------|
| Hybrid A* | 考虑运动学约束的全局规划 | 高 |
| 分层/拓扑规划 | 大图分治 | 中 |
| 动态重规划 | D* Lite / EPRM | 中 |
| 多机器人协调 | CBS / MAPF | 低 |
| 学习-based 规划 | 神经网络辅助搜索 | 研究阶段 |
| 3D 规划 | 无人机/多层建筑 | 按需 |

## 12. 参考文献

### 经典教材

1. LaValle, S.M. *Planning Algorithms.* Cambridge University Press, 2006.
2. Choset, H. et al. *Principles of Robot Motion: Theory, Algorithms, and Implementations.* MIT Press, 2005.

### 关键论文

3. Hart, P.E., Nilsson, N.J. & Raphael, B. "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." IEEE TSMC, 1968. (A*)
4. Nash, K. et al. "Theta*: Any-Angle Path Planning on Grids." AAAI, 2007.
5. Karaman, S. & Frazzoli, E. "Sampling-based Algorithms for Optimal Motion Planning." IJRR, 2011. (RRT*)
6. Ferguson, D. & Stentz, A. "Field D*: An Interpolation-based Path Planner and Replanner." ISRR, 2005.
7. Dolgov, D. et al. "Practical Search Techniques in Path Planning for Autonomous Driving." AAAI, 2008. (Hybrid A*)
8. Eriksson, L. & Borenstein, J. "GURVEY: A Goal-Directed Navigation System for Mobile Robots." IEEE AES Magazine, 1990. (NavFn 前身)

### 工程参考

9. Navigation2 文档: https://docs.nav2.org/
10. ROS Planning Wiki: http://wiki.ros.org/motion_planning

## 13. 相关文档

- [架构设计](architecture.md)
- [Planning 路径规划指南](guide.md)
- [NavFn 规划器](navfn.md)
- [Dijkstra 规划器](dijkstra.md)
- [Theta* 规划器](theta_star.md)
