# 9. 路径规划算法综述（Survey）

本文从**学术发展史、算法体系、工程实践、影响因素**四个维度，系统综述移动机器人路径规划（Path Planning / Motion Planning）领域，并明确 Autonomy `planning` 模块在其中的定位、能力边界与选型依据。

> 公式推导见 [Planning 指南 · 数学公式](03_math.md)；实现细节见 [架构设计](05_architecture.md) 与各算法子文档。

---

## 9.1 概述：路径规划在导航栈中的位置

移动机器人导航（Navigation）通常分解为四个耦合子问题：

| 子问题 | 英文 | 典型模块 | 频率 |
|--------|------|----------|------|
| 定位 | Localization | `localization` | 10–50 Hz |
| 建图 | Mapping | `map` | 1–10 Hz |
| **路径规划** | **Planning** | **`planning`** | **1–5 Hz** |
| 运动控制 | Control | `control` | 10–50 Hz |

**全局路径规划**（Global Planning）在已知或部分已知环境地图上，计算从起点 $q_s$ 到终点 $q_g$ 的**无碰撞几何路径**；**局部规划**在此基础上考虑动力学约束、实时避障，输出可执行速度指令。Autonomy `planning` 模块专注前者。

```
感知/地图 ──→ Costmap2D ──→ Global Planner ──→ Path ──→ Local Planner ──→ Controller ──→ 底盘
                ↑                    ↑
           static/obstacle      NavFn / Dijkstra / Theta*
           inflation layer
```

---

## 9.2 问题形式化

### 9.2.1 配置空间

移动机器人在平面环境中可建模为 $SE(2)$：位置 $(x,y)$ + 朝向 $\theta$。若仅做几何路径规划（全局层常见简化），可退化为 $\mathbb{R}^2$：

$$
\mathcal{C} = \mathbb{R}^2, \quad
\mathcal{C}_{\mathrm{obs}} \subset \mathcal{C}, \quad
\mathcal{C}_{\mathrm{free}} = \mathcal{C} \setminus \mathcal{C}_{\mathrm{obs}}
$$

给定 $q_s, q_g \in \mathcal{C}_{\mathrm{free}}$，求曲线 $\tau: [0,1] \to \mathcal{C}_{\mathrm{free}}$，使 $\tau(0)=q_s$，$\tau(1)=q_g$。

### 9.2.2 多目标优化

实际规划 rarely 只优化长度，常见复合目标：

$$
J(\tau) = \int_0^1 \Big(
  w_l \|\tau'(s)\|
  + w_c \, c(\tau(s))
  + w_k \|\tau''(s)\|^2
  + w_\theta \|\dot{\theta}(s)\|^2
\Big) ds
$$

| 项 | 物理含义 | 主导算法类型 |
|----|----------|--------------|
| $\|\tau'\|$ | 路径长度 | 图搜索、Theta* |
| $c(\tau)$ | 距障碍代价 | NavFn、代价地图加权 A* |
| $\|\tau''\|^2$ | 曲率/平滑 | CHOMP、SimpleSmoother |
| $\dot{\theta}$ | 朝向变化 | Hybrid A*、局部规划 |

### 9.2.3 栅格离散化（Autonomy 实现）

$$
m_x = \left\lfloor \frac{x - x_0}{\Delta} \right\rfloor, \quad
m_y = \left\lfloor \frac{y - y_0}{\Delta} \right\rfloor
$$

默认 $\Delta = 0.05\,\mathrm{m}$，每个栅格存储 $c_{ij} \in [0,255]$。规划在离散图上进行，输出通过 `mapToWorld` 转回连续坐标。

### 9.2.4 约束类型一览

| 约束类型 | 说明 | 全局规划是否考虑 |
|----------|------|------------------|
| 几何约束 | 不碰撞 | ✅ 所有规划器 |
| 运动学约束 | 最小转弯半径、非完整 | ❌（Hybrid A* 可扩展） |
| 动力学约束 | 速度/加速度限制 | ❌（control 模块） |
| 时间约束 | 动态障碍、截止时间 | 部分（重规划） |
| 拓扑约束 | 单行道、电梯 | ❌（拓扑规划扩展） |

---

## 9.3 发展时间轴

按四个历史阶段分块展示，每阶段内用**卡片网格**排列里程碑（年份 / 名称 / 说明分行），避免文字挤在一行。完整对照见 [§9.3.5 分阶段特征表](#935-分阶段特征表)。

<div class="planning-timeline-v2">

<div class="timeline-era-block era-foundation">
  <div class="timeline-era-header">奠基期 · 1950s–1970s</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1959</div>
      <div class="timeline-milestone-title">Dijkstra</div>
      <div class="timeline-milestone-desc">单源最短路径，图搜索理论奠基</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1968</div>
      <div class="timeline-milestone-title">A*</div>
      <div class="timeline-milestone-desc">启发式搜索，f = g + h 框架</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1970s</div>
      <div class="timeline-milestone-title">C-space</div>
      <div class="timeline-milestone-desc">构型空间概念，运动规划形式化</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1979</div>
      <div class="timeline-milestone-title">可视图法</div>
      <div class="timeline-milestone-desc">Visibility Graph，多边形障碍最短路径</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-reactive">
  <div class="timeline-era-header">势场与反应式 · 1980s–1990s</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1985</div>
      <div class="timeline-milestone-title">APF</div>
      <div class="timeline-milestone-desc">人工势场，引力 + 斥力实时避障</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1990</div>
      <div class="timeline-milestone-title">GURVEY / NavFn</div>
      <div class="timeline-milestone-desc">导航函数，Eikonal 方程栅格求解</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1995</div>
      <div class="timeline-milestone-title">DWA</div>
      <div class="timeline-milestone-desc">动态窗口法，局部速度空间采样</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1998</div>
      <div class="timeline-milestone-title">FMM</div>
      <div class="timeline-milestone-desc">Fast Marching Method，快速行进法</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-sampling">
  <div class="timeline-era-header">采样革命 · 1998–2011</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1998</div>
      <div class="timeline-milestone-title">PRM</div>
      <div class="timeline-milestone-desc">概率路线图，高维 C-space 采样</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1999</div>
      <div class="timeline-milestone-title">RRT</div>
      <div class="timeline-milestone-desc">快速扩展随机树，单查询采样</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2000</div>
      <div class="timeline-milestone-title">RRT-Connect</div>
      <div class="timeline-milestone-desc">双向 RRT，加速连通</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2005</div>
      <div class="timeline-milestone-title">Field D*</div>
      <div class="timeline-milestone-desc">任意角栅格搜索，路径更短</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2007</div>
      <div class="timeline-milestone-title">Theta*</div>
      <div class="timeline-milestone-desc">视线检测任意角 A*，Autonomy 已实现</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2011</div>
      <div class="timeline-milestone-title">RRT*</div>
      <div class="timeline-milestone-desc">渐近最优采样规划</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-engineering">
  <div class="timeline-era-header">工程化与自动驾驶 · 2008–至今</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2008</div>
      <div class="timeline-milestone-title">Hybrid A*</div>
      <div class="timeline-milestone-desc">考虑朝向与运动学约束的栅格 A*</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2010</div>
      <div class="timeline-milestone-title">ROS nav stack</div>
      <div class="timeline-milestone-desc">move_base + navfn 工程化栈</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2014</div>
      <div class="timeline-milestone-title">CHOMP / STOMP</div>
      <div class="timeline-milestone-desc">轨迹优化规划器</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2018</div>
      <div class="timeline-milestone-title">Navigation2</div>
      <div class="timeline-milestone-desc">nav2 插件化架构，行为树导航</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2020</div>
      <div class="timeline-milestone-title">nav2_smac</div>
      <div class="timeline-milestone-desc">Hybrid A* / State Lattice 插件</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2023</div>
      <div class="timeline-milestone-title">学习规划兴起</div>
      <div class="timeline-milestone-desc">Neural Planner、Diffusion Policy 等</div>
    </div>
    <div class="timeline-milestone timeline-milestone-highlight">
      <div class="timeline-milestone-year">2025</div>
      <div class="timeline-milestone-title">Autonomy planning</div>
      <div class="timeline-milestone-desc">NavFn / Dijkstra / Theta* 三插件成熟</div>
    </div>
  </div>
</div>

</div>

### 9.3.5 分阶段特征表

| 阶段 | 年代 | 代表方法 | 核心突破 | 局限 |
|------|------|----------|----------|------|
| 图搜索奠基 | 1959–1979 | Dijkstra, A*, 可视图 | 完备性、最优性理论 | 高维 C-space 维数灾难 |
| 势场反应式 | 1980–1999 | APF, DWA, NavFn | 实时性好、实现简单 | 局部极小、网格依赖 |
| 采样规划 | 1998–2011 | PRM, RRT, RRT* | 高维、复杂拓扑 | 路径不光滑、概率性 |
| 任意角/混合 | 2005–2012 | Field D*, Theta*, Hybrid A* | 路径更短、考虑朝向 | 计算量上升 |
| 工程栈整合 | 2010–至今 | nav2, OMPL, Smac | 插件化、产品级 | 调参复杂 |
| 学习增强 | 2020–至今 | Neural Planner, Diffusion | 大数据泛化 | 安全可解释性不足 |

### 9.3.6 Autonomy 在技术谱系中的位置

```
1959 Dijkstra ──→ 1990 NavFn/GURVEY ──→ 2010 ROS navfn ──→ 2018 nav2_navfn_planner
                                                                    │
                                                              2025 Autonomy
                                                              ├── NavfnPlanner
                                                              ├── DijkstraPlanner
                                                              └── ThetaStarPlanner (2007)
```

Autonomy `planning` 定位于 **nav2 兼容的工程化全局栅格规划层**，尚未覆盖采样规划（RRT*）、运动学规划（Hybrid A*）和学习规划。

---

## 9.4 算法分类全景

### 9.4.1 四维分类法

```
路径规划算法
│
├── 按搜索空间
│   ├── 离散：栅格 / 图 / 拓扑图
│   ├── 连续：样条 / 优化变量
│   └── 混合：Hybrid A*, State Lattice
│
├── 按搜索策略
│   ├── 完备搜索：Dijkstra, A*
│   ├── 启发式搜索：Weighted A*, Theta*
│   ├── 采样搜索：RRT, PRM
│   └── 优化搜索：CHOMP, TrajOpt
│
├── 按时间维度
│   ├── 静态：一次规划
│   ├── 动态：D*, EPRM
│   └── 反应式：DWA, MPC
│
└── 按最优性
    ├── 最优：Dijkstra, A* (可采纳 h)
    ├── 渐近最优：RRT*, BIT*
    ├── 有界次优：Weighted A*
    └── 启发式：Greedy, APF
```

### 9.4.2 方法—特性矩阵（扩展版）

| 方法族 | 代表算法 | 维度 | 完备性 | 最优性 | 路径质量 | 实时性 | 实现难度 |
|--------|----------|------|--------|--------|----------|--------|----------|
| 图搜索 | Dijkstra, A* | 2D 栅格 | 完备 | 最优* | 中（锯齿） | 高 | 低 |
| 导航函数 | NavFn, FMM | 2D 栅格 | 完备 | 近似最优 | 中 | 高 | 低 |
| 任意角 | Theta*, Field D* | 2D 栅格 | 完备 | 近似最优 | 高 | 中 | 中 |
| 可视图 | Visibility | 2D 多边形 | 完备 | 最优 | 高 | 中 | 中 |
| 采样 | RRT, RRT* | 高维 | 概率完备 | 渐近最优† | 低–中 | 中 | 中 |
| 优化 | CHOMP, GPMP | 连续 | 不完备 | 局部最优 | 高 | 低–中 | 高 |
| 混合 | Hybrid A* | SE(2) 栅格 | 完备‡ | 有界次优 | 高 | 中 | 高 |
| 学习 | Neural, Diffusion | — | 不保证 | 不保证 | 变化大 | 推理快 | 很高 |

\* 在离散图上最优；† RRT* 渐近最优；‡ 分辨率完备。

---

## 9.5 图搜索与导航函数（Autonomy 核心）

### 9.5.1 经典图搜索

**Dijkstra**（1959）：非负权图单源最短路径，松弛方程：

$$
d(v) = \min_{(u,v)\in E} \{ d(u) + w(u,v) \}
$$

**A\***（1968）：引入启发式 $f(n)=g(n)+h(n)$，当 $h$ 可采纳（不高估）时保证最优。

| 算法 | 数据结构 | 时间复杂度 | Autonomy 实现 |
|------|----------|-----------|---------------|
| Dijkstra | 二叉堆 | $O((V+E)\log V)$ | NavFn 桶队列 + `DijkstraPlanner` |
| A* | 优先队列 | $O(b^d)$ 最坏 | `NavfnPlanner(use_astar=true)` |
| D* Lite | 增量堆 | 与变化量相关 | ❌ 未实现 |
| LPA* | 类似 D* | 动态重规划 | ❌ 未实现 |

### 9.5.2 NavFn 导航势场

NavFn 近似求解 Eikonal 方程 $\|\nabla\phi\|=F$，用平面波更新 + 桶队列 Dijkstra 反向传播，再梯度跟踪提取路径。详见 [NavFn 规划器](06_navfn.md)。

**与 Fast Marching Method (FMM) 对比**：

| 维度 | NavFn | FMM |
|------|-------|-----|
| 精度 | 平面波近似 | 精确 Eikonal |
| 速度 | 快（桶队列 O(1) 均摊） | 较慢（堆 O(log N)） |
| 工程采用 | ROS/nav2/Autonomy | 学术研究较多 |

### 9.5.3 Theta* 任意角规划

在 A* 扩展时加入 Line-of-Sight 检测，允许祖父节点直连邻居，路径逼近欧氏直线。详见 [Theta* 规划器](08_theta_star.md)。

**Any-Angle 方法谱系**：

| 方法 | 年份 | 核心思想 | Autonomy |
|------|------|----------|----------|
| Theta* | 2007 | 扩展时即时 LOS | ✅ |
| Lazy Theta* | 2010 | 延迟 LOS 验证 | ❌ |
| Block A* | 2011 | 分块 LOS | ❌ |
| Field D* | 2005 | 插值场 + 任意角 | ❌ |
| Anya* | 2013 | 压缩栅格 | ❌ |
| Post-smoothing | — | 栅格路径 + 字符串拉直 | 部分 (`PathSimplifier`) |

---

## 9.6 采样-based 方法

### 9.6.1 主要算法详解

| 算法 | 提出 | 核心机制 | 优点 | 缺点 |
|------|------|----------|------|------|
| **PRM** | 1996 | 随机采样构型 + 连边 | 多查询效率高 | 窄通道采样困难 |
| **RRT** | 1998 | 随机扩展树向目标生长 | 高维、实现简单 | 路径质量差 |
| **RRT-Connect** | 2000 | 双向树对接 | 速度快 | 仍不最优 |
| **RRT\*** | 2011 | 重连 + 代价递减 | 渐近最优 | 收敛慢 |
| **BIT\*** | 2014 | 批量隐式搜索 | 高维渐近最优 | 实现复杂 |
| **FMT\*** | 2014 | 快速行进树 | 低维效率高 | 高维性能下降 |

### 9.6.2 与 Autonomy 的关系

当前 **未集成** OMPL / RRT*。适用场景：

- 机械臂 6+ DOF 规划
- 非凸迷宫、窄缝（采样更鲁棒）
- 高维状态空间

可通过 `GlobalPlanner` 插件接口集成 OMPL 或自定义 RRT 实现。

---

## 9.7 优化-based 方法

### 9.7.1 轨迹优化谱系

| 方法 | 类型 | 优化变量 | 约束处理 |
|------|------|----------|----------|
| CHOMP | 梯度下降 | 离散路点 | 软约束（障碍代价） |
| STOMP | 随机采样优化 | 轨迹系数 | 软约束 |
| TrajOpt | 序列凸优化 | 路点 | 硬/软混合 |
| GPMP2 | 高斯过程 | 连续轨迹 | 概率约束 |
| MPC | 模型预测控制 | 控制序列 | 硬约束 |

### 9.7.2 Autonomy 中的优化：SimpleSmoother

全局搜索后的**后处理**优化（非全局搜索）：

$$
E = w_d \sum_i \|p_i - o_i\|^2 + w_s \sum_i \|p_{i-1} - 2p_i + p_{i+1}\|^2
$$

Gauss-Seidel 迭代更新，带碰撞回退。详见 [数学原理](03_math.md)。

---

## 9.8 势场与反应式方法

| 方法 | 机制 | 优点 | 致命缺陷 |
|------|------|------|----------|
| APF 人工势场 | 引力 + 斥力叠加 | 极简、实时 | **局部极小**（U 形障碍） |
| 调和势场 | 解 $\nabla^2\phi=0$ | 无局部极小 | $O(n^3)$ 求解 |
| NavFn | 平面波 Eikonal 近似 | 快、nav2 验证 | 网格锯齿 |
| DWA | 速度空间采样 | 局部避障好 | 非全局最优 |
| TEB | 弹性带优化 | 考虑时间 | 参数敏感 |

---

## 9.9 动态环境与重规划

| 方法 | 场景 | 机制 | Autonomy |
|------|------|------|----------|
| 周期性重规划 | 障碍缓慢变化 | 定时 `GetPlan()` | ✅ Navigator 层 |
| 路径有效性检查 | 路径被挡 | `IsPathValid()` | ✅ |
| D* / D* Lite | 地图增量变化 | 修复受影响节点 | ❌ |
| EPRM | 动态障碍 | 扩展 PRM | ❌ |
| MPC | 快速动态 | 滚动优化 | control 模块 |

Autonomy 当前采用 **「全局规划 + 路径校验 + 触发重规划」** 策略，而非增量式 D*。

---

## 9.10 鱼骨图：路径规划质量影响因素

路径规划效果不佳时，可从以下六个维度排查。采用**鱼骨图（石川图）** 的「上因 → 鱼头 → 下因」结构，以卡片网格呈现，避免 Mermaid 节点文字截断。

<div class="fishbone-wrap">

<div class="fishbone-label">上层原因（人 · 机 · 法）</div>
<div class="fishbone-grid">
  <div class="fishbone-card">
    <h4>地图与感知</h4>
    <ul>
      <li>分辨率过粗</li>
      <li>膨胀半径不足</li>
      <li>静态层未加载</li>
      <li>障碍层延迟</li>
      <li>未知区策略误设</li>
    </ul>
  </div>
  <div class="fishbone-card">
    <h4>算法与参数</h4>
    <ul>
      <li>规划器选型不当</li>
      <li>tolerance 过小</li>
      <li>allow_unknown 误设</li>
      <li>Theta* 切角穿墙</li>
      <li>A* / Dijkstra 选型</li>
    </ul>
  </div>
  <div class="fishbone-card">
    <h4>机器人模型</h4>
    <ul>
      <li>footprint 不准确</li>
      <li>robot_radius 偏差</li>
      <li>点机器人假设</li>
      <li>运动学约束未建模</li>
    </ul>
  </div>
</div>

<div class="fishbone-head">路径规划质量不佳</div>

<div class="fishbone-label">下层原因（环 · 系 · 果）</div>
<div class="fishbone-grid">
  <div class="fishbone-card fishbone-lower">
    <h4>环境与任务</h4>
    <ul>
      <li>窄通道 / 多障碍</li>
      <li>动态障碍未更新</li>
      <li>多目标路点复杂</li>
      <li>大地图规模过大</li>
    </ul>
  </div>
  <div class="fishbone-card fishbone-lower">
    <h4>系统集成</h4>
    <ul>
      <li>TF 坐标系错误</li>
      <li>costmap 未及时更新</li>
      <li>规划频率过低</li>
      <li>后处理简化失真</li>
    </ul>
  </div>
  <div class="fishbone-card fishbone-lower">
    <h4>下游执行</h4>
    <ul>
      <li>局部规划跟不上</li>
      <li>控制器跟踪偏差</li>
      <li>航点过密或过疏</li>
    </ul>
  </div>
</div>

</div>

### 9.10.1 因素—对策速查表

| 鱼骨分支 | 典型症状 | 优先检查项 | 建议对策 |
|----------|----------|-----------|----------|
| 地图与感知 | 路径穿墙 | `inflation_radius`, 障碍层话题 | 增大膨胀，确认 `/scan` 有数据 |
| 算法与参数 | 找不到路径 | `tolerance`, goal 是否在膨胀区 | 增大 tolerance，换可达目标 |
| 机器人模型 | 身体蹭障碍 | `footprint` vs `robot_radius` | 使用 footprint 模式 |
| 环境任务 | 窄缝失败 | 8-连通切角 | Theta* 改 4-连通或换 NavFn |
| 系统与集成 | 路径漂移 | `frame_id`, TF 树 | 统一到 costmap global frame |
| 下游执行 | 走不到位 | 航点间距、控制频率 | 禁用 DP 简化，检查 controller |

---

## 9.11 分层规划架构

### 9.11.1 经典三层架构

采用全宽纵向分层示意图，自上而下展示数据流；每层三列排布（层级标识 · 职责 · 输入输出），避免文字重叠。

<div class="nav-stack-diagram nav-stack-wide">

<div class="nav-stack-inputs-bar">
  <div class="nav-input-card">
    <span class="nav-input-label">感知</span>
    <span class="nav-input-text">激光 / 点云 / 深度</span>
    <span class="nav-input-arrow">→</span>
    <span class="nav-input-target">obstacle_layer</span>
  </div>
  <div class="nav-input-card">
    <span class="nav-input-label">定位</span>
    <span class="nav-input-text">SLAM / AMCL</span>
    <span class="nav-input-arrow">→</span>
    <span class="nav-input-target">TF map→odom</span>
  </div>
  <div class="nav-input-card">
    <span class="nav-input-label">地图</span>
    <span class="nav-input-text">先验栅格 / SLAM 图</span>
    <span class="nav-input-arrow">→</span>
    <span class="nav-input-target">static_layer</span>
  </div>
</div>

<div class="nav-costmap-banner">
  <span>三路输入经融合与膨胀后，生成</span>
  <strong>Costmap2D</strong>
  <span class="nav-costmap-detail">（static_layer + obstacle_layer + inflation_layer）</span>
  <span class="nav-costmap-arrow">↓ 作为 L2 规划环境模型</span>
</div>

<div class="nav-stack-pipeline">

  <div class="nav-layer-row nav-layer-behavior">
    <div class="nav-layer-meta">
      <span class="nav-layer-badge">L1</span>
      <div class="nav-layer-title">行为层</div>
      <div class="nav-layer-sub">autonomy / navigator</div>
      <span class="nav-meta-freq">事件驱动</span>
    </div>
    <div class="nav-layer-body">
      <div class="nav-body-block">
        <div class="nav-body-label">核心职责</div>
        <ul>
          <li>解析用户导航意图，维护任务状态机</li>
          <li>向规划层下发起点 / 终点与恢复策略</li>
          <li>监测路径失效并触发重规划</li>
        </ul>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">关键组件</div>
        <div class="nav-chip-list">
          <span class="nav-chip">行为树 BT</span>
          <span class="nav-chip">compute_path_to_pose</span>
          <span class="nav-chip">Recovery</span>
        </div>
      </div>
    </div>
    <div class="nav-layer-io">
      <div class="nav-io-block nav-io-out">
        <div class="nav-io-heading">输出 → L2</div>
        <ul class="nav-io-list">
          <li>目标位姿 <code>PoseStamped</code></li>
          <li>重规划 / 取消信号</li>
        </ul>
      </div>
    </div>
  </div>

  <div class="nav-pipe">
    <span class="nav-pipe-label">导航目标 + 重规划触发</span>
  </div>

  <div class="nav-layer-row nav-layer-global">
    <div class="nav-layer-meta">
      <span class="nav-layer-badge">L2</span>
      <div class="nav-layer-title">全局规划层</div>
      <div class="nav-layer-sub">autonomy / planning</div>
      <span class="nav-meta-freq">1–5 Hz</span>
    </div>
    <div class="nav-layer-body">
      <div class="nav-body-block">
        <div class="nav-body-label">核心职责</div>
        <ul>
          <li>在全局代价地图上搜索无碰撞几何路径</li>
          <li>周期性或按需校验已有路径是否仍可行</li>
          <li>将栅格路径转换为可发布的航点序列</li>
        </ul>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">规划器插件</div>
        <div class="nav-chip-list">
          <span class="nav-chip">navfn_planner</span>
          <span class="nav-chip">dijkstra_planner</span>
          <span class="nav-chip">theta_star_planner</span>
        </div>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">服务入口</div>
        <div class="nav-chip-list">
          <span class="nav-chip">PlannerServer</span>
          <span class="nav-chip">GetPlan()</span>
          <span class="nav-chip">IsPathValid()</span>
        </div>
      </div>
    </div>
    <div class="nav-layer-io">
      <div class="nav-io-block nav-io-in">
        <div class="nav-io-heading">输入</div>
        <ul class="nav-io-list">
          <li><code>Costmap2D</code> 全局地图</li>
          <li>起点 / 终点位姿</li>
          <li>规划器 ID 与参数</li>
        </ul>
      </div>
      <div class="nav-io-block nav-io-out">
        <div class="nav-io-heading">输出 → L3</div>
        <ul class="nav-io-list">
          <li><code>planning_msgs::Path</code></li>
          <li>含位姿的航点序列</li>
        </ul>
      </div>
    </div>
  </div>

  <div class="nav-pipe">
    <span class="nav-pipe-label">planning_msgs::Path（全局路径）</span>
  </div>

  <div class="nav-layer-row nav-layer-local">
    <div class="nav-layer-meta">
      <span class="nav-layer-badge">L3</span>
      <div class="nav-layer-title">局部规划与控制层</div>
      <div class="nav-layer-sub">autonomy / control</div>
      <span class="nav-meta-freq">10–50 Hz</span>
    </div>
    <div class="nav-layer-body">
      <div class="nav-body-block">
        <div class="nav-body-label">核心职责</div>
        <ul>
          <li>跟踪全局路径，生成短时可行轨迹</li>
          <li>结合局部地图做实时动态避障</li>
          <li>输出满足运动学约束的速度指令</li>
        </ul>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">典型算法</div>
        <div class="nav-chip-list">
          <span class="nav-chip">DWA / DWB</span>
          <span class="nav-chip">TEB</span>
          <span class="nav-chip">Pure Pursuit</span>
          <span class="nav-chip">MPPI</span>
        </div>
      </div>
    </div>
    <div class="nav-layer-io">
      <div class="nav-io-block nav-io-in">
        <div class="nav-io-heading">输入</div>
        <ul class="nav-io-list">
          <li>L2 下发的全局 <code>Path</code></li>
          <li>局部代价地图 / 障碍物</li>
          <li>当前里程计与 TF</li>
        </ul>
      </div>
      <div class="nav-io-block nav-io-out">
        <div class="nav-io-heading">输出 → 底盘</div>
        <ul class="nav-io-list">
          <li><code>cmd_vel</code> 线速度 / 角速度</li>
        </ul>
      </div>
    </div>
  </div>

  <div class="nav-pipe">
    <span class="nav-pipe-label">cmd_vel（速度指令）</span>
  </div>

  <div class="nav-robot-bar">
    <span class="nav-robot-icon">▣</span>
    机器人底盘执行运动
  </div>

</div>

</div>

### 9.11.2 Autonomy 导航栈映射

| 层级 | 模块 | 关键接口 | 典型频率 |
|------|------|----------|----------|
| 行为 | `navigator` | BT 节点、`compute_path_to_pose` | 事件驱动 |
| 全局规划 | `planning` | `GetPlan()`, `IsPathValid()` | 1–5 Hz |
| 局部规划 | `control` | 速度指令生成 | 10–50 Hz |
| 地图 | `map/costmap_2d` | `/global_costmap` | 5 Hz |
| 定位 | `localization` | TF `map→odom` | 10–30 Hz |

---

## 9.12 Autonomy 内置规划器深度对比

### 9.12.1 四维雷达图（文字版）

| 维度 (1–5) | NavFn Dijkstra | NavFn A* | DijkstraPlanner | ThetaStarPlanner |
|------------|----------------|----------|-----------------|------------------|
| 计算速度 | ★★★★★ | ★★★★ | ★★★★★ | ★★★ |
| 路径长度 | ★★★ | ★★★ | ★★★ | ★★★★★ |
| 路径平滑 | ★★★ | ★★★ | ★★★ | ★★★★ |
| 确定性 | ★★★★★ | ★★★★★ | ★★★★★ | ★★★★★ |
| 窄通道 | ★★★★ | ★★★★ | ★★★★ | ★★★ (8-连通) |
| 大地图 | ★★★ | ★★★★ | ★★★ | ★★★ |
| 调参难度 | ★★ | ★★ | ★★ | ★★★ |

### 9.12.2 量化对比（典型 500×500 栅格地图）

| 指标 | NavFn Dijkstra | NavFn A* | Theta* |
|------|----------------|----------|--------|
| 首次规划延迟 | 20–80 ms | 10–50 ms | 50–200 ms |
| 路径航点数 | 多（梯度跟踪） | 多 | 少（直线段） |
| 路径/直线比 | 1.2–1.5× | 1.2–1.5× | 1.0–1.1× |
| 内存 | O(N) 势场 | O(N) 势场 | O(N) g+parent |

\* 实测值因硬件、地图、起终点而异，仅供量级参考。

---

## 9.13 代价地图与规划的耦合

### 9.13.1 代价值语义

| 值 | 常量 | NavFn 行为 | Theta* 行为 |
|----|------|-----------|-------------|
| 0 | FREE_SPACE | 通行代价 50 | 可通行 |
| 1–252 | 梯度代价 | $50+0.8c$ | 加权 $\tau(n)$ |
| 253 | INSCRIBED | 高代价 | **阻塞** |
| 254 | LETHAL | 不可通行 | **阻塞** |
| 255 | UNKNOWN | 取决于 `allow_unknown` | 取决于配置 |

### 9.13.2 膨胀层数学

膨胀代价随距离衰减：

$$
c(d) = \begin{cases}
254 & d \leq r_{\text{inscribed}} \\
253 & d \leq r_{\text{inflation}} \\
(253 - s_{\mathrm{infl}}) \cdot e^{-\lambda (d - r_{\text{inscribed}})} & \mathrm{otherwise}
\end{cases}
$$

其中 $s_{\mathrm{infl}}$ 对应膨胀层内部使用的缩放系数 `scale`。

**关键约束**：$r_{\text{inflation}} \geq r_{\text{robot}}$，否则全局规划假设点机器人时会穿墙。

### 9.13.3 层叠顺序（`planner.lua` 默认）

```
static_layer → obstacle_layer → inflation_layer
     ↓               ↓                ↓
  先验地图        实时激光          安全缓冲
```

---

## 9.14 场景选型矩阵

| 场景 | 地图 | 通道 | 推荐规划器 | 关键配置 |
|------|------|------|-----------|----------|
| 室内仓储 AGV | 静态 SLAM | 宽 | `navfn_planner` | 默认 |
| 办公室服务机器人 | 静态 + 激光 | 中 | `theta_star_planner` | `how_many_corners=8` |
| 窄走廊 | 静态 | 窄 | `navfn_planner` 或 Theta* 4-连通 | `how_many_corners=4` |
| 医院/商场 | 大范围 | 混合 | `navfn` + `use_astar=true` | 大地图提前终止 |
| 调试/回归测试 | 任意 | — | `dijkstra_planner` | 确定性输出 |
| 探索未知区 | 部分未知 | — | 任意 | `allow_unknown=true` |
| 动态人群 | 实时更新 | — | NavFn + 高频重规划 | 缩短 `costmap_update_timeout` |
| 泊车/倒车 | SE(2) | — | ❌ 需 Hybrid A* | 插件扩展 |

---

## 9.15 选型决策树

```
需要全局几何路径？
├── 否 → control 模块局部规划 / DWA / TEB
└── 是
    ├── 环境地图是否就绪？
    │   ├── 否 → 先建图 / 定位
    │   └── 是
    │       ├── 是否需要任意角短路径？
    │       │   ├── 是 → theta_star_planner
    │       │   │       ├── 窄通道 → how_many_corners=4
    │       │   │       └── 开阔 → how_many_corners=8
    │       │   └── 否
    │       │       ├── 需要确定性输出？ → dijkstra_planner
    │       │       ├── 大地图 ( > 2048² )？ → navfn + use_astar=true
    │       │       └── 默认 → navfn_planner
    │       └── 目标在膨胀区边缘？
    │           └── 增大 tolerance (0.1 → 0.3 m)
    └── 需要 SE(2) 运动学约束？
        └── 扩展 Hybrid A* 插件（未内置）
```

---

## 9.16 业界生态对比

### 9.16.1 ROS / nav2 规划器生态

| 规划器 | 包名 | 算法 | Autonomy 状态 |
|--------|------|------|---------------|
| NavFn | nav2_navfn_planner | 导航势场 | ✅ 移植 |
| Dijkstra | nav2_navfn (mode) | Dijkstra | ✅ 独立插件 |
| Theta* | nav2_theta_star_planner | Theta* | ✅ 移植 |
| Smac 2D | nav2_smac_planner | A* / 平滑 | ❌ 可插件扩展 |
| Smac Hybrid | nav2_smac_planner | Hybrid A* | ❌ |
| Smac Lattice | nav2_smac_planner | 状态格栅 | ❌ |
| OMPL | ompl_interface | RRT/PRM 等 | ❌ |
| Nav2 Route | nav2_route | 拓扑路由 | ❌ |

### 9.16.2 商业/开源框架

| 框架 | 全局规划方案 | 特点 |
|------|-------------|------|
| Navigation2 | 插件式多种 | ROS 2 事实标准 |
| Autonomy | NavFn/Dijkstra/Theta* | 对齐 nav2，autolink 通信 |
| Move Base (ROS 1) | navfn/global_planner | legacy |
| Tesla FSD / Waymo | 专有 + 学习 | 不公开 |
| Apollo | Open Space + EM | 自动驾驶 |

---

## 9.17 开放问题与未来方向

### 9.17.1 技术演进路线图（展望）

<div class="roadmap-steps">
  <div class="roadmap-step roadmap-current">
    <strong>2025</strong>
    <span>Autonomy<br/>栅格三剑客成熟</span>
  </div>
  <div class="roadmap-step">
    <strong>2026</strong>
    <span>Hybrid A*<br/>拓扑路由层</span>
  </div>
  <div class="roadmap-step">
    <strong>2027</strong>
    <span>D* Lite<br/>增量重规划</span>
  </div>
  <div class="roadmap-step">
    <strong>2028</strong>
    <span>学习辅助<br/>启发式 A*</span>
  </div>
  <div class="roadmap-step">
    <strong>2029+</strong>
    <span>端到端<br/>Nav Policy</span>
  </div>
</div>

### 9.17.2 优先级路线图

| 优先级 | 方向 | 描述 | 预期收益 |
|--------|------|------|----------|
| P0 | Hybrid A* 插件 | SE(2) 泊车、倒车 | 覆盖非完整机器人 |
| P0 | 拓扑路由层 | 大图分层 | 长距离效率 |
| P1 | D* Lite 重规划 | 增量修复 | 动态环境效率 |
| P1 | OMPL 集成 | RRT* / PRM | 高维扩展 |
| P2 | 学习启发式 | 神经网络 $h(n)$ | 搜索加速 |
| P2 | 多机 MAPF | CBS / ECBS | 仓储多车 |
| P3 | 3D 规划 | 无人机 / 多层 | 新场景 |

---

## 9.18 关键术语表

| 术语 | 英文 | 解释 |
|------|------|------|
| 完备性 | Completeness | 若解存在则算法必能找到 |
| 最优性 | Optimality | 找到代价最小的解 |
| 可采纳启发式 | Admissible Heuristic | $h(n) \leq h^*(n)$，保证 A* 最优 |
| 任意角路径 | Any-Angle Path | 不限于栅格方向的直线段组合 |
| 导航函数 | Navigation Function | 全空间单调指向目标的势场 |
| 膨胀 | Inflation | 障碍周围安全缓冲区 |
| 重规划 | Replanning | 环境变化后重新计算路径 |

---

## 9.19 参考文献

### 9.19.1 教材

1. [LaValle, S.M. *Planning Algorithms.* Cambridge, 2006.](https://lavalle.pl/planning/)
2. [Choset, H. et al. *Principles of Robot Motion.* MIT Press, 2005.](https://mitpress.mit.edu/9780262033275/principles-of-robot-motion/)
3. [Siegwart, R. et al. *Autonomous Mobile Robots.* MIT Press, 2011.](https://mitpress.mit.edu/9780262015356/autonomous-mobile-robots/)

### 9.19.2 里程碑论文

| 年份 | 论文 | 贡献 |
|------|------|------|
| 1959 | [Dijkstra, "A note on two problems in connexion with graphs"](https://doi.org/10.1007/BF01386390) | 最短路径 |
| 1968 | [Hart et al., "A Formal Basis for the Heuristic Determination of Minimum Cost Paths"](https://doi.org/10.1109/TSSC.1968.300136) | A* |
| 1985 | [Khatib, "Real-Time Obstacle Avoidance for Manipulators and Mobile Robots"](https://doi.org/10.1177/027836498600500106) | 人工势场 |
| 1990 | [Eriksson & Borenstein, "The GURVEY: An Autonomous Navigation Algorithm Developed for a Hospital Guidance Robot"](https://ieeexplore.ieee.org/document/67314) | GURVEY/NavFn 前身 |
| 1998 | [Kavraki et al., "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces"](https://doi.org/10.1109/70.508439) | PRM |
| 1998 | [LaValle, "Rapidly-Exploring Random Trees: A New Tool for Path Planning"](https://lavalle.pl/rrtpubs.html) | RRT |
| 2005 | [Ferguson & Stentz, "The Field D* Algorithm for Improved Path Planning and Replanning in Uniform and Non-Uniform Cost Environments"](https://www.ri.cmu.edu/publications/the-field-d-algorithm-for-improved-path-planning-and-replanning-in-uniform-and-non-uniform-cost-environments/) | Field D* |
| 2007 | [Nash et al., "Theta*: Any-Angle Path Planning on Grids"](https://cdn.aaai.org/AAAI/2007/AAAI07-187.pdf) | Theta* |
| 2008 | [Dolgov et al., "Practical Search Techniques in Path Planning for Autonomous Driving"](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf) | Hybrid A* |
| 2011 | [Karaman & Frazzoli, "Sampling-based Algorithms for Optimal Motion Planning"](https://doi.org/10.1177/0278364911406761) | RRT* |

### 9.19.3 工程文档

- [Navigation2 Planner Server](https://docs.nav2.org/configuration/packages/configuring-planner-server.html)
- [nav2_navfn_planner](https://github.com/ros-navigation/navigation2/tree/main/nav2_navfn_planner)
- [nav2_theta_star_planner](https://github.com/ros-navigation/navigation2/tree/main/nav2_theta_star_planner)

---

## 9.20 相关文档

- [Planning 路径规划指南](00_guide.md)
- [架构设计](05_architecture.md)
- [NavFn 规划器](06_navfn.md)
- [Dijkstra 规划器](07_dijkstra.md)
- [Theta* 规划器](08_theta_star.md)
