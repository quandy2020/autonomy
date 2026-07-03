# 6. 路径规划算法综述（Survey）

> **本文范围**：算法谱系、Autonomy 能力边界与**选型依据**。  
> 形式化与代价地图见 [§0 指南](00_guide.md)；模块架构见 [§1](01_architecture.md)；各规划器推导见 [§3–§5](03_navfn.md)。

---

## 6.1 综述定位

| 维度 | 本文 | 其他文档 |
|------|------|----------|
| 问题定义 $J(\tau)$、栅格离散 | 不展开 | [§0.3](00_guide.md#03-问题形式化) |
| 导航栈分层、GetPlan 时序 | 不展开 | [§1](01_architecture.md)、[Navigator 综述](../16_Navigator/09_survey.md) |
| NavFn / Theta* 公式与伪代码 | 摘要 + 链接 | [§2 总览](02_planner_algorithms.md) · [§3–§5](03_navfn.md) |
| 算法历史、分类、选型 | **本文** | — |
| 几何路径 vs 轨迹、局部时空联合 | 摘要 | [Control 轨迹规划综述 §6.2.4](../09_Control/06_survey.md#624-几何路径轨迹与局部轨迹规划) |

**建议阅读顺序**

| 角色 | 路径 |
|------|------|
| 选型 / 集成 | §6.6 内置规划器 → §6.6.3 场景矩阵 → §6.7 排错 |
| 算法研发 | §6.3 分类 → §6.4 时间轴 → §6.5 算法族 → [§2](02_planner_algorithms.md) → §3–§5 |
| 背景调研 | §6.4 → §6.5 → §6.8 业界生态 → §6.10 参考文献 |

---

## 6.2 Autonomy 能力边界

Autonomy `planning` 是 **nav2 兼容的全局栅格规划层**：在 Costmap2D 上输出几何路径，不负责局部避障与速度跟踪（见 `control`）。

```
1959 Dijkstra → 1990 NavFn → 2010 ROS navfn → 2018 nav2_navfn_planner → 2025 Autonomy
                                                                              ├── NavfnPlanner
                                                                              ├── DijkstraPlanner
                                                                              └── ThetaStarPlanner
```

| 能力 | 状态 | 说明 |
|------|------|------|
| 栅格势场 / Dijkstra / A* | ✅ | `navfn_planner`、`dijkstra_planner` |
| 任意角 Theta* | ✅ | `theta_star_planner` |
| 路径简化 / 平滑 | ✅ | `PathSimplifier`、`SimpleSmoother`（后处理） |
| 重规划 / 路径校验 | ✅ | Navigator + `IsPathValid()` |
| Hybrid A* / SE(2) | ❌ | 需插件扩展 |
| 采样规划 RRT* / OMPL | ❌ | 高维 C-space |
| 增量 D* Lite | ❌ | 动态地图增量修复 |
| 学习 / 端到端规划 | ❌ | 研究向 |

---

## 6.3 算法分类全景

### 6.3.1 四维分类法

```
路径规划算法
├── 搜索空间：离散（栅格/图）· 连续（样条/优化）· 混合（Hybrid A*）
├── 搜索策略：完备（Dijkstra）· 启发式（A*, Theta*）· 采样（RRT）· 优化（CHOMP）
├── 时间维度：静态 · 动态（D*）· 反应式（DWA, MPC）
└── 最优性：最优 · 渐近最优（RRT*）· 有界次优 · 启发式
```

### 6.3.2 方法—特性矩阵

| 方法族 | 代表 | 完备性 | 最优性 | 路径质量 | 实时性 | Autonomy |
|--------|------|--------|--------|----------|--------|----------|
| 图搜索 | Dijkstra, A* | 完备 | 离散最优* | 中（锯齿） | 高 | ✅ |
| 导航函数 | NavFn, FMM | 完备 | 近似 | 中 | 高 | ✅ NavFn |
| 任意角 | Theta*, Field D* | 完备 | 近似 | 高 | 中 | ✅ Theta* |
| 采样 | RRT, RRT* | 概率完备 | 渐近† | 低–中 | 中 | ❌ |
| 优化 | CHOMP, TrajOpt | 不完备 | 局部 | 高 | 低–中 | 后处理 only |
| 混合 | Hybrid A* | 分辨率完备‡ | 有界次优 | 高 | 中 | ❌ |

\* 离散图上最优；† RRT* 渐近最优；‡ 分辨率完备。

---

## 6.4 发展时间轴

按四个历史阶段分块展示里程碑；完整对照见 [§6.4.1](#641-分阶段特征表)。

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

### 6.4.1 分阶段特征表

| 阶段 | 年代 | 代表方法 | 核心突破 | 局限 |
|------|------|----------|----------|------|
| 图搜索奠基 | 1959–1979 | Dijkstra, A*, 可视图 | 完备性、最优性理论 | 高维维数灾难 |
| 势场反应式 | 1980–1999 | APF, DWA, NavFn | 实时、实现简单 | 局部极小、网格依赖 |
| 采样规划 | 1998–2011 | PRM, RRT, RRT* | 高维、复杂拓扑 | 路径不光滑、概率性 |
| 任意角/混合 | 2005–2012 | Field D*, Theta*, Hybrid A* | 路径更短、考虑朝向 | 计算量上升 |
| 工程栈整合 | 2010–至今 | nav2, OMPL, Smac | 插件化、产品级 | 调参复杂 |
| 学习增强 | 2020–至今 | Neural, Diffusion | 大数据泛化 | 安全可解释性不足 |

---

## 6.5 算法族速览

以下各节为**摘要**；Autonomy 已实现的三类见 §6.6，完整推导见对应子文档。

### 6.5.1 经典图搜索

Dijkstra（1959）：$d(v)=\min_{(u,v)\in E}\{d(u)+w(u,v)\}$。A*（1968）：$f(n)=g(n)+h(n)$，$h$ 可采纳时离散最优。

| 算法 | 复杂度（典型） | Autonomy |
|------|----------------|----------|
| Dijkstra | $O((V+E)\log V)$ | `DijkstraPlanner`、NavFn 桶队列 |
| A* | $O(b^d)$ 最坏 | `navfn_planner` + `use_astar=true` |
| D* Lite / LPA* | 与变化量相关 | ❌ |

详见 [Dijkstra §4](04_dijkstra.md)、[NavFn §4](03_navfn.md)。

### 6.5.2 NavFn 导航势场

近似 Eikonal $\|\nabla\phi\|=F$：自 $q_s$ 播种、桶队列传播，再自 $q_g$ 梯度跟踪。与 FMM 相比：NavFn 用平面波近似，均摊 $O(1)$ 入队，nav2/Autonomy 默认采用。

详见 [NavFn 规划器](03_navfn.md)。

### 6.5.3 Theta* 任意角规划

A* 扩展时做 Line-of-Sight，允许祖父节点直连，路径逼近欧氏直线。Any-Angle 谱系中 Autonomy 实现 Theta*；`PathSimplifier` 提供后处理拉直。

| 方法 | Autonomy |
|------|----------|
| Theta* (2007) | ✅ |
| Lazy Theta*, Field D*, Anya* | ❌ |

详见 [Theta* 规划器](05_theta_star.md)。

### 6.5.4 采样、优化与反应式（未内置）

| 族 | 代表 | 适用 | Autonomy | 深入阅读 |
|----|------|------|----------|----------|
| 采样 | PRM, RRT, RRT* | 高维、窄缝迷宫 | ❌ 可经 `GlobalPlanner` 插件接 OMPL | Planning 全局 |
| 优化 | CHOMP, TrajOpt | 轨迹平滑、约束 | 后处理 `SimpleSmoother`（[§0.6](00_guide.md#06-路径后处理)） | [Control §6.7.10–§6.7.11](../09_Control/06_survey.md#6710-chomp20092013) |
| 反应式 | APF, DWA, DWB | 局部避障、滚动采样 | `control` 模块 | [Control §6.7.4](../09_Control/06_survey.md#674-dwa-dwb1995-2010) |
| 时空联合 | TEB, NMPC, MPPI | 动态障碍、显式 $\Delta t$ | `control`（第三方/待集成） | [Control §6.6.4](../09_Control/06_survey.md#664-时空联合轨迹规划) |

**CHOMP / TrajOpt 在栈中的两种角色**：(1) Planning 侧 **Path 几何平滑**（Autonomy `SimpleSmoother` 为轻量替代）；(2) Control 侧 **局部轨迹优化**（含障碍势场、与 TEB/MPPI 选型对照见 Control 综述）。

### 6.5.5 动态环境与重规划

| 策略 | Autonomy |
|------|----------|
| 周期性 / 失效触发 `GetPlan()` | ✅ Navigator |
| `IsPathValid()` | ✅ |
| D* Lite 增量修复 | ❌ |

当前策略：**全局规划 + 路径校验 + 触发重规划**，非增量 D*。

**与 Control 的分工**：Planning 输出 **无时间** 的几何 `Path`；局部 **轨迹** $\tau(t)$、**时空联合**（TEB、MPC、MPPI rollout）与 **time-scaling** 均在 `control` 层完成。全局侧 `SimpleSmoother` 仅做几何后处理，不等价于 TEB/NMPC 的时间分配。详见 [Control 综述 §6.2.4–§6.6.4](../09_Control/06_survey.md#624-几何路径轨迹与局部轨迹规划)。

## 6.6 Autonomy 内置规划器选型

本节是工程读者**最常用**部分：对比三插件并给出场景与决策树。

### 6.6.1 定性对比

| 维度 (1–5) | NavFn Dijkstra | NavFn A* | DijkstraPlanner | ThetaStarPlanner |
|------------|----------------|----------|-----------------|------------------|
| 计算速度 | ★★★★★ | ★★★★ | ★★★★★ | ★★★ |
| 路径长度 | ★★★ | ★★★ | ★★★ | ★★★★★ |
| 路径平滑 | ★★★ | ★★★ | ★★★ | ★★★★ |
| 确定性 | ★★★★★ | ★★★★★ | ★★★★★ | ★★★★★ |
| 窄通道 | ★★★★ | ★★★★ | ★★★★ | ★★★ (8-连通) |
| 大地图 | ★★★ | ★★★★ | ★★★ | ★★★ |
| 调参难度 | ★★ | ★★ | ★★ | ★★★ |

### 6.6.2 量化参考（500×500 栅格，量级）

| 指标 | NavFn Dijkstra | NavFn A* | Theta* |
|------|----------------|----------|--------|
| 首次规划延迟 | 20–80 ms | 10–50 ms | 50–200 ms |
| 路径航点数 | 多 | 多 | 少 |
| 路径/直线比 | 1.2–1.5× | 1.2–1.5× | 1.0–1.1× |
| 内存 | $O(N)$ 势场 | $O(N)$ 势场 | $O(N)$ g+parent |

\* 随硬件、地图、起终点变化，仅供选型量级。

### 6.6.3 场景选型矩阵

| 场景 | 地图 | 通道 | 推荐规划器 | 关键配置 |
|------|------|------|-----------|----------|
| 室内仓储 AGV | 静态 SLAM | 宽 | `navfn_planner` | 默认 |
| 办公室服务机器人 | 静态 + 激光 | 中 | `theta_star_planner` | `how_many_corners=8` |
| 窄走廊 | 静态 | 窄 | `navfn_planner` 或 Theta* 4-连通 | `how_many_corners=4` |
| 医院/商场 | 大范围 | 混合 | `navfn` + `use_astar=true` | 大地图提前终止 |
| 调试/回归测试 | 任意 | — | `dijkstra_planner` | 确定性输出 |
| 探索未知区 | 部分未知 | — | 任意 | `allow_unknown=true` |
| 动态人群 | 实时更新 | — | NavFn + 高频重规划 | 缩短 costmap 超时 |
| 泊车/倒车 SE(2) | — | — | ❌ Hybrid A* | 插件扩展 |

### 6.6.4 选型决策树

```
需要全局几何路径？
├── 否 → control 局部规划（DWA / TEB / MPPI）
└── 是
    ├── 地图未就绪？ → 先建图 / 定位
    └── 是
        ├── 要任意角短路径？
        │   ├── 是 → theta_star_planner
        │   │       ├── 窄通道 → how_many_corners=4
        │   │       └── 开阔 → how_many_corners=8
        │   └── 否
        │       ├── 要确定性？ → dijkstra_planner
        │       ├── 大地图 (>2048²)？ → navfn + use_astar=true
        │       └── 默认 → navfn_planner
        └── 目标在膨胀区边缘？ → 增大 tolerance (0.1→0.3 m)
```

---

## 6.7 工程质量与排错

### 6.7.1 代价地图与规划

规划质量高度依赖 Costmap2D：代价值语义、膨胀半径、图层顺序直接影响 NavFn / Theta* 行为。  
**不重复展开** — 见 [§0.4 代价地图](00_guide.md#04-代价地图)、[Map · Costmap2D](../07_Map/06_costmap2d.md)、[架构 §1.2.1](01_architecture.md#121-地图层-costmap2dwrapper)。

关键约束：$r_{\mathrm{inflation}} \geq r_{\mathrm{robot}}$，否则点机器人假设下路径可能穿墙。

### 6.7.2 质量影响因素（鱼骨图）

路径规划效果不佳时，从「地图 · 算法 · 模型 · 环境 · 集成 · 下游」六维排查：

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

### 6.7.3 因素—对策速查表

| 分支 | 典型症状 | 优先检查 | 对策 |
|------|----------|----------|------|
| 地图与感知 | 路径穿墙 | `inflation_radius`、障碍话题 | 增大膨胀，确认 `/scan` |
| 算法与参数 | 找不到路径 | `tolerance`、goal 在膨胀区 | 增大 tolerance，换目标 |
| 机器人模型 | 身体蹭障碍 | `footprint` vs `robot_radius` | 使用 footprint |
| 环境任务 | 窄缝失败 | 8-连通切角 | Theta* 改 4-连通或 NavFn |
| 系统与集成 | 路径漂移 | `frame_id`、TF | 统一到 costmap global frame |
| 下游执行 | 走不到位 | 航点间距、控制频率 | 禁用 DP 简化，查 controller |

更多排错见 [§0.8 故障排查](00_guide.md#08-故障排查)。

---

## 6.8 业界生态

### 6.8.1 ROS / nav2 规划器

| 规划器 | 包 | Autonomy |
|--------|-----|----------|
| NavFn | nav2_navfn_planner | ✅ |
| Dijkstra | nav2_navfn (mode) | ✅ 独立插件 |
| Theta* | nav2_theta_star_planner | ✅ |
| Smac 2D / Hybrid / Lattice | nav2_smac_planner | ❌ 可插件扩展 |
| OMPL | ompl_interface | ❌ |
| Nav2 Route | nav2_route | ❌ |

### 6.8.2 其他框架

| 框架 | 全局规划 | 特点 |
|------|----------|------|
| Navigation2 | 插件多种 | ROS 2 事实标准 |
| Autonomy | NavFn / Dijkstra / Theta* | 对齐 nav2，autolink |
| Move Base (ROS 1) | navfn | legacy |
| Apollo | Open Space + EM | 自动驾驶 |

---

## 6.9 开放问题与路线图

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

| 优先级 | 方向 | 预期收益 |
|--------|------|----------|
| P0 | Hybrid A* 插件 | SE(2) 泊车、倒车 |
| P0 | 拓扑路由层 | 长距离大图效率 |
| P1 | D* Lite | 动态环境增量修复 |
| P1 | OMPL 集成 | 高维扩展 |
| P2 | 学习启发式 | 搜索加速 |
| P2 | 多机 MAPF | 仓储多车 |

---

## 6.10 术语表

| 术语 | 解释 |
|------|------|
| 完备性 | 若解存在则算法必能找到 |
| 可采纳启发式 | $h(n)\leq h^*(n)$，保证 A* 离散最优 |
| 任意角路径 | 不限于栅格方向的直线段组合 |
| 导航函数 | 全空间单调指向目标的势场 |
| 膨胀 | 障碍周围安全缓冲区 |
| 重规划 | 环境变化后重新计算路径 |
| 几何路径 | 无时间参数的 $SE(2)$ 航点序列；Planning 产出 |
| 局部轨迹 | 含 $t$ 或控制序列的 $\tau(t)$；Control 产出（见 [Control 术语表 §6.17](../09_Control/06_survey.md#617-术语表)） |

---

## 6.11 参考文献

**教材**

1. [LaValle, *Planning Algorithms* (2006)](https://lavalle.pl/planning/)
2. [Choset et al., *Principles of Robot Motion* (2005)](https://mitpress.mit.edu/9780262033275/principles-of-robot-motion/)

**里程碑论文**

| 年份 | 论文 | 贡献 |
|------|------|------|
| 1959 | [Dijkstra](https://doi.org/10.1007/BF01386390) | 最短路径 |
| 1968 | [Hart et al., A*](https://doi.org/10.1109/TSSC.1968.300136) | 启发式搜索 |
| 1990 | [Eriksson & Borenstein, GURVEY](https://ieeexplore.ieee.org/document/67314) | NavFn 前身 |
| 1998 | [Kavraki et al., PRM](https://doi.org/10.1109/70.508439) · [LaValle, RRT](https://lavalle.pl/rrtpubs.html) | 采样规划 |
| 2007 | [Nash et al., Theta*](https://cdn.aaai.org/AAAI/2007/AAAI07-187.pdf) | 任意角 |
| 2008 | [Dolgov et al., Hybrid A*](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf) | 运动学栅格 |
| 2011 | [Karaman & Frazzoli, RRT*](https://doi.org/10.1177/0278364911406761) | 渐近最优 |

**工程**

- [Navigation2 Planner Server](https://docs.nav2.org/configuration/packages/configuring-planner-server.html)
- [nav2_navfn_planner](https://github.com/ros-navigation/navigation2/tree/main/nav2_navfn_planner)
- [nav2_theta_star_planner](https://github.com/ros-navigation/navigation2/tree/main/nav2_theta_star_planner)

---

## 6.12 相关文档

- [§0 指南](00_guide.md) · [§1 架构](01_architecture.md) · [§2 规划器总览](02_planner_algorithms.md)
- [NavFn](03_navfn.md) · [Dijkstra](04_dijkstra.md) · [Theta*](05_theta_star.md)
- [Navigator 导航编排](../16_Navigator/09_survey.md) · [Control 轨迹规划综述](../09_Control/06_survey.md)（局部时空联合 · [§6.6.4](../09_Control/06_survey.md#664-时空联合轨迹规划)）
