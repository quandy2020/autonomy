# Planning 路径规划指南

`autonomy/planning` 是 Autonomy 机器人框架中的**全局路径规划**子系统，设计对齐 ROS 2 Navigation2（nav2）的 `nav2_planner` 架构。本文档为 Planning 模块总入口，涵盖模块概述、数学原理与使用指南；各专题详见下方子文档。

```{toctree}
:maxdepth: 2
:titlesonly:

architecture
navfn
dijkstra
theta_star
survey
```

## 文档目录

| 文档 | 内容 |
|------|------|
| [架构设计](architecture.md) | 模块分层、类关系、数据流、插件机制 |
| [NavFn 规划器](navfn.md) | 导航势场算法、A*/Dijkstra 模式、梯度跟踪 |
| [Dijkstra 规划器](dijkstra.md) | 基于 NavFn 的 Dijkstra 变体 |
| [Theta* 规划器](theta_star.md) | 任意角度网格规划、视线检测 |
| [路径规划综述](survey.md) | 算法分类、对比与选型建议 |
| 本文 · [模块概述](#模块概述) | 定位、能力、快速开始 |
| 本文 · [数学公式与代码对照](#planning-math) | 问题形式化、算法公式、源码映射 |
| 本文 · [使用指南](#planning-usage) | 配置、API、集成、故障排查 |

---

(planning-overview)=
## 模块概述

### 模块定位

| 维度 | 说明 |
|------|------|
| 规划层级 | 全局路径规划（Global Planner） |
| 输入 | 起点/终点位姿、全局代价地图 |
| 输出 | `planning_msgs::Path`（世界坐标系下的位姿序列） |
| 下游消费者 | Navigator、Controller、可视化等 |
| 对标参考 | nav2_planner、nav2_navfn_planner、nav2_theta_star_planner |

### 核心能力

- **多算法插件**：内置 NavFn、Dijkstra、Theta* 三种全局规划器
- **插件扩展**：基于 autolink `PluginManager` 加载外部 `.so` 插件
- **代价地图集成**：共享 `Costmap2DWrapper`，支持静态层、障碍层、膨胀层
- **路径后处理**：Douglas-Peucker 简化、`SimpleSmoother` 平滑
- **服务接口**：`is_path_valid` 路径碰撞检测服务
- **可取消规划**：所有规划器支持 `cancel_checker` 中断

### 源码目录结构

```
autonomy/planning/
├── planner_server.hpp/cpp      # 规划服务入口（对齐 nav2_planner）
├── planner_options.hpp/cpp     # Lua 配置 → PlannerOptions proto
├── constants.hpp               # 话题/服务/节点名称常量
├── common/
│   ├── planner_interface.hpp   # GlobalPlanner 抽象基类
│   ├── smoother_interface.hpp  # Smoother 抽象基类
│   └── planner_exceptions.hpp  # 规划异常类型
├── planner/
│   ├── navfn/                  # NavFn 核心 + NavfnPlanner 插件
│   ├── dijkstra/               # DijkstraPlanner（继承 NavfnPlanner）
│   └── theta_star/             # ThetaStarPlanner 插件
├── utils/
│   ├── simple_smoother.*       # 路径平滑器
│   ├── path_simplifier.*       # Douglas-Peucker 简化
│   └── geometry_utils.*        # 几何工具
└── proto/                      # Protobuf 配置定义
```

### 快速开始

1. 编辑 `config/planner/planner.lua` 选择默认规划器与参数
2. 在 `config/autonomy.lua` 中通过 `AUTONOMY_PLANNER` 引入配置
3. 系统启动时 `autonomy::system::Autonomy` 自动构造 `PlannerServer`

```cpp
#include "autonomy/planning/planner_server.hpp"

// 需先 autolink::Init()
auto options = autonomy::planning::CreateOptions("config");
auto server = std::make_shared<autonomy::planning::PlannerServer>(options);

commsgs::geometry_msgs::PoseStamped start, goal;
// ... 填充起点终点 ...

auto path = server->GetPlan(start, goal, "navfn_planner", []() { return false; });
```

### 相关模块

- `autonomy/map/costmap_2d` — 全局代价地图
- `autonomy/navigator` — 导航行为树，调用规划服务
- `autonomy/control` — 局部轨迹跟踪
- `autonomy/commsgs/planning_msgs` — 路径消息定义

---

(planning-math)=
## 数学公式与代码对照

本节用数学公式讲清 `autonomy/planning` 的核心原理，并给出与源码的一一对应关系。

### 1. 路径规划问题形式化

#### 1.1 连续空间定义

给定配置空间 $\mathcal{C} \subset \mathbb{R}^2$（平面移动机器人可简化为 $SE(2)$ 的位置分量），障碍区域 $\mathcal{C}_{\text{obs}}$，自由空间：

$$
\mathcal{C}_{\text{free}} = \mathcal{C} \setminus \mathcal{C}_{\text{obs}}
$$

给定起点 $q_s = (x_s, y_s)$ 和终点 $q_g = (x_g, y_g)$，求路径 $\tau: [0,1] \to \mathcal{C}_{\text{free}}$，使得 $\tau(0)=q_s$，$\tau(1)=q_g$，并最小化复合代价：

$$
J(\tau) = \int_0^1 \Big( w_l \|\tau'(s)\| + w_c \, c(\tau(s)) \Big) \, ds
$$

其中 $c(\cdot)$ 为代价地图给出的通行代价，$w_l, w_c$ 为权重。

#### 1.2 栅格离散化

Autonomy 将连续问题离散到 $M \times N$ 栅格。世界坐标 $(x, y)$ 到栅格索引 $(m_x, m_y)$ 的映射（与 `Costmap2D::worldToMap` 一致）：

$$
m_x = \left\lfloor \frac{x - x_0}{\Delta} \right\rfloor, \quad
m_y = \left\lfloor \frac{y - y_0}{\Delta} \right\rfloor
$$

其中 $(x_0, y_0)$ 为地图原点，`resolution` $\Delta$ 为栅格分辨率（默认 $0.05\,\text{m}$）。

反变换（`mapToWorld`）：

$$
x = x_0 + m_x \cdot \Delta, \quad y = y_0 + m_y \cdot \Delta
$$

对应代码（`navfn_planner.cpp`）：

```cpp
mx = static_cast<unsigned int>((wx - costmap->getOriginX()) / costmap->getResolution());
my = static_cast<unsigned int>((wy - costmap->getOriginY()) / costmap->getResolution());
```

### 2. 代价地图与 NavFn 代价变换

#### 2.1 Costmap 代价值语义

| 值 | 符号 | 含义 |
|----|------|------|
| 0 | FREE | 完全自由 |
| 1–252 | $c_{ij}$ | 距障碍越近代价越高 |
| 253 | INSCRIBED | 内切膨胀边界 |
| 254 | LETHAL | 致命障碍 |
| 255 | UNKNOWN | 未知 |

#### 2.2 NavFn 内部代价映射

ROS/Autonomy costmap 值 $c \in [0,252]$ 线性映射为 NavFn 通行代价 $F_{ij}$：

$$
F_{ij} = C_n + \alpha \cdot c_{ij} = 50 + 0.8 \cdot c_{ij}
$$

其中 $C_n = \texttt{COST\_NEUTRAL} = 50$，$\alpha = \texttt{COST\_FACTOR} = 0.8$。

对应源码（`navfn.hpp`）：

```cpp
#define COST_NEUTRAL 50
#define COST_FACTOR 0.8
```

### 3. 导航势场（NavFn 核心）

#### 3.1 连续模型：Eikonal 方程

理想情况下，导航势场 $\phi(x,y)$ 满足 Eikonal 方程：

$$
\|\nabla \phi(x,y)\| = F(x,y)
$$

边界条件 $\phi(q_g) = 0$。解 $\phi(q)$ 表示从 $q$ 到 $q_g$ 的最小通行代价。NavFn 用**平面波近似**在离散栅格上求解。

#### 3.2 反向传播约定

NavFn 从**目标格**向**起点格**传播势场：

$$
\phi(q_g) = 0, \quad \phi(q_s) \text{ 由传播得到}
$$

```cpp
planner_->setStart(map_goal);   // 路径提取起点 = 用户 goal
planner_->setGoal(map_start);   // 传播种子 = 用户 start
```

#### 3.3 单元更新方程（`updateCell`）

对栅格 $n$，设四邻域势场值为 $P_l, P_r, P_u, P_d$，当前格通行代价 $h = F_n$：

$$
t_a = \min(P_u, P_d), \quad t_c = \min(P_l, P_r)
$$

令 $\Delta_c = |t_c - t_a|$（若 $t_a > t_c$ 则交换），新势场值：

$$
\phi_n = \begin{cases}
t_a + h & \text{if } \Delta_c \geq h \\[4pt]
t_a + h \cdot v\!\left(\dfrac{\Delta_c}{h}\right) & \text{otherwise}
\end{cases}
$$

其中 $v(r) = -0.2301\, r^2 + 0.5307\, r + 0.7040$，$r = \Delta_c / h$。

#### 3.4 梯度跟踪提取路径

$$
\mathbf{p}_{k+1} = \mathbf{p}_k - \delta \cdot \frac{\nabla \phi(\mathbf{p}_k)}{\|\nabla \phi(\mathbf{p}_k)\|}
$$

离散梯度用中心差分近似，步长 $\delta = 0.5$ 格。

### 4. Dijkstra 算法

在图 $G=(V,E)$ 上，边权 $w(u,v) \geq 0$，单源最短路径满足松弛方程：

$$
d(v) = \min_{u \in \text{pred}(v)} \big\{ d(u) + w(u,v) \big\}
$$

NavFn 用**桶队列**按阈值 $T, T+2C_n, T+4C_n, \ldots$ 分桶传播。`atStart=true` 时，一旦 $\phi(q_s) < \texttt{POT\_HIGH}$ 即停止。

| 数学概念 | 代码位置 |
|----------|----------|
| 松弛/更新 | `NavFn::updateCell()` |
| 桶队列传播 | `NavFn::propNavFnDijkstra()` |
| 强制 Dijkstra | `DijkstraPlanner` 中 `SetUseAstar(false)` |

### 5. A* 算法（NavFn A* 模式）

A* 按 $f(n) = g(n) + h(n)$ 排序扩展：

$$
h(n) = \|n - q_s\|_2 \cdot C_n
$$

| 模式 | 配置 | 排序键 |
|------|------|--------|
| Dijkstra | `use_astar = false` | $f = g$ |
| A* | `use_astar = true` | $f = g + h$ |

### 6. Theta* 算法

8-连通网格路径长度满足 $L_{\text{grid}} \geq \|q_a - q_b\|$。Theta* 通过视线捷径使 $L_{\text{theta}} \approx \|q_a - q_b\|$。

对当前格 $s$、父节点 $\text{par} = \pi(s)$、邻居 $n$：

**若 $\text{LOS}(\text{par}, n) = \text{true}$：**

$$
g(n) = g(\text{par}) + w_e \cdot \|\text{par} - n\|_2 + w_t \cdot \tau(n), \quad \pi(n) = \text{par}
$$

**否则：**

$$
g(n) = g(s) + w_e \cdot \|s - n\|_2 + w_t \cdot \tau(n), \quad \pi(n) = s
$$

启发式：$h(n) = w_h \cdot \|n - q_g\|_2$。

对应代码（`theta_star_planner.cpp`）：

```cpp
if (lineOfSight(px, py, unx, uny)) {
    tentative_g = g_score[par_idx]
        + w_euc_cost_ * euclideanDistance(px, py, unx, uny)
        + traversalCost(unx, uny);
    new_parent = par_idx;
} else {
    tentative_g = g_score[curr_idx]
        + w_euc_cost_ * euclideanDistance(cx, cy, unx, uny)
        + traversalCost(unx, uny);
    new_parent = curr_idx;
}
```

### 7. 目标容差搜索

当 goal 格不可达时，在 $\mathcal{B}_\varepsilon(q_g) = \{ q \mid \|q - q_g\|_\infty \leq \varepsilon \}$ 内搜索：

$$
q^* = \arg\min_{q \in \mathcal{B}_\varepsilon} \|q - q_g\|_2 \quad
\text{s.t.} \quad \phi(q) < \texttt{POT\_HIGH}
$$

默认 $\varepsilon = 0.1\,\text{m}$。

### 8. 路径后处理

**SimpleSmoother** 能量函数：

$$
E = w_d \sum_{i=1}^{N-1} \|p_i - o_i\|^2
  + w_s \sum_{i=1}^{N-1} \|p_{i-1} - 2p_i + p_{i+1}\|^2
$$

坐标更新：$p_i \leftarrow p_i + w_d (o_i - p_i) + w_s (p_{i-1} + p_{i+1} - 2p_i)$。

### 9. 完整规划流程

$$
\boxed{
\begin{aligned}
&\textbf{输入: } q_s, q_g, \text{Costmap } C \\[4pt]
&1.\; \text{离散化: } (m_x, m_y) = \text{worldToMap}(x, y) \\
&2.\; \text{复制代价图: } c_{ij} \leftarrow C,\; c_{q_s} \leftarrow 0 \\
&3.\; \text{选择算法: NavFn } \phi \leftarrow \text{Dijkstra/A*} \text{ 或 Theta* } \pi \leftarrow \text{A*+LOS} \\
&4.\; \text{提取路径: } \tau \leftarrow \text{GradientTrace}(\phi) \text{ 或 Backtrack}(\pi) \\
&5.\; \text{容差修正 + 后处理} \\
&\textbf{输出: } \text{Path } \tau = \{q_0, q_1, \ldots, q_N\}
\end{aligned}
}
$$

### 10. 算法对比（公式级）

| 算法 | 优化目标 | 扩展规则 | 路径几何 |
|------|----------|----------|----------|
| Dijkstra | $\min \sum w_{ij}$ | $d(v)=\min\{d(u)+w\}$ | 网格 + 梯度 |
| A* | 同 Dijkstra | $f=g+h$ 排序 | 网格 + 梯度 |
| Theta* | $\min \sum (w_e d + w_t c)$ | A* + LOS 捷径 | 任意角直线段 |

---

(planning-usage)=
## 使用指南

### 1. 系统配置

#### 1.1 配置文件结构

规划模块配置位于 `config/planner/planner.lua`，通过 `config/autonomy.lua` 引入：

```lua
-- config/autonomy.lua
include "planner/planner.lua"

AUTONOMY = {
    planning = AUTONOMY_PLANNER,
    -- ...
}
```

#### 1.2 完整配置示例

```lua
AUTONOMY_PLANNER = {
    navfn_planner = {
        tolerance = 0.1,
        use_astar = false,
        allow_unknown = false,
        use_final_approach_orientation = false,
    },
    dijkstra_planner = {
        tolerance = 0.1,
        allow_unknown = false,
        use_final_approach_orientation = false,
    },
    theta_star_planner = {
        how_many_corners = 8,
        allow_unknown = false,
        w_euc_cost = 2.0,
        w_traversal_cost = 1.0,
        w_heuristic_cost = 1.0,
        terminal_checking_interval = 5000,
    },
    expected_planner_frequency = 5.0,
    costmap_update_timeout = 5.0,
    planner_plugins = {
        "navfn_planner",
        "dijkstra_planner",
        "theta_star_planner",
    },
    default_planner_id = "navfn_planner",
    path_simplify_epsilon = 0.0,
    auto_smooth_after_plan = false,
    auto_smooth_duration = 1.0,
    smoother_plugins = { "simple_smoother" },
    default_smoother_id = "simple_smoother",
    simple_smoother = {
        tolerance = 1e-10,
        max_iterations = 1000,
        w_data = 0.2,
        w_smooth = 0.3,
        do_refinement = true,
        refinement_num = 2,
        enforce_path_inversion = true,
    },
    costmap = {
        enabled = true,
        name = "global_map",
        frame_id = "map",
        resolution = 0.05,
        width = 20.0,
        height = 20.0,
        update_frequency = 5.0,
        robot_radius = 0.22,
        plugins = {"static_layer", "obstacle_layer", "inflation_layer"},
    },
}
```

#### 1.3 配置加载（C++ API）

```cpp
#include "autonomy/planning/planner_options.hpp"

auto options = autonomy::planning::CreateOptions("config");
```

### 2. 创建与使用 PlannerServer

#### 2.1 前置条件

```cpp
#include "autolink/common/init.hpp"
#include "autonomy/planning/planner_server.hpp"

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);
    auto options = autonomy::planning::CreateOptions("config");
    auto server = std::make_shared<autonomy::planning::PlannerServer>(options);
}
```

#### 2.2 单次规划

```cpp
commsgs::geometry_msgs::PoseStamped start, goal;
// 填充 start / goal ...

try {
    auto path = server->GetPlan(start, goal, "navfn_planner", []() { return false; });
} catch (const planning::common::NoValidPathCouldBeFound& e) {
    // 处理无路径
} catch (const planning::common::PlannerException& e) {
    // 处理其他规划错误
}
```

#### 2.3 选择规划器

| 调用方式 | 行为 |
|----------|------|
| `GetPlan(..., "navfn_planner", ...)` | 使用指定插件 |
| `GetPlan(..., "theta_star_planner", ...)` | 切换为 Theta* |
| `GetPlan(..., "", ...)` | 仅当加载了 1 个插件时自动选择 |
| 未指定 + 多个插件 | 抛出 `InvalidPlanner` 异常 |

#### 2.4 路径发布回调

```cpp
server->SetPathUpdateCallback(
    [](const commsgs::planning_msgs::Path& path) {
        AINFO << "New plan with " << path.poses.size() << " poses";
    });
```

#### 2.5 路径有效性检查

```cpp
bool valid = server->IsPathValid(path, 253, false);
```

### 3. 系统集成

`system::Autonomy` 在初始化时自动创建 `PlannerServer`：

```cpp
planner_ = std::make_shared<planning::PlannerServer>(options_.planner_options());
```

Navigator 行为树典型流程：

```
用户设定目标 → Navigator BT → GetPlan(start, goal) → 路径下发 Controller
                                    ↓
                              IsPathValid() 周期性检查 → 路径失效 → 重新规划
```

### 4. 通信接口

| 服务/节点 | 名称 | 说明 |
|-----------|------|------|
| 服务 | `is_path_valid` | 检查路径是否碰撞自由 |
| 节点 | `planner_server` | autolink 节点名 |
| 代价地图 | `/global_costmap` | 全局代价地图 |
| Action | `compute_path_to_pose` | 单目标规划（Navigator 封装） |
| Action | `compute_path_through_poses` | 多航点规划 |

### 5. 路径后处理

```cpp
#include "autonomy/planning/utils/path_simplifier.hpp"
#include "autonomy/planning/utils/simple_smoother.hpp"

auto simplified = planning::utils::SimplifyPath(path, epsilon);  // epsilon<=0 禁用

auto smoother = std::make_shared<planning::utils::SimpleSmoother>(
    "simple_smoother", costmap_wrapper, options.simple_smoother());
bool completed = smoother->Smooth(path, std::chrono::milliseconds(1000));
```

> 默认 `path_simplify_epsilon = 0.0`（禁用）。非零值可能将绕障轨迹坍缩为近似直线。

### 6. 自定义规划器插件

**Step 1**：继承 `common::GlobalPlanner`，实现 `CreatePlan()`。

**Step 2**：注册插件：

```cpp
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(my_namespace::MyPlanner, GlobalPlanner);
```

**Step 3**：编写 XML 描述并在 `planner.lua` 中启用：

```lua
planner_plugins = { "navfn_planner", "my_planner:MyPlanner" },
planner_plugin_libraries = { "config/planning/plugins/my_planner_plugin.xml" },
```

### 7. 规划器选型速查

| 场景 | 推荐规划器 | 关键配置 |
|------|-----------|----------|
| 通用室内导航 | `navfn_planner` | 默认即可 |
| 确定性/调试 | `dijkstra_planner` | — |
| 开阔环境/短路径 | `theta_star_planner` | `how_many_corners=8` |
| 窄通道 | `navfn_planner` 或 Theta* 4-连通 | `how_many_corners=4` |
| 大地图 | `navfn_planner` | `use_astar=true` |
| 未知环境探索 | 任意 | `allow_unknown=true` |

### 8. 故障排查

| 错误/异常 | 原因 | 解决方案 |
|-----------|------|----------|
| `NoValidPathCouldBeFound` | 起终点不连通 | 检查 costmap、增大 `tolerance` |
| `StartOccupied` / `GoalOccupied` | 起终点在障碍上 | 调整位姿或清除膨胀 |
| `InvalidPlanner` | 插件 ID 不存在 | 检查 `planner_plugins` 配置 |
| `PlannerTimedOut` | costmap 未及时更新 | 增大 `costmap_update_timeout` |
| `PlannerTFError` | 坐标变换失败 | 检查 TF 树和 `frame_id` |

诊断检查清单：

1. `costmap_wrapper->isCurrent()` 是否为 true
2. 起终点 `worldToMap` 是否在地图范围内
3. `obstacle_layer` / `inflation_layer` 是否生效
4. 启动日志是否显示 `PlannerServer has N planners available`

### 9. 性能建议

| 建议 | 说明 |
|------|------|
| 控制地图大小 | 全局 costmap 建议 ≤ 2048×2048 |
| 合理设置更新频率 | `update_frequency = 5.0` Hz 通常足够 |
| 避免频繁全图重规划 | 仅在路径失效时重规划 |
| 禁用不必要的后处理 | `path_simplify_epsilon = 0` |
| 大地图用 A* | `navfn_planner.use_astar = true` |
