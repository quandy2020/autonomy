(navigator-guide)=
# 0. Navigator 导航编排指南

`autonomy/navigator`：移动机器人**导航编排**层，对齐 nav2 `nav2_bt_navigator`。通过行为树（BT）协调 `planning`、`control`、`map`、`localization`，完成单点导航与多点巡航。

| 本文 §0 | 其他文档 |
|---------|----------|
| 快速开始、配置、排错 | [§1 架构](01_architecture.md) · [§2 BT 总览](02_bt_algorithms.md) · [§6 综述](06_survey.md) |

---

## 0.1 文档地图

| 角色 | 阅读顺序 |
|------|----------|
| 新手 | §0.2 → [§1](01_architecture.md) → [§2](02_bt_algorithms.md) |
| BT 定制 | [§4 单点 BT](04_navigate_to_pose.md) → [§5 插件](05_bt_plugins.md) → [§3 引擎](03_bt_engine.md) |
| 集成 | §0.2、§0.5、§0.8 → [§6.6 选型](06_survey.md#66-autonomy-编排选型) |
| 背景调研 | [§6 综述](06_survey.md) → [§4](04_navigate_to_pose.md) |

| § | 文档 | 内容 |
|---|------|------|
| 0 | 本指南 | 上手、形式化、配置、排错 |
| 1 | [架构](01_architecture.md) | 分层、数据流、状态、扩展 |
| 2 | [BT 总览](02_bt_algorithms.md) | 模式对比与专题索引 |
| 3–5 | 引擎 · 单点 BT · 插件 | 实现与 XML 专题 |
| 6 | [综述](06_survey.md) | 历史、分类、选型 |

---

## 0.2 快速开始

1. `config/navigator/navigator.lua` — 帧、容差、BT XML、插件列表
2. `config/autonomy.lua` — `navigator = include("navigator/navigator.lua").navigator`
3. 启动 `system::Autonomy` 并调用 `NavigateToPose()`

`config/common.lua` 与 planner / controller / navigator **三处必须一致**：

```lua
AUTONOMY_COMMON = {
    global_frame = "map",
    robot_base_frame = "base_link",
    default_planner_id = "navfn_planner",
    default_controller_id = "FollowPath",
    default_goal_checker_id = "goal_checker",
    default_smoother_id = "simple_smoother",
    goal_reached_tolerance = 0.25,
}
```

**直驱模式（当前默认）**：`Autonomy::Configure()` 设置 `use_bt_navigation_ = false`，`NavigateToPose()` 仅调用 `GetPlan` 并 `NotifyPath`，不执行 `FollowPath`。

```cpp
autonomy::system::RuntimeOptions runtime;
runtime.config_directory = "config";
runtime.use_bt_navigation = false;
autonomy->Configure(runtime);
autonomy->NavigateToPose(goal, cancel, keep_alive, 300.0);
```

**BT 模式（目标）**：

```bash
export AUTONOMY_BT_PLUGIN_PATH=/path/to/install/lib
bazel run //autonomy/system/tools:nav_test -- --config_directory=config --use_bt=true
```

| 模式 | 配置 | 行为 |
|------|------|------|
| 直驱规划 | `use_bt_navigation = false` | 单次 `GetPlan`，不 FollowPath |
| BT 单点 | `use_bt_navigation = true` | `navigate_to_pose.xml` 完整流水线 |
| BT 多点 | `NavigateThroughPoses()` | `navigate_through_poses.xml` |

> **当前阶段**：配置、接口骨架、BT XML 已就绪；`BtEngine` + 52 插件 + `BtNavigator` 待迁回。详见 [§1.2 实现状态](01_architecture.md#12-实现状态)。

---

## 0.3 问题形式化

(navigator-overview)=

**任务.** 给定起点 $q_s$、目标 $q_g \in \mathrm{SE}(2)$，求控制序列使机器人在全局帧 $\mathcal{F}_g$（默认 `map`）下进入目标邻域，并满足 costmap 可行约束。

$$
\min_{\{u_k\}} \; J = \sum_{k=0}^{N-1} \Big( w_t \,\Delta t_k + w_r \,\mathbf{1}_{\{r_k\}} \Big)
\quad
\mathrm{s.t.}\;
q_k \in \mathcal{C}_{\mathrm{free}},\;
T_g^{-1} T(q_k) \in \mathcal{E}_{\mathrm{goal}}
$$

**编排层职责**：不直接解 $J$，而是通过 BT 调度 `ComputePath` → `FollowPath` → `Recovery`，并维护任务生命周期。路径搜索与跟踪分别由 [Planning](../08_Planning/00_guide.md)、[Control](../09_Control/00_guide.md) 完成。

**双层到达判定.** BT `GoalReached` 仅检 XY（每 tick 无状态）；`FollowPath` 内 `SimpleGoalChecker` 检 XY + 航向（stateful）。`GoalReached` 置于 `ReactiveFallback` 首位，XY 满足后整树立即 SUCCESS：

$$
d_{xy}(q_r, q_g) = \sqrt{(x_r - x_g)^2 + (y_r - y_g)^2} \leq \varepsilon_{xy}
$$

默认 $\varepsilon_{xy} = 0.25\,\mathrm{m}$（`AUTONOMY_COMMON.goal_reached_tolerance`）。完整 GoalChecker 推导见 [Control · SimpleGoalChecker](../09_Control/checker/15_simple_goal_checker.md#4-数学问题定义)。

**TF 模式切换**（`navigate_to_pose.xml`）：

$$
\mathrm{Branch} =
\begin{cases}
\mathrm{GoalReached}, & d_{xy} \leq \varepsilon_{xy} \\
\mathrm{GlobalMode}, & \mathrm{TF}_{\mathrm{OK}} \land \mathrm{InitPoseReceived} \\
\mathrm{LocalSurvivalMode}, & \neg \mathrm{TF}_{\mathrm{OK}}
\end{cases}
$$

`TransformAvailable` 等价于 $\mathrm{canTransform}(\mathcal{F}_g, \mathcal{F}_b, t, \tau_{\mathrm{tf}})$，默认 $\tau_{\mathrm{tf}} = 0.1\,\mathrm{s}$。逐层 XML 解析见 [§4](04_navigate_to_pose.md)。

---

## 0.4 能力边界

| 能力 | 状态 | 说明 |
|------|------|------|
| `NavigatorOptions` 配置管线 | ✅ | Lua → Protobuf |
| `NavigatorInterface` / `BehaviorTreeNavigator` | ✅ | 头文件完整 |
| `NavigatorMuxer` 互斥 | ✅ | 单 Navigator 活跃 |
| BT XML（单点含局部生存 / 多点） | ✅ | `config/navigator/behavior_tree/` |
| 52 BT 插件清单 + Groot 模型 | ⏳ | `navigator.lua` 已配置，`.so` 待迁回 |
| `BtEngine` / `BtActionServer` | ⏳ | 实现待恢复 |
| `Autonomy` BT 接线 | ⏳ | 当前 `use_bt_navigation_ = false` |
| 直驱规划验证 | ✅ | `NavigateDirectToPose` → `GetPlan` |

---

## 0.5 配置与 API

(navigator-usage)=

**`config/navigator/navigator.lua`**

| 字段 | 说明 | 默认 |
|------|------|------|
| `global_frame` / `robot_base_frame` | TF 帧 | 来自 `common.lua` |
| `bt_loop_duration` | BT tick 周期 (ms) | `10` |
| `default_server_timeout` | 子 Action 超时 (ms) | `20000` |
| `local_survival_timeout` | 局部生存最长等待 (s) | `120.0` |
| `goal_reached_tolerance` | BT `goal_reached_tol` | `0.25` |
| `plugin_lib_names` | BT 插件 `.so` 列表 | 52 个 |
| `navigate_to_pose.behavior_tree_file` | 单点 BT XML | `navigate_to_pose.xml` |

**`system::Autonomy`**

| API | 用途 |
|-----|------|
| `Configure(RuntimeOptions)` | 加载 navigator 配置 |
| `NavigateToPose(goal, …)` | 单点导航 |
| `NavigateThroughPoses(goals, …)` | 多点巡航 |
| `ReplanToGoal(goal)` | 触发重规划 |
| `GetLastPath()` | 最近规划路径 |
| `RequestCancelNavigation()` | 取消当前导航 |

顶层 Action：`NavigateToPoseAction`、`NavigateThroughPosesAction`；子 Action 含 `ComputePathToPose`、`FollowPath`、`Spin` / `BackUp` 等。分层与时序见 [§1.3–§1.4](01_architecture.md#13-分层架构)。

---

## 0.6 自定义行为树

1. 复制 `navigate_to_pose.xml` 并修改节点组合（参考 [§4](04_navigate_to_pose.md)）
2. 更新 `navigator.lua` 的 `behavior_tree_file`，或 Goal 的 `behavior_tree` 字段运行时指定
3. 新增节点时同步 `autonomy_tree_nodes.xml` 与 `plugin_lib_names`

```xml
<RateController hz="10.0">
  <ComputePathToPose goal="{goal}" path="{path}" planner_id="{selected_planner}"/>
</RateController>
```

黑板键须在 `PopulateBlackboardDefaults()` 或 `OnGoalReceived()` 注入。插件目录见 §0.5 与 [§3.2](03_bt_engine.md#32-btengine)。

---

## 0.7 BT 插件环境

```bash
export AUTONOMY_BT_PLUGIN_PATH=/path/to/install/lib
```

`BtEngine::LoadPlugins()` 从 `plugin_lib_names` 加载各 `.so` 的 `BT_REGISTER_NODES`。Groot 模型：`config/navigator/behavior_tree/autonomy_tree_nodes.xml`。节点目录见 [§5](05_bt_plugins.md)。

---

## 0.8 故障排查

| 错误码 | 名称 | 常见原因 | 处理 |
|--------|------|----------|------|
| 9001 | `NAV_TO_POSE_NOT_INITIALIZED` | BtNavigator 未 Configure | 检查 `Autonomy::Configure()` |
| 9002 | `NAV_TO_POSE_TIMEOUT` | 超过 `timeout_sec` | 增大超时或检查卡住原因 |
| 9003 | `NAV_TO_POSE_CANCELED` | 用户 Cancel | 正常行为 |
| 9004 | `NAV_TO_POSE_PREEMPTED` | 新 Goal 抢占 | 正常行为 |
| 9005 | `FAILED_TO_LOAD_BEHAVIOR_TREE` | XML 或插件缺失 | 检查 BT 文件与 `AUTONOMY_BT_PLUGIN_PATH` |
| 9006 | `NAV_TO_POSE_TF_ERROR` | map→base_link 不可用 | 检查 localization / TF |
| 9007 | `NAV_TO_POSE_INVALID_GOAL` | Goal 帧或位姿无效 | 检查 `frame_id` |
| 9008 | `NO_VALID_PATH` | 规划失败 | costmap、起终点 |
| 9009 | `PLANNER_FAILED` | ComputePath FAILURE | Planning [§0.8](../08_Planning/00_guide.md#08-故障排查) |
| 9010 | `CONTROLLER_FAILED` | FollowPath FAILURE | Control [§0.19](../09_Control/00_guide.md#019-故障排查) |
| 9011 | `SMOOTHER_FAILED` | SmoothPath 超时 | 增大 `max_smoothing_duration` |
| 9012 | `PATH_INVALID` | IsPathValid FAILURE | 清图或重规划 |
| 9013 | `GOAL_CHECKER_FAILED` | 航向无法对齐 | 增大 `yaw_goal_tolerance` |

| 现象 | 先查 |
|------|------|
| 直驱无路径 | Planning [§0.8](../08_Planning/00_guide.md#08-故障排查) |
| `no robot pose` | Controller 里程计 / TF |
| BT 插件加载失败 | `AUTONOMY_BT_PLUGIN_PATH`、`plugin_lib_names` 与 `.so` 一致 |
| TF 错误 (9006) | localization、`map`→`base_link` 发布 |
| 容差不一致 | `common.lua` 三处 `goal_reached_tolerance` |
| FollowPath 失败 (9010) | Control [§0.19](../09_Control/00_guide.md#019-故障排查) |

| 多 Navigator 冲突 | `NavigatorMuxer` 拒绝并发 Goal |

选型矩阵见 [§6.6](06_survey.md#66-autonomy-编排选型)。
