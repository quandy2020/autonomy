# 9. 导航编排综述（Survey）

本文从**学术背景、行为树导航体系、工程实践、Nav2 对照**四个维度，综述移动机器人**导航编排**（Navigation Orchestration）领域，并明确 Autonomy `navigator` 模块的定位与能力边界。

> 数学公式见 [03_math.md](03_math.md)；架构细节见 [05_architecture.md](05_architecture.md)。

---

## 9.1 概述：**导航编排**在导航栈中的位置

移动机器人完整导航栈通常分为五层：

| 层级 | 英文 | 典型模块 | 频率 |
|------|------|----------|------|
| L0 应用 | Application | 用户接口、任务调度 | 事件驱动 |
| **L1 编排** | **Orchestration** | **`navigator`** | **事件驱动** |
| L2 全局规划 | Global Planning | `planning` | 1–5 Hz |
| L3 局部控制 | Local Control | `control` | 10–50 Hz |
| L4 感知定位 | Perception / Localization | `localization`, `map` | 10–50 Hz |

**导航编排**（Navigation Orchestration）负责将用户意图（"去那里"）转化为子系统调用序列（规划、跟踪、恢复），并管理任务生命周期。Autonomy `navigator` 模块专注 L1 编排层。

<div class="nav-stack-pipeline">

  <div class="nav-layer-row nav-layer-behavior">
    <div class="nav-layer-meta">
      <span class="nav-layer-badge">L1</span>
      <div class="nav-layer-title">编排层</div>
      <div class="nav-layer-sub">autonomy / navigator</div>
      <span class="nav-meta-freq">事件驱动</span>
    </div>
    <div class="nav-layer-body">
      <div class="nav-body-block">
        <div class="nav-body-label">核心职责</div>
        <ul>
          <li>解析导航意图，维护任务状态机</li>
          <li>通过 BT 协调规划 / 控制 / 恢复</li>
          <li>监测路径失效、TF 丢失并触发恢复</li>
          <li>提供 Action 接口与 Feedback</li>
        </ul>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">关键组件</div>
        <div class="nav-chip-list">
          <span class="nav-chip">BehaviorTree.CPP</span>
          <span class="nav-chip">52 BT 插件</span>
          <span class="nav-chip">RecoveryNode</span>
          <span class="nav-chip">PipelineSequence</span>
        </div>
      </div>
    </div>
    <div class="nav-layer-io">
      <div class="nav-io-block nav-io-out">
        <div class="nav-io-heading">输出 → L2/L3</div>
        <ul class="nav-io-list">
          <li>目标位姿 <code>PoseStamped</code></li>
          <li>规划 / 跟踪 / 恢复指令</li>
          <li>取消 / 抢占信号</li>
        </ul>
      </div>
    </div>
  </div>

  <div class="nav-pipe">
    <span class="nav-pipe-label">Action 调用 + 黑板数据</span>
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
        <div class="nav-body-label">BT 调用</div>
        <div class="nav-chip-list">
          <span class="nav-chip">ComputePathToPose</span>
          <span class="nav-chip">ComputePathThroughPoses</span>
          <span class="nav-chip">IsPathValid</span>
        </div>
      </div>
    </div>
  </div>

  <div class="nav-pipe">
    <span class="nav-pipe-label">Path</span>
  </div>

  <div class="nav-layer-row nav-layer-local">
    <div class="nav-layer-meta">
      <span class="nav-layer-badge">L3</span>
      <div class="nav-layer-title">局部控制层</div>
      <div class="nav-layer-sub">autonomy / control</div>
      <span class="nav-meta-freq">10–50 Hz</span>
    </div>
    <div class="nav-layer-body">
      <div class="nav-body-block">
        <div class="nav-body-label">BT 调用</div>
        <div class="nav-chip-list">
          <span class="nav-chip">FollowPath</span>
          <span class="nav-chip">Spin / BackUp</span>
          <span class="nav-chip">DriveOnHeading</span>
        </div>
      </div>
    </div>
  </div>

</div>

---

## 9.2 行为树导航：理论与优势

### 9.2.1 行为树 vs 状态机 vs FSM

| 维度 | 有限状态机 (FSM) | 行为树 (BT) |
|------|-----------------|-------------|
| 结构 | 扁平转移图 | 层次化树形 |
| 可读性 | 状态多时难维护 | XML 直观、Groot 可视化 |
| 组合性 | 状态爆炸 | 子树可复用 |
| 响应性 | 需显式转移 | ReactiveFallback 每 tick 重检 |
| 恢复 | 需额外设计 | RecoveryNode 内置 |
| 行业采用 | 早期 ROS | Nav2、Isaac Sim、游戏 AI |

### 9.2.2 核心控制节点

| 节点 | 代数语义 |
|------|----------|
| `Sequence` | $N_1 \land N_2 \land \cdots \land N_m$ |
| `Fallback` | $N_1 \lor N_2 \lor \cdots \lor N_m$ |
| `ReactiveFallback` | 每 tick 重新求 $N_1 \lor N_2 \lor \cdots$ |
| `PipelineSequence` | 流水线：$N_i$ SUCCESS 后激活 $N_{i+1}$，已完成节点不再 tick |
| `RecoveryNode` | $\mathrm{retry}(M, R, K)$：$M$ 失败则执行 $R$，最多 $K$ 次 |
| `Decorator` | $f(\mathrm{child})$：频率限制、取反、触发控制 |

### 9.2.3 黑板模式

黑板（Blackboard）是 BT 节点间的共享数据区：

$$
\mathcal{B} = \{ g, p, q_r, p_{\mathrm{sel}}, \ldots \}
$$

键名对应 `goal`、`path`、`current_pose`、`selected_planner` 等。

- **写入者**：Action 节点（`path`）、Navigator（`goal`）、OnLoop（`current_pose`）
- **读取者**：Condition 节点（`GoalReached`）、Action 节点（`FollowPath`）
- **优势**：解耦节点间依赖，XML 通过 `{key}` 引用

---

## 9.3 导航编排模式

### 9.3.1 标准流水线（Pipeline）

最常用的导航模式，Nav2 和 Autonomy 均采用：

$$
\mathrm{Plan} \xrightarrow{\mathrm{Smooth}} \mathrm{Validate} \xrightarrow{\mathrm{Follow}} \mathrm{Goal}
$$

- `PipelineSequence` 保证按序执行
- `RateController` 限制规划频率
- `FollowPath` 持续 RUNNING 直到 GoalChecker 判定到达

### 9.3.2 反应式导航（Reactive）

Autonomy `navigate_to_pose.xml` 的 `ReactiveFallback`：

1. 每 tick 优先检验 `GoalReached`
2. TF 可用 → 全局模式
3. TF 不可用 → 局部生存模式

这是 **Autonomy 相对 nav2 的增强**：定位丢失时不立即失败，而是低速运动等待重定位。

### 9.3.3 恢复行为（Recovery）

| 恢复级别 | 触发 | 动作 | Autonomy 配置 |
|----------|------|------|---------------|
| L1 局部 | Pipeline FAILURE | 清局部 costmap + Wait | LocalSurvivalMode |
| L2 导航 | SafeNavigate FAILURE | 清全局+局部 + BackUp + Spin | NavigationRecovery ×8 |
| L3 航点 | ThroughPoses FAILURE | 清图 + Wait 0.5s | ThroughPosesRecovery ×6 |

### 9.3.4 多点巡航（Waypoint）

$$
q_{g,1} \to q_{g,2} \to \cdots \to q_{g,n}
$$

`ComputePathThroughPoses` 一次规划贯穿所有航点，10 Hz 重规划应对动态环境。

---

## 9.4 Nav2 导航栈对照

### 9.4.1 包级对照

| Nav2 包 | Autonomy 模块 | 关系 |
|---------|---------------|------|
| `nav2_bt_navigator` | `autonomy/navigator` | 编排层 |
| `nav2_behavior_tree` | `navigator/behavior_tree/plugins` | BT 插件 |
| `nav2_planner` | `autonomy/planning` | 被 BT 调用 |
| `nav2_controller` | `autonomy/control` | 被 BT 调用 |
| `nav2_smoother` | `autonomy/planning` (SimpleSmoother) | 被 BT 调用 |
| `nav2_costmap_2d` | `autonomy/map/costmap_2d` | 被规划/控制/BT 使用 |

### 9.4.2 Action 对照

| Nav2 Action | Autonomy Action | BT 节点 |
|-------------|-----------------|---------|
| `navigate_to_pose` | `NavigateToPoseAction` | 顶层 |
| `navigate_through_poses` | `NavigateThroughPosesAction` | 顶层 |
| `compute_path_to_pose` | `ComputePathToPoseAction` | `ComputePathToPose` |
| `follow_path` | `FollowPathAction` | `FollowPath` |
| `smooth_path` | `SmoothPathAction` | `SmoothPath` |
| `spin` / `backup` / `drive_on_heading` | 同名 Action | 恢复节点 |

### 9.4.3 架构类对照

| Nav2 类 | Autonomy 类 | 状态 |
|---------|-------------|------|
| `nav2_core::BehaviorTreeNavigator` | `BehaviorTreeNavigator<ActionT>` | ✅ |
| `nav2_core::NavigatorMuxer` | `NavigatorMuxer` | ✅ |
| `nav2_bt_navigator::BtNavigator` | `BtNavigator` | ⏳ |
| `nav2_bt_navigator::NavigateToPoseNavigator` | `NavigateToPoseNavigator` | ⏳ |
| `nav2_behavior_tree::BtActionServer` | `BtActionServer` | ⏳ |

---

## 9.5 导航编排的影响因素

### 9.5.1 规划频率

| 频率 | 适用场景 | 代价 |
|------|----------|------|
| 1–2 Hz | 静态环境、计算受限 | 动态障碍响应慢 |
| 5 Hz | 室内通用（Autonomy 默认） | 平衡 |
| 10 Hz | 动态环境、多点巡航 | CPU 占用高 |

### 9.5.2 目标判定策略

| 策略 | BT 层 | Controller 层 | 适用 |
|------|-------|---------------|------|
| 仅 XY | `GoalReached` | `PositionGoalChecker` | 物流、巡检 |
| XY + 航向 | — | `SimpleGoalChecker` | 标准导航 |
| XY + 航向 + 停止 | — | `StoppedGoalChecker` | 精密对接 |

Autonomy 默认 BT 仅检 XY（快速 SUCCESS），Controller 层检航向（精确停车）。

### 9.5.3 定位丢失处理

| 策略 | 描述 | Autonomy |
|------|------|----------|
| 立即失败 | TF 丢失 → 导航 FAILURE | nav2 默认 |
| 局部生存 | 低速运动 + 等待重定位 | `navigate_to_pose.xml` |
| 纯里程计 | 切换到 odom 帧导航 | 未实现 |

### 9.5.4 恢复次数与超时

| 参数 | 影响 |
|------|------|
| `number_of_retries` | 恢复链最大重试 |
| `local_survival_timeout` | 局部生存最长等待 |
| `default_server_timeout` | 子 Action 超时 |
| `time_allowance` | 单个恢复动作时限 |

---

## 9.6 导航编排发展史

| 时期 | 代表系统 | 编排方式 |
|------|----------|----------|
| 2000s | ROS navigation stack | 状态机 + move_base |
| 2018 | ROS 2 Navigation2 | 行为树 (BT.CPP) |
| 2020s | Isaac Sim / Nav2 | BT + 可配置 XML |
| Autonomy | `autonomy/navigator` | nav2 对齐 + 局部生存增强 |

**从 move_base 到 BT 的演进动机**：

1. move_base 恢复逻辑硬编码，难以定制
2. BT XML 可热替换，无需重编译
3. Groot 可视化降低调试门槛
4. 插件化节点便于社区扩展

---

## 9.7 Autonomy Navigator 能力边界

### 9.7.1 已实现

- 配置管线（Lua → Protobuf）
- 接口骨架（`NavigatorInterface`、`BehaviorTreeNavigator`、`NavigatorMuxer`）
- BT XML 定义（单点含局部生存、多点巡航）
- 52 个 BT 插件清单与 Groot 模型
- `system::Autonomy` 直驱规划路径

### 9.7.2 待恢复

- `BtEngine` / `BtActionServer` / `BtContext` 实现
- 52 个 BT 插件 `.so`
- `BtNavigator` + 两个具体 Navigator
- `Autonomy` BT 模式接线（`use_bt_navigation_ = true`）
- gRPC `navigator_stub`

### 9.7.3 不在范围内

| 功能 | 负责模块 |
|------|----------|
| 路径搜索算法 | `planning` |
| 局部轨迹跟踪 | `control` |
| 代价地图构建 | `map/costmap_2d` |
| 定位算法 | `localization` |
| 任务调度（多机器人） | 上层应用 |

---

## 9.8 选型建议

| 场景 | 推荐配置 | 理由 |
|------|----------|------|
| 算法调试（仅验证规划） | 直驱模式 | 跳过 BT，快速迭代 |
| 标准室内导航 | BT 单点默认 XML | 完整流水线 + 恢复 |
| 定位不稳定环境 | BT 单点 + 增大 `local_survival_timeout` | 局部生存等待重定位 |
| 仓库巡检 | BT 多点 | 10 Hz 重规划 + 航点管理 |
| 动态拥挤环境 | BT 单点 + `RateController hz=10` | 高频重规划 |
| 窄通道 | BT + `theta_star_planner` | Selector 切换规划器 |
| 远程调度 | BT + Bridge gRPC（待实现） | 云端下发目标 |

---

## 9.9 相关文档

- [Navigator 导航编排指南](00_guide.md)
- [数学原理](03_math.md)
- [模块架构设计](05_architecture.md)
- [单点导航 BT](07_navigate_to_pose.md)
- [Planning 路径规划综述](../08_Planning/09_survey.md)
- [Control 运动控制综述](../09_Control/09_survey.md)
