(navigator-math)=
# 3. 数学原理

> 行为树节点语义与架构细节见 [06_bt_engine.md](06_bt_engine.md)；单点导航 BT 逐层解析见 [07_navigate_to_pose.md](07_navigate_to_pose.md)；GoalChecker 完整推导见 [Control §15 SimpleGoalChecker](../09_Control/checker/15_simple_goal_checker.md#4-数学问题定义)。

### 3.1 问题形式化（SE(2)）

**导航编排**层将用户目标建模为 SE(2) 位姿：

$$
q = (x, y, \theta) \in \mathbb{R}^2 \times \mathbb{S}^1
$$

位姿变换（齐次形式）：

$$
T(q) =
\begin{bmatrix}
\cos\theta & -\sin\theta & x \\
\sin\theta &  \cos\theta & y \\
0 & 0 & 1
\end{bmatrix}
\in \mathrm{SE}(2)
$$

给定起点 $q_s$、目标 $q_g$，导航任务求控制序列 $\{u_k\}_{k=0}^{N-1}$，使机器人在全局帧 $\mathcal{F}_g$（默认 `map`）下到达目标邻域，同时满足：

$$
\min_{\{u_k\}} \; J = \sum_{k=0}^{N-1} \Big( w_t \,\Delta t_k + w_r \,\mathbf{1}_{\{r_k\}} \Big)
$$

约束：

$$
q_k \in \mathcal{C}_{\mathrm{free}}, \quad
T_g^{-1} T(q_k) \in \mathcal{E}_{\mathrm{goal}}
$$

其中 $\mathcal{C}_{\mathrm{free}}$ 由全局/局部 costmap 定义，$\mathcal{E}_{\mathrm{goal}}$ 为到达容差区域（见 §3.2）。

**坐标变换**：机器人当前位姿 $q_r$ 通过 TF 链获得：

$$
T_g^r = T_g^o \cdot T_o^r
$$

`TransformAvailable` 判定等价于：

$$
\exists \, \Delta t \leq \tau_{\text{tf}} : \; \mathrm{lookup}(T_g^r, t - \Delta t) \neq \emptyset
$$

默认 $\tau_{\text{tf}} = 0.1\,\mathrm{s}$（`transform_tolerance` / XML 端口）。

### 3.2 双层目标判定

Autonomy 采用 **BT 层快速判定 + Controller 层精确判定** 的双层架构，避免 FollowPath 仍在 RUNNING 时无法及时 SUCCESS。

#### 3.2.1 BT 层 — `GoalReached`（仅 XY）

Condition 节点每 tick 瞬时判定，**不检查航向**：

$$
d_{xy}(q_r, q_g) = \sqrt{(x_r - x_g)^2 + (y_r - y_g)^2}
$$

$$
\mathrm{GoalReached} =
\begin{cases}
\mathrm{SUCCESS}, & d_{xy}(q_r, q_g) \leq \varepsilon_{xy} \\
\mathrm{FAILURE}, & \mathrm{otherwise}
\end{cases}
$$

默认 $\varepsilon_{xy} = 0.25\,\mathrm{m}$（`AUTONOMY_COMMON.goal_reached_tolerance` → blackboard `goal_reached_tol`）。

**设计动机**：`GoalReached` 置于 `ReactiveFallback` 首位，每 tick 优先检验；一旦 XY 满足，整棵导航树立即 SUCCESS，无需等待 Controller GoalChecker 完成航向对齐。

#### 3.2.2 Controller 层 — `SimpleGoalChecker`（Stateful XY + Yaw）

`FollowPath` 内部委托 `SimpleGoalChecker::IsGoalReached()`，采用 **两阶段 stateful** 判定：

**阶段 A（`check_xy_ == true`）**：

$$
d_{xy}^2 = (x_r - x_g)^2 + (y_r - y_g)^2
$$

若 $d_{xy}^2 > \varepsilon_{xy}^2$，返回 **false**。

若 XY 通过且 `stateful == true`，锁定：`check_xy_ \leftarrow \mathrm{false}`。

**阶段 B（航向）**：

$$
\Delta\theta = \mathrm{AngleDiff}(\theta_r, \theta_g)
$$

$$
\mathrm{reached} = \big( |\Delta\theta| \leq \varepsilon_\theta \big)
$$

`Reset()` 时恢复 `check_xy_ = true`。

#### 3.2.3 两层判定对比

| 维度 | BT `GoalReached` | Controller `SimpleGoalChecker` |
|------|------------------|-------------------------------|
| 检查量 | 仅 $d_{xy}$ | $d_{xy}$ + $|\Delta\theta|$ |
| 状态 | 无状态（每 tick 重算） | Stateful（XY 锁定后只检航向） |
| 触发时机 | ReactiveFallback 首位 | FollowPath RUNNING 期间 |
| 容差来源 | `goal_reached_tol` | `xy_goal_tolerance` / `yaw_goal_tolerance` |
| 典型用途 | 快速结束导航任务 | 精确停车对齐 |

**协同时序**：

$$
\underbrace{\mathrm{FollowPath\ RUNNING}}_{\mathrm{Controller}}
\xrightarrow{d_{xy} \leq \varepsilon_{xy}}
\underbrace{\mathrm{GoalReached\ SUCCESS}}_{\mathrm{BT}}
$$

即 Controller 仍在检查 XY 与 Yaw 时，只要 BT 层先满足 XY 容差，导航树就会立即成功返回。

若 BT 层 $\varepsilon_{xy}$ 与 Controller 层 $\varepsilon_{xy}$ 不一致，可能出现 BT 已 SUCCESS 但 Controller 仍在微调航向——因此 **`common.lua` 要求三处容差一致**。

#### 3.2.4 `PositionGoalChecker` 与 `StoppedGoalChecker`

| Checker | 判定函数 | 说明 |
|---------|----------|------|
| `PositionGoalChecker` | $d_{xy} \leq \varepsilon_{xy}$ | 仅位置，Stateful |
| `StoppedGoalChecker` | Simple + $v_{trans} \leq v_{stop}$ | 精密对接 |

BT 层 `GoalReached` 与 `PositionGoalChecker` 语义最接近，均忽略航向。

### 3.3 反馈度量（Feedback Metrics）

`NavigateToPoseAction::Feedback` 字段及其计算：

| 字段 | 符号 / 公式 | 更新频率 |
|------|-------------|----------|
| `current_pose` | $q_r(t)$，来自 TF 或里程计 | 每 BT tick |
| `navigation_time` | $t - t_{\text{start}}$ | 每 tick |
| `distance_remaining` | 见下文 | 每 tick |
| `number_of_recoveries` | RecoveryNode 触发计数 | 恢复时 +1 |
| `estimated_time_remaining` | $d_{\text{rem}} / \bar{v}$（可选） | 每 tick |

**剩余距离**（两种常用定义）：

沿路径弧长（推荐，与 FollowPath 一致）：

$$
d_{\text{rem}} = \sum_{j=i^*}^{n-1} \| p_{j+1} - p_j \|_2
$$

其中 $i^* = \arg\min_i \| p_i - q_r \|_2$ 为路径上最近点索引。

欧氏距离（简化）：

$$
d_{\text{rem}} = d_{xy}(q_r, q_g)
$$

**恢复计数**：SafeNavigate 的 `RecoveryNode` 每次执行 `NavigationRecovery` 子树时：

$$
N_{\text{rec}} \leftarrow N_{\text{rec}} + 1
$$

写入 Feedback 供上层监控导航健康度。

### 3.4 TF 可用性

#### 3.4.1 TransformAvailable 判定

$$
\mathrm{TF}_{\mathrm{OK}}(t) =
\begin{cases}
\mathrm{true}, & \mathrm{canTransform}(\mathcal{F}_g, \mathcal{F}_b, t, \tau_{\text{tf}}) \\
\mathrm{false}, & \mathrm{otherwise}
\end{cases}
$$

其中 $\mathcal{F}_b$、$\mathcal{F}_g$ 分别对应黑板键 `robot_base_frame`、`global_frame`，默认 `base_link` 与 `map`。

#### 3.4.2 模式切换条件

`navigate_to_pose.xml` 中 `ReactiveFallback` 三分支优先级：

$$
\mathrm{Branch} =
\begin{cases}
\mathrm{GoalReached}, & d_{xy}(q_r, q_g) \leq \varepsilon_{xy} \\
\mathrm{GlobalMode}, & \mathrm{TF}_{\mathrm{OK}}(t) \land \mathrm{InitPoseReceived} \\
\mathrm{LocalSurvivalMode}, & \neg \mathrm{TF}_{\mathrm{OK}}(t)
\end{cases}
$$

其中 $\mathrm{InitPoseReceived}$ 对应 BT 条件节点 `InitialPoseReceived`（黑板键 `initial_pose_received`）。

TF 恢复时，`GlobalMode` 的 `TransformAvailable` SUCCESS → 自动退出局部生存模式，无需显式状态变量。

### 3.5 BT Tick 时序

#### 3.5.1 主循环周期

`BtActionServer::Run()` 以固定周期驱动 tick：

$$
T_{\text{loop}} = t_{\mathrm{loop}} = 10\,\mathrm{ms}
$$

默认配置项为 `bt_loop_duration = 10 ms`。

等价 tick 频率：

$$
f_{\text{tick}} = \frac{1}{T_{\text{loop}}} = 100\,\mathrm{Hz}
$$

单次 tick 伪代码：

$$
s_{k+1} = \mathrm{tick}(\mathcal{T}, \mathcal{B}_k), \quad
\mathcal{B}_{k+1} = \mathrm{update}(\mathcal{B}_k, s_{k+1})
$$

其中 $\mathcal{T}$ 为行为树，$\mathcal{B}$ 为黑板。

#### 3.5.2 与子系统频率关系

| 组件 | 频率 | 关系 |
|------|------|------|
| BT tick | 100 Hz | $T_{\text{loop}} = 10$ ms |
| RateController(5 Hz) | 5 Hz | $T_{\min} = 200$ ms |
| Controller | 10–50 Hz | 独立控制环 |
| Costmap 更新 | 5–20 Hz | 异步 |

BT tick 频率 **高于** RateController 子频率，Decorator 在 tick 间拦截多余调用。

### 3.6 RateController

限制子节点最大执行频率：

$$
T_{\min} = \frac{1}{f_{\max}}, \quad f_{\max} = \mathrm{hz}
$$

对第 $k$ 次 tick：

$$
\mathrm{RateController.tick} =
\begin{cases}
\mathrm{child.tick}(), & t_k - t_{\text{last}} \geq T_{\min} \\
\mathrm{child.status}(), & \mathrm{otherwise}
\end{cases}
$$

其中 `otherwise` 表示保持上一次子节点状态，不重新执行子节点。

`navigate_to_pose.xml` 中 `ComputePathToPose` 包裹 `RateController hz="5.0"`：

$$
T_{\min} = 0.2\,\mathrm{s}, \quad f_{\max} = 5\,\mathrm{Hz}
$$

**作用**：避免每 10 ms tick 都触发全局规划，降低 CPU 与 costmap 锁竞争。

### 3.7 PipelineSequence

流水线控制节点：前序子节点 SUCCESS 后激活下一个，**已完成子节点保持 SUCCESS 且不再 tick**（与标准 `Sequence` 不同）。

形式化（$m$ 个子节点）：

$$
\mathrm{active}(i) =
\begin{cases}
\mathrm{true}, & i = 0 \lor \big( i > 0 \land \mathrm{status}(N_{i-1}) = \mathrm{SUCCESS} \big) \\
\mathrm{false}, & \mathrm{otherwise}
\end{cases}
$$

$$
\mathrm{PipelineSequence.status} =
\begin{cases}
\mathrm{RUNNING}, & \exists i : \mathrm{active}(i) \land \mathrm{status}(N_i) = \mathrm{RUNNING} \\
\mathrm{FAILURE}, & \exists i : \mathrm{status}(N_i) = \mathrm{FAILURE} \\
\mathrm{SUCCESS}, & \forall i : \mathrm{status}(N_i) = \mathrm{SUCCESS}
\end{cases}
$$

**GlobalNavigatePipeline 时序**：

$$
\underbrace{\mathrm{Selector\times 3}}_{\mathrm{single\mbox{-}shot\ success}}
\xrightarrow{}
\underbrace{\mathrm{ComputePath}}_{\mathrm{periodic\ success}}
\xrightarrow{}
\underbrace{\mathrm{SmoothPath}}_{\mathrm{single\mbox{-}shot\ success}}
\xrightarrow{}
\underbrace{\mathrm{IsPathValid}}_{\mathrm{checked\ every\ tick}}
\xrightarrow{}
\underbrace{\mathrm{FollowPath}}_{\mathrm{continuous\ running}}
$$

| 特性 | `Sequence` | `PipelineSequence` |
|------|-----------|-------------------|
| 已完成子节点 | 不再 tick | 保持 SUCCESS，不再 tick |
| 适用 | 一次性步骤链 | 规划 + 持续跟踪混合流水线 |

### 3.8 RecoveryNode

恢复控制节点，参数化重试：

$$
\mathrm{RecoveryNode}(M, R, K)
$$

语义：

$$
\mathrm{result} =
\begin{cases}
\mathrm{SUCCESS}, & \mathrm{status}(M) = \mathrm{SUCCESS} \\
\mathrm{FAILURE}, & \mathrm{status}(M) = \mathrm{FAILURE} \land \mathrm{retries} \geq K \\
\mathrm{RUNNING}, & \mathrm{executing}(R) \lor \mathrm{retrying}(M)
\end{cases}
$$

每次 $M$ 失败后执行恢复子树 $R$，计数 $\mathrm{retries} \leftarrow \mathrm{retries} + 1$，若 $\mathrm{retries} < K$ 则重新 tick $M$。

**SafeNavigate**（`navigate_to_pose.xml`）：

$$
\mathrm{RecoveryNode}(\mathrm{AdaptiveNavigation}, \mathrm{NavigationRecovery}, K=8)
$$

**LocalSurvivalMode**：

$$
\mathrm{RecoveryNode}(\mathrm{LocalMotionAndRelocalize}, \mathrm{RelocalizationRecovery}, K=200)
$$

### 3.9 局部生存模式时序

TF 丢失时进入局部生存分支，核心时间约束：

#### 3.9.1 生存时间窗

`TimeExpired` 判定自节点激活起：

$$
t_{\text{elapsed}} = t_{\text{now}} - t_{\text{activate}}
$$

$$
\mathrm{TimeExpired} =
\begin{cases}
\mathrm{SUCCESS}, & t_{\text{elapsed}} \geq T_{\text{survival}} \\
\mathrm{FAILURE}, & \mathrm{otherwise}
\end{cases}
$$

默认 $T_{\text{survival}} = 120\,\mathrm{s}$（黑板键 `local_survival_timeout`）。

XML 中用 `Inverter` 包裹：

$$
\mathrm{Inverter}(\mathrm{TimeExpired}) =
\begin{cases}
\mathrm{FAILURE}, & t_{\text{elapsed}} \geq T_{\text{survival}} \\
\mathrm{SUCCESS}, & t_{\text{elapsed}} < T_{\text{survival}}
\end{cases}
$$

即超时会让 Sequence 失败并退出局部生存分支；未超时则继续局部运动。

#### 3.9.2 RoundRobin 局部运动

三轮循环（每轮 `ForceFailure` 保证继续轮询）：

| 轮次 | 动作 | 距离/角度 | 速度 | 时限 |
|------|------|-----------|------|------|
| 1 | DriveOnHeading | 0.8 m | 0.12 m/s | 8 s |
| 2 | Spin | 0.8 rad | — | 6 s |
| 3 | BackUp | 0.20 m | 0.08 m/s | 6 s |

单轮最大耗时上界：

$$
T_{\text{cycle}} \leq 8 + 6 + 6 = 20\,\mathrm{s}
$$

120 s 生存窗口内最多约 $\lfloor 120/20 \rfloor = 6$ 个完整循环（不含 RelocalizationRecovery）。

#### 3.9.3 TF 反向检验

$$
\mathrm{Inverter}(\mathrm{TransformAvailable}) =
\begin{cases}
\mathrm{SUCCESS}, & \neg \mathrm{TF}_{\mathrm{OK}}(t) \\
\mathrm{FAILURE}, & \mathrm{TF}_{\mathrm{OK}}(t)
\end{cases}
$$

TF 仍不可用时继续局部模式；一旦 TF 恢复，分支失败并切回全局模式。

### 3.10 路径有效性（IsPathValid）

BT Condition `IsPathValid` 调用 `PlannerServer::IsPathValid()`，沿路径做碰撞检测。

#### 3.10.1 最近点索引

$$
i^* = \arg\min_{i \in [0, n-1]} \| p_i - q_r \|_2
$$

#### 3.10.2 沿路径代价检验

对 $i \in [i^*, n-1]$，计算位姿 $(x_i, y_i, \theta_i)$ 处代价：

**半径模式**（`use_radius`）：

$$
c_i = \mathrm{costmap}(m_x, m_y), \quad (m_x, m_y) = \mathrm{worldToMap}(x_i, y_i)
$$

**Footprint 模式**：

$$
c_i = \mathrm{footprintCostAtPose}(x_i, y_i, \theta_i, \mathcal{F}_{\text{robot}})
$$

#### 3.10.3 有效性判定

$$
\mathrm{IsPathValid} =
\begin{cases}
\mathrm{false}, & \exists i \geq i^* : c_i \geq c_{\max} \lor c_i = \mathrm{LETHAL} \\
\mathrm{true}, & \mathrm{otherwise}
\end{cases}
$$

未知区域（`NO_INFORMATION`）：

$$
c_i =
\begin{cases}
\mathrm{LETHAL}, & u_{\mathrm{obs}} = \mathrm{true} \\
\mathrm{FREE}, & \mathrm{otherwise}
\end{cases}
$$

其中 $u_{\mathrm{obs}}$ 为配置项 `consider_unknown_as_obstacle`。

Pipeline 中 `IsPathValid` FAILURE → 触发 SafeNavigate 恢复链（清图 + BackUp + Spin）。

### 3.11 黑板键（Blackboard Keys）

黑板 $\mathcal{B}$ 是 BT 节点间共享键值存储，XML 通过 `{key}` 引用。

#### 3.11.1 配置注入键（PopulateBlackboardDefaults）

| 键 | 类型 | 来源 | 默认值 |
|----|------|------|--------|
| `global_frame` | string | `navigator.lua` | `map` |
| `robot_base_frame` | string | `navigator.lua` | `base_link` |
| `default_planner_id` | string | `common.lua` | `navfn_planner` |
| `default_controller_id` | string | `common.lua` | `FollowPath` |
| `default_goal_checker_id` | string | `common.lua` | `goal_checker` |
| `goal_reached_tol` | double | `common.lua` | `0.25` |
| `local_survival_timeout` | double | `navigator.lua` | `120.0` |
| `initial_pose_received` | bool | OnLoop / AMCL | `false` |

#### 3.11.2 运行时数据键

| 键 | 写入者 | 读取者 |
|----|--------|--------|
| `goal` | Navigator::OnGoalReceived | GoalReached, ComputePathToPose |
| `goals` | NavigateThroughPosesNavigator | ComputePathThroughPoses |
| `path` | ComputePath*, SmoothPath | IsPathValid, FollowPath |
| `selected_planner` | PlannerSelector | ComputePath* |
| `selected_controller` | ControllerSelector | FollowPath |
| `selected_smoother` | SmootherSelector | SmoothPath |
| `compute_path_error_code` | ComputePathToPose | AreErrorCodesPresent |
| `follow_path_error_code` | FollowPath | WouldAControllerRecoveryHelp |

#### 3.11.3 系统键

| 键 | 常量 | 说明 |
|----|------|------|
| `autolink_node` | `kBlackboardAutolinkNodeKey` | autolink 节点指针，插件访问 TF/Action |

**黑板更新方程**（Action 节点完成时）：

$$
\mathcal{B}[p] \leftarrow \pi_{\text{plan}}(q_s, q_g), \quad
\mathcal{B}[p_{\mathrm{sel}}] \leftarrow \mathrm{Selector.read}()
$$

其中 $p$、`p_sel` 分别对应黑板键 `path`、`selected_planner`。

### 3.12 导航编排流水线总览

$$
q_g \xrightarrow{\mathrm{TF\ check}} \mathrm{Plan}_{5\mathrm{Hz}} \xrightarrow{\mathrm{Smooth}} \mathrm{Validate} \xrightarrow{\mathrm{Follow}} \mathrm{GoalReached?}
$$

$$
\mathrm{TF\ lost} \xrightarrow{t < 120\,\mathrm{s}} \mathrm{RoundRobin\ local\ motion} \xrightarrow{\mathrm{TF\ restored}} \mathrm{GlobalMode}
$$

$$
\mathrm{Pipeline\ FAILURE} \xrightarrow{\times 8} \mathrm{NavigationRecovery}
$$

### 3.13 相关文档

- [单点导航 BT · GoalReached](07_navigate_to_pose.md#741-goalreached第一优先级)
- [BT 插件 · Control 节点](08_bt_plugins.md#84-control-节点3)
- [使用指南 · common.lua 契约](04_usage.md#412-commonlua-参数契约)
- [Control §15 SimpleGoalChecker · 到达判定](../09_Control/checker/15_simple_goal_checker.md#4-数学问题定义)
