# 7. 单点导航行为树

本文逐层解析 `config/navigator/behavior_tree/navigate_to_pose.xml`——Autonomy 默认的**单点导航**行为树，含全局导航、局部生存模式与恢复链。

## 7.1 设计目标

| 目标 | 实现 |
|------|------|
| 标准 nav2 流水线 | 规划 → 平滑 → 验证 → 跟踪 |
| 定位韧性 | TF 丢失时进入局部生存模式 |
| 快速 SUCCESS | `GoalReached` 作为 ReactiveFallback 首子节点 |
| 失败恢复 | 清图 + 后退 + 旋转，最多 8 次 |
| 动态插件切换 | Planner / Controller / Smoother Selector |

## 7.2 树形结构总览

```
RecoveryNode [SafeNavigate, retries=8]
├── ReactiveFallback [AdaptiveNavigation]
│   ├── GoalReached                          ← 已到达 → 整树 SUCCESS
│   ├── Sequence [GlobalMode]                ← TF 可用
│   │   ├── InitialPoseReceived
│   │   ├── TransformAvailable(map←base_link)
│   │   └── PipelineSequence [GlobalNavigatePipeline]
│   │       ├── ControllerSelector
│   │       ├── PlannerSelector
│   │       ├── SmootherSelector
│   │       ├── RateController(5Hz) → ComputePathToPose
│   │       ├── SmoothPath
│   │       ├── IsPathValid
│   │       └── FollowPath
│   └── RecoveryNode [LocalSurvivalMode, retries=200]  ← TF 不可用
│       ├── Sequence [LocalMotionAndRelocalize]
│       │   ├── Inverter(TimeExpired)
│       ├── Inverter(TransformAvailable)
│       └── RoundRobin → DriveOnHeading / Spin / BackUp
│       └── Sequence [RelocalizationRecovery]
│           ├── ClearEntireCostmap(local)
│           └── Wait(0.2s)
└── Sequence [NavigationRecovery]            ← 失败恢复
    ├── ClearEntireCostmap(local + global)
    ├── BackUp(0.25m)
    └── Spin(1.57 rad)
```

## 7.3 顶层：SafeNavigate

```xml
<RecoveryNode name="SafeNavigate" number_of_retries="8">
```

| 参数 | 值 | 含义 |
|------|-----|------|
| `number_of_retries` | 8 | 主分支失败后最多执行 8 次恢复链 |

**语义**：`AdaptiveNavigation` FAILURE 时执行 `NavigationRecovery`（清图 + 后退 + 旋转），恢复成功后重试主分支。

## 7.4 自适应导航：AdaptiveNavigation

```xml
<ReactiveFallback name="AdaptiveNavigation">
```

`ReactiveFallback` **每 tick 从第一个子节点重新检验**，适合响应式条件判断。

### 7.4.1 GoalReached（第一优先级）

```xml
<GoalReached goal="{goal}" goal_reached_tol="{goal_reached_tol}" transform_tolerance="0.1"/>
```

**判定**（仅 XY）：

$$
\sqrt{(x-x_g)^2 + (y-y_g)^2} \leq \varepsilon_{xy}
$$

- 通过 → 整棵 `ReactiveFallback` SUCCESS → 导航完成
- 不通过 → 继续检验下一子节点

> 放在首位的意义：即使 `FollowPath` 仍在 RUNNING，一旦 XY 到达即可立即 SUCCESS，避免多余 tick。

### 7.4.2 GlobalMode（第二优先级）

```xml
<Sequence name="GlobalMode">
  <InitialPoseReceived/>
  <TransformAvailable child="{robot_base_frame}" parent="{global_frame}"/>
  ...
</Sequence>
```

**前置条件**（全部 SUCCESS 才进入 Pipeline）：

| 节点 | 检验内容 |
|------|----------|
| `InitialPoseReceived` | blackboard `initial_pose_received == true` |
| `TransformAvailable` | `canTransform(global_frame, robot_base_frame)` |

### 7.4.3 GlobalNavigatePipeline

```xml
<PipelineSequence name="GlobalNavigatePipeline">
```

流水线按序执行，典型时序：

```mermaid
flowchart LR
    A[Selector×3] --> B[ComputePath 5Hz]
    B --> C[SmoothPath]
    C --> D[IsPathValid]
    D --> E[FollowPath RUNNING]
```

#### Selector 三件套

| 节点 | 默认 | Topic |
|------|------|-------|
| `ControllerSelector` | `{default_controller_id}` | `controller_selector` |
| `PlannerSelector` | `{default_planner_id}` | `planner_selector` |
| `SmootherSelector` | `simple_smoother` | `smoother_selector` |

运行时可通过 topic 动态切换插件，blackboard 写入 `selected_*`。

#### ComputePathToPose（5 Hz 重规划）

```text
<RateController hz="5.0">
  <ComputePathToPose
    goal="{goal}" path="{path}"
    planner_id="{selected_planner}" .../>
</RateController>
```

- 最大规划频率 5 Hz（$T_{\min} = 0.2$ s）
- 输出路径写入 blackboard `{path}`
- 调用 `PlannerServer` action `compute_path_to_pose`

#### SmoothPath

```xml
<SmoothPath
  unsmoothed_path="{path}" smoothed_path="{path}"
  smoother_id="{selected_smoother}"
  max_smoothing_duration="1.0"
  check_for_collisions="false"/>
```

原地平滑：输入输出共用 `{path}` 键。

#### IsPathValid

沿路径做 footprint 碰撞检测，调用 `is_path_valid` 服务。FAILURE → Pipeline FAILURE → 触发恢复。

#### FollowPath

```text
<FollowPath
  path="{path}" controller_id="{selected_controller}" .../>
```

持续 RUNNING 直到 Controller GoalChecker 判定到达或 FAILURE。

### 7.4.4 LocalSurvivalMode（第三优先级）

当 `GlobalMode` 的 `TransformAvailable` FAILURE（TF 不可用）时进入。

```xml
<RecoveryNode name="LocalSurvivalMode" number_of_retries="200">
```

#### 时间窗控制

```xml
<Inverter>
  <TimeExpired seconds="{local_survival_timeout}"/>
</Inverter>
```

- 未超时：Inverter 返回 FAILURE → Sequence 继续（局部运动）
- 已超时（默认 120 s）：Inverter 返回 SUCCESS → Sequence FAILURE → 触发 RelocalizationRecovery

#### TF 反向检验

```xml
<Inverter>
  <TransformAvailable child="{robot_base_frame}" parent="{global_frame}"/>
</Inverter>
```

- TF **不可用**时：Inverter SUCCESS → 继续局部运动
- TF **恢复**时：Inverter FAILURE → 退出局部模式，回到 GlobalMode

#### RoundRobin 局部运动

| 轮次 | 节点 | 参数 |
|------|------|------|
| 1 | `DriveOnHeading` | dist=0.8m, speed=0.12m/s, timeout=8s |
| 2 | `Spin` | spin_dist=0.8rad, timeout=6s |
| 3 | `BackUp` | dist=0.20m, speed=0.08m/s, timeout=6s |

每个运动包裹在 `ForceFailure` 中：无论运动结果如何，都继续轮询下一个原语。

#### RelocalizationRecovery

```xml
<ClearEntireCostmap service_name="local_costmap/clear_costmap"/>
<Wait wait_duration="0.2"/>
```

清局部 costmap 后短暂等待，给定位模块恢复 TF 的机会。

## 7.5 失败恢复：NavigationRecovery

```xml
<Sequence name="NavigationRecovery">
  <ClearEntireCostmap service_name="local_costmap/clear_costmap"/>
  <ClearEntireCostmap service_name="global_costmap/clear_costmap"/>
  <BackUp backup_dist="0.25" backup_speed="0.08" time_allowance="8.0"/>
  <Spin spin_dist="1.57" time_allowance="8.0" is_recovery="true"/>
</Sequence>
```

| 步骤 | 目的 |
|------|------|
| 清局部 + 全局 costmap | 移除陈旧障碍标记 |
| 后退 0.25 m | 脱离碰撞区域 |
| 旋转 π/2 rad | 改变朝向，寻找新路径 |

## 7.6 模式切换状态机

```
                    ┌─────────────┐
                    │ GoalReached?│──YES──→ SUCCESS
                    └──────┬──────┘
                           NO
              ┌────────────┴────────────┐
              ▼                         ▼
     TF available?              TF NOT available
              │                         │
              ▼                         ▼
        GlobalMode              LocalSurvivalMode
     (plan→smooth→follow)    (RoundRobin + relocalize)
              │                         │
              │    TF restored ◄────────┘
              │                         │
              └──── failure ────────────┘
                           ▼
                  NavigationRecovery
                  (×8 retries max)
```

## 7.7 关键参数调优

| 参数 | 位置 | 建议 |
|------|------|------|
| `goal_reached_tol` | `common.lua` / blackboard | 室内 0.25m；精确定位 0.10m |
| `local_survival_timeout` | `navigator.lua` | 定位慢恢复 180s；快速失败 60s |
| `RateController hz` | XML | 动态环境 10Hz；静态 2Hz |
| `number_of_retries` | SafeNavigate | 复杂环境 12；简单 4 |
| `DriveOnHeading speed` | XML | 局部生存低速 0.12m/s |

## 7.8 与 Nav2 默认树对比

| 特性 | nav2 默认 | Autonomy `navigate_to_pose.xml` |
|------|-----------|--------------------------------|
| 规划频率 | 1 Hz | 5 Hz |
| 局部生存模式 | 无 | ✅ TF 丢失时 RoundRobin |
| GoalReached 位置 | Pipeline 末端 | ReactiveFallback 首位 |
| 恢复链 | 清图 + Wait | 清图 + BackUp + Spin |

## 7.9 相关文档

- [数学原理 · 局部生存模式](03_math.md#39-局部生存模式时序)
- [BT 插件节点](08_bt_plugins.md)
- [多点导航 BT](08_bt_plugins.md#87-多点导航树-navigate_through_posesxml)
