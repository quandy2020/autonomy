# 02 · 架构

## 0. 需求基线

功能/非功能需求、验收标准和优先级见 [01-requirements.md](01-requirements.md)。
本文只解释满足这些需求的架构手段。

## 1. 设计目标

`autonomy/navigation` 是 end-to-end 导航子模块，输入深度/里程计/IMU/地图，
输出安全的速度指令。核心约束：

- VNTPA 用位姿索引关键帧作为环境表示；simulation 策略可以消费已校验 OccupancyGrid，执行可达 frontier 搜索与空闲栅格路径跟随。
- Component 只做 autolink 适配，业务逻辑全部在 `Runner` 与各库中，便于无框架单测。
- 安全链独立进程、唯一 `/cmd_vel` 出口，编译期禁用异常/RTTI。
- 所有插件通过 `PluginManager` 工厂注册，配置切换不重新编译。
- 安全失败关闭：非法输入/超时/模型失败一律零速。

## 2. 进程拓扑

```text
navigation process
├── ObservationBuilderComponent   (depth 触发, ~30 Hz)
├── NavigationComponent           (Observation 触发, 决策 5 Hz)
│   └── /navigate_to_pose action server
│   └── Runner
│       ├── ObservationEncoder
│       ├── ContextProvider / memory
│       ├── PolicyInterface worker (single-flight)
│       ├── ChunkBlender
│       └── ScenarioManager (scenario -> stage -> rules)
└── BehaviorComponent             (10 Hz, 默认 XML/恢复请求/heartbeat)

safety process
└── SafetyComponent               (50 Hz)
    └── SafetyShieldImplementation
        ├── ReflexPolicyArbiter
        ├── CbfFilter
        ├── AebVelocitySampler
        ├── VelocityFeasibilityFilter
        └── Watchdog / zero-speed fallback

guardian process
└── GuardianComponent             (2 Hz)
    ├── HeartbeatMonitor
    ├── DegradationPolicy
    └── Degradation writer
```

三份 DAG 见 `dag/navigation.dag`、`dag/safety.dag`、`dag/guardian.dag`；
launch 见 `launch/*.launch`。

## 3. 分层

| 层 | 目录 | 职责 | 允许依赖 |
| --- | --- | --- | --- |
| 值类型/基础设施 | `common/` | Pose/Twist、Status、配置、平台模型、时间 | automsgs、autonomy/common |
| 感知 | `perception/` | 深度预处理、DeFM 编码、池化、不确定性、Observation 融合 | common |
| 记忆 | `memory/` | 体素、关键帧过滤、位姿索引、快照、漂移、语义图 | common |
| 规划 | `planning/` | 插件接口、VNTPA/VINT/NOMAD、场景、任务、规则、Blender | common、perception、memory |
| 安全 | `safety/` | 三级链、反射、CBF、AEB、可行性、看门狗 | common、planning 接口 |
| 行为 | `behavior/` | 默认 XML、BT 节点、会话快照与恢复请求 | planning、memory、guardian |
| 守护 | `guardian/` | 心跳、故障分类、降级、诊断 | common、lifecycle proto |
| 入口 | `navigation_component.*` | autolink 薄适配 + Runner 装配 | 全部 |

依赖方向单向向上；`safety` 不依赖 `perception`，只消费 `ReflexObservation`。

## 4. Component / Runner 分离

### Component

- 只负责：创建 reader/writer、解析配置、调用业务对象、发布状态/心跳。
- 不包含算法；可在没有 autolink 的环境用 fake 替代。
- 注册宏：`AUTOLINK_REGISTER_COMPONENT(namespace::Class)`。

### Runner

- 业务核心；构造注入 `Dependencies`（encoder/memory/policy/blender/scenario）。
- `Runner::Create(config)` 通过 `PluginManager` 装配默认管线。
- 生命周期：`Start()` → `Tick(observation, pose, velocity)` → `Stop()`。
- `Tick` 不等待策略推理；worker 只接收最新 job（single-flight），结果按 sequence 丢弃过期项。
- 线程：Tick/SetGoal/Stop 在导航线程；worker 调用策略的 Query，并在工作版本变化时 Reset，以隔离取消/抢占前的策略缓存。

```text
Tick:
  encode(observation) -> feature
  memory.Insert(keyframe)
  TryTakeResult() -> if chunk: blender.Push
  PostJobIfDue()  -> worker.Query(observation, feature, goal, context)
  blender.Consume(dt) -> command
  ScenarioManager.Plan(frame) -> rules -> final command
```

## 5. 插件体系

`PluginManager` 是进程级单例，三类工厂：

- `EncoderFactory(name, config)` → `ObservationEncoder`
- `PolicyFactory(name, config)` → `PolicyInterface`
- `MemoryFactory(name, options)` → `ContextProvider`

注册入口：

- `RegisterEnginePlugins()`：DeFM/VNTPA Engine 适配器（Engine 适配器随导航库编译，需模型）。
- `RegisterSimulationPlugins()`：`DepthStatisticsEncoder` + `SimulationPolicy`，无模型仿真用。

配置只写插件名与参数；`Runner::Create` 负责查找/构造/Configure。

## 5.1 BehaviorTree 节点规范

`behavior/nodes/` 与 `autonomy/task/behavior_tree` 保持一致：

- 每个节点 `hpp` 声明类，`cpp` 实现 + `BT_REGISTER_NODES` 注册；
- 条件节点派生 `NavigationCondition`（`BT::ConditionNode`）；
- 立即动作派生 `NavigationSyncAction`（`BT::SyncActionNode`）；
- 长动作使用 `BT::StatefulActionNode` 基类（`NavigationStatefulAction`，后续任务节点使用）；
- XML 节点 ID 与 `factory.registerNodeType<T>("ID")` 一致；
- 节点通过 blackboard `node_context` 读取 `NodeContext`，不直接持有 Runner。

当前已注册：`CheckPoseHealth`、`CheckModuleHealth`、`CheckSafetyIntervention`、
`CheckGoalReachable`、`CheckRelocalization`、`StartNavigation`、`RequestOperatorAssist`。

## 6. 线程与所有权

| 对象 | 线程 | 所有权 |
| --- | --- | --- |
| Observation / DepthFeature | 导航线程 | 值拷贝/消息 |
| `ContextView` | 导航线程读取 | 不可变 shared batch |
| `PolicyInterface` | worker | Runner 持有 |
| `ChunkBlender` | 导航线程 | Runner 持有 |
| `SafetyShieldImplementation` | safety timer 线程 | SafetyComponent 持有 |
| Guardian supervisor | guardian timer 线程 | GuardianComponent 持有 |

`Runner` 用 mutex 保护 goal/blender 状态；worker 不做内存写入，只返回 `PolicyResult`。

## 7. 生命周期与失败语义

- `Runner::Start` 检查依赖非空；失败返回 false，component Init 失败。
- `Tick` 无目标/停止/编码失败 → `nullopt`，调用方按零速处理。
- 安全链失败：`EmergencyStop` 或零速；`SafetyResult.reason` 记录原因。
- 模型/引擎失败：policy 返回 `Unavailable`，blender 不推新 chunk，最终零速。
- Guardian 失联：`DEGRADATION_MODE_REFLEX_ONLY` 或 `SAFE_STOP`。

## 8. 扩展点

| 需求 | 扩展方式 |
| --- | --- |
| 新策略 | 实现 `PolicyInterface`，注册 `RegisterPolicy`，配置 `policy.name` |
| 新编码器 | 实现 `ObservationEncoder`，注册 `RegisterEncoder` |
| 新记忆 | 实现 `ContextProvider`，注册 `RegisterMemory` |
| 新场景 | 实现 `ScenarioBase` + `StageBase`，加入 `ScenarioManager` |
| 新规则 | 实现 `RuleBase`，加入 `RuleOptions.enabled_rules` |
| 新底盘 | 实现 `DriveModel`，配置 `platform_profile` |
| 新安全层 | 在 `SafetyShieldImplementation` 串联，保持 fail-closed |

## 9. 已知边界

- `BehaviorComponent` 已加载默认 XML，通过受会话/目标/次数/新鲜度约束的请求驱动 Runner 局部恢复。
  保留障碍绕行、取消/重建及耗尽终态已实测；复杂/动态场景与其他任务树见 [19](19-behavior-recovery-validation.md) 的边界。
- 仿真策略是 goal-seek + frontier，不含全局路径规划。
- Engine 适配器需要 `autonomy::common` + ONNX/TensorRT；默认仿真不启用。
- autolink 仅 Linux；macOS 只能编译核心库/单测。
