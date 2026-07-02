# 5. Control 模块架构设计

本文描述 `autonomy/control` 的逻辑架构、核心组件关系与运行时数据流。

## 5.1 设计目标

Control 模块遵循以下设计原则：

1. **与 nav2 对齐**：接口语义、插件模式、FollowPath 循环与 Navigation2 `nav2_controller` 保持一致
2. **插件化扩展**：局部控制器以 `ControllerInterface` 插件形式注册，支持进程内与动态库两种加载
3. **配置驱动**：Lua → Protobuf 配置管线，与 planning/map 模块统一
4. **Checker 解耦**：Goal / Progress Checker 独立于控制器，便于组合与测试
5. **速度后处理**：VelocitySmoother 作为可选下游，约束加速度与 deadband

## 5.2 实现状态

| 组件 | 实现度 | 说明 |
|------|--------|------|
| `ControllerServer` 构造/生命周期 | ✅ | costmap、odom、TF 初始化 |
| FollowPath 控制循环 | ⏳ | `ComputeControl()` 等为 stub |
| 插件加载 | ⏳ | 成员已声明，加载逻辑待参照 PlannerServer |
| Goal Checker ×3 | ✅ | 算法完整，配置接线待完成 |
| Progress Checker ×2 | ✅ | 算法完整，时间窗口待实现 |
| VelocitySmoother | ✅ | 算法完整，pub/sub 待接线 |
| Controller 插件 | ❌ | 无具体实现类 |
| CheckerOptions Lua 加载 | ⏳ | proto 已定义，C++ 未引用 |

## 5.3 分层架构

<div class="plan-arch-diagram">

  <div class="plan-arch-layer plan-arch-app">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">应用层</span>
      <span class="plan-arch-title">Navigator / 行为树</span>
      <span class="plan-arch-sub">外部调用方，不隶属于 control 包</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-body-block">
        <div class="nav-body-label">对外接口</div>
        <div class="nav-chip-list">
          <span class="nav-chip">FollowPath</span>
          <span class="nav-chip">ComputeControl()</span>
          <span class="nav-chip">SetSpeedLimit()</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>Action / 服务调用</span></div>

  <div class="plan-arch-layer plan-arch-server">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">服务层</span>
      <span class="plan-arch-title">ControllerServer</span>
      <span class="plan-arch-sub">模块唯一对外服务入口 · <code>controller_server</code> 节点</span>
    </div>
    <div class="plan-arch-body plan-arch-body-cols">
      <div class="nav-body-block">
        <div class="nav-body-label">核心职责</div>
        <ul>
          <li>插件加载、注册与按 <code>controller_id</code> 调度</li>
          <li>@ <code>controller_frequency</code> 循环计算速度</li>
          <li>Goal / Progress Checker 集成</li>
          <li>路径更新、失败容忍、零速发布</li>
        </ul>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">关键话题</div>
        <div class="nav-chip-list">
          <span class="nav-chip">cmd_vel</span>
          <span class="nav-chip">odom</span>
          <span class="nav-chip">/local_costmap</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>调度控制请求</span></div>

  <div class="plan-arch-split">
    <div class="plan-arch-layer plan-arch-plugin">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">算法层</span>
        <span class="plan-arch-title">ControllerInterface 插件</span>
      </div>
      <div class="plan-arch-body">
        <div class="nav-body-block">
          <div class="nav-body-label">规划插件（配置预留）</div>
          <div class="nav-chip-list">
            <span class="nav-chip">GracefulController</span>
            <span class="nav-chip">MppiController</span>
            <span class="nav-chip">DWB / RPP …</span>
          </div>
        </div>
        <div class="nav-body-block">
          <div class="nav-body-label">核心 API</div>
          <div class="nav-chip-list">
            <span class="nav-chip">ComputeVelocityCommands()</span>
            <span class="nav-chip">SetPlan()</span>
          </div>
        </div>
      </div>
    </div>

    <div class="plan-arch-link">
      <span class="plan-arch-link-text">共享</span>
      <span class="plan-arch-link-arrow">↔</span>
    </div>

    <div class="plan-arch-layer plan-arch-map">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">地图层</span>
        <span class="plan-arch-title">Costmap2DWrapper</span>
      </div>
      <div class="plan-arch-body">
        <div class="nav-body-block">
          <div class="nav-body-label">模式</div>
          <div class="nav-chip-list">
            <span class="nav-chip">独立 local costmap</span>
            <span class="nav-chip">共享 global costmap</span>
          </div>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>辅助判定</span></div>

  <div class="plan-arch-split">
    <div class="plan-arch-layer plan-arch-post">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">判定层</span>
        <span class="plan-arch-title">Checker 插件</span>
      </div>
      <div class="plan-arch-body">
        <div class="nav-body-block">
          <div class="nav-body-label">Goal Checker</div>
          <div class="nav-chip-list">
            <span class="nav-chip">SimpleGoalChecker</span>
            <span class="nav-chip">PositionGoalChecker</span>
            <span class="nav-chip">StoppedGoalChecker</span>
          </div>
        </div>
        <div class="nav-body-block">
          <div class="nav-body-label">Progress Checker</div>
          <div class="nav-chip-list">
            <span class="nav-chip">SimpleProgressChecker</span>
            <span class="nav-chip">PoseProgressChecker</span>
          </div>
        </div>
      </div>
    </div>

    <div class="plan-arch-link">
      <span class="plan-arch-link-text">平滑</span>
      <span class="plan-arch-link-arrow">→</span>
    </div>

    <div class="plan-arch-layer plan-arch-post">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">后处理层</span>
        <span class="plan-arch-title">VelocitySmoother</span>
      </div>
      <div class="plan-arch-body">
        <div class="nav-body-block">
          <div class="nav-body-label">功能</div>
          <div class="nav-chip-list">
            <span class="nav-chip">加速度约束</span>
            <span class="nav-chip">deadband</span>
            <span class="nav-chip">OPEN/CLOSED_LOOP</span>
          </div>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe plan-arch-pipe-out"><span>cmd_vel → 底盘 / 仿真</span></div>

</div>

## 5.4 FollowPath 控制循环（设计）

```
ComputeControl()
│
├─ 初始化：SetPlan(path), Reset checkers, Reset controller
│
└─ while (not canceled and not goal_reached)
     │
     ├─ UpdateGlobalPath()          // 接收路径更新
     │
     ├─ GetRobotPose(pose)          // odom 优先，fallback costmap
     │
     ├─ progress_checker->Check(pose)
     │    └─ false → throw FailedToMakeProgress
     │
     ├─ ComputeAndPublishVelocity()
     │    ├─ controller->ComputeVelocityCommands(pose, vel, cmd_vel, goal_checker)
     │    ├─ 检查 ControllerResultCode
     │    └─ 发布 cmd_vel（可选经 VelocitySmoother）
     │
     └─ IsGoalReached()
          └─ goal_checker->IsGoalReached(pose, end_pose, velocity)
│
└─ OnGoalExit()                     // 清零速度、Reset 状态
```

**频率控制**：循环周期 $T = 1 / f_{ctrl}$，`controller_frequency` 默认 20 Hz。

**失败容忍**：若连续 $t > T_{\mathrm{fail}}$ 无法生成有效命令，终止 FollowPath。$T_{\mathrm{fail}}$ 对应 `failure_tolerance`。

## 5.5 类关系

```
ControllerServer
├── controllers_: map<string, ControllerInterface::SharedPtr>
├── goal_checkers_: map<string, GoalChecker::SharedPtr>
├── progress_checkers_: map<string, ProgressChecker::SharedPtr>
├── costmap_wrapper_: Costmap2DWrapper
├── odom_smoother_: OdomSmoother
└── tf_buffer_: transform::Buffer

ControllerInterface (abstract)
├── ComputeVelocityCommands()
├── SetPlan()
└── SetSpeedLimit()

GoalChecker (abstract)
├── IsGoalReached()
└── GetTolerances()

ProgressChecker (abstract)
└── Check()
```

## 5.6 数据流时序

```
Planning          ControllerServer       ControllerPlugin      Checker
   │                     │                      │                  │
   │──── Path ──────────→│ SetPlan()            │                  │
   │                     │─────────────────────→│                  │
   │                     │                      │                  │
   │                     │←── odom ── Sensor    │                  │
   │                     │                      │                  │
   │                     │── @ 20Hz ───────────→│ ComputeVel()     │
   │                     │                      │── goal_checker ─→│
   │                     │←──── cmd_vel ────────│                  │
   │                     │── publish cmd_vel ──→ 底盘              │
   │                     │                      │                  │
   │                     │── IsGoalReached ─────────────────────→│
   │                     │←── true ──────────────────────────────│
   │                     │ OnGoalExit()         │                  │
```

## 5.7 配置管线

```
controller.lua
    │
    ├─ AUTONOMY_CONTROLLER table
    │
    ▼
LuaParameterDictionary
    │
    ▼
control::LoadOptions()
    │
    ▼
proto::ControllerOptions
    │
    ├─ controller_frequency, failure_tolerance
    ├─ controller_plugins[]
    ├─ costmap_2d_options（待键名统一）
    │
    ▼
ControllerServer(options)
```

**待扩展**：`CheckerOptions`、`VelocitySmootherOptions` 从 Lua 加载并注入对应组件。

## 5.8 异常体系

| 异常类 | 触发条件 |
|--------|----------|
| `InvalidController` | 插件 id 无效 |
| `ControllerTFError` | TF 查询失败 |
| `FailedToMakeProgress` | ProgressChecker 返回 false |
| `PatienceExceeded` | 超时无进度（Nav2 语义，待实现） |
| `InvalidPath` | 空路径 |
| `NoValidControl` | 控制器返回无效命令 |
| `ControllerTimedOut` | costmap 未更新 |

## 5.9 插件加载设计

参照 `PlannerServer` 已实现的模式：

```cpp
// 1. 解析 "graceful_controller:GracefulController"
auto [id, type] = ParsePluginSpec(spec);

// 2. 注册（进程内）
plugin_manager.RegisterInProcessClass<GracefulController>(type);

// 3. 加载外部 .so（可选）
plugin_manager.LoadPlugin(library_path);

// 4. 创建实例
auto controller = plugin_manager.CreateInstance<ControllerInterface>(
    type, options, id, tf_buffer, costmap_wrapper);
controllers_[id] = controller;
```

Goal / Progress Checker 同理，以独立插件 map 管理。

## 5.10 与 Nav2 架构映射

| Nav2 | Autonomy | 差异 |
|------|----------|------|
| ROS2 LifecycleNode | autolink 节点 | 无 lifecycle，Start/Shutdown 手动 |
| pluginlib | Autolink PluginManager | planning 已用，control 待接 |
| rclcpp::Rate | 自实现 sleep | 基于 `controller_frequency` |
| nav2_core::Controller | ControllerInterface | API 等价 |
| bond / heartbeat | 无 | 可选扩展 |

## 5.11 后续开发路线

1. **P0**：实现 `ComputeControl()` 完整循环 + cmd_vel 发布
2. **P0**：参照 PlannerServer 实现插件加载
3. **P1**：CheckerOptions / VelocitySmootherOptions Lua 接线
4. **P1**：实现 GracefulController 或 MPPI 首个插件
5. **P2**：FollowPath Action Server 与 Navigator BT 打通
6. **P2**：修复 `costmap` vs `costmap_2d_options` 键名
