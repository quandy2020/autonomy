(navigator-overview)=
# 1. 模块概览

### 1.1 定位

| 维度 | 说明 |
|------|------|
| 编排层级 | 导航任务编排（Navigation Orchestration） |
| 输入 | 目标位姿 `PoseStamped`、多点航点序列、可选自定义 BT XML |
| 输出 | 导航结果/反馈、路径更新回调、恢复行为触发 |
| 上游 | `system::Autonomy`、Bridge gRPC、autolink Action Client |
| 下游 | `planning`（规划）、`control`（跟踪）、`map`（清图）、`transform`（TF） |
| 对标 | nav2_bt_navigator、nav2_behavior_tree |

Navigator **不实现**路径搜索或局部控制算法，而是通过 BT 节点调用各子系统服务，并管理恢复、重规划与任务生命周期。

### 1.2 核心能力

| 能力 | 状态 | 说明 |
|------|------|------|
| `NavigatorOptions` 配置管线 | ✅ | Lua → Protobuf，与 `common.lua` 共享帧与容差 |
| `NavigatorInterface` 生命周期接口 | ✅ | Idle / Running / Completed / Failed / Canceled |
| `BehaviorTreeNavigator` 模板基类 | ✅ | 对标 nav2 `BehaviorTreeNavigator` |
| `NavigatorMuxer` 互斥调度 | ✅ | 同一时刻仅允许一个 Navigator 活跃 |
| BT XML（单点 / 多点） | ✅ | `navigate_to_pose.xml`、`navigate_through_poses.xml` |
| BT 插件清单（52 个） | ⏳ | `navigator.lua` 已配置，插件 `.so` 待迁回 |
| `BtEngine` / `BtActionServer` | ⏳ | 头文件引用存在，实现待恢复 |
| `BtNavigator` 顶层编排 | ⏳ | 设计完成，源码待恢复 |
| `Autonomy` BT 导航路径 | ⏳ | 当前 `use_bt_navigation_ = false`，走直驱规划 |

> **当前阶段**：配置、接口骨架、BT XML 已就绪；完整 BT 栈（引擎 + 52 插件 + 具体 Navigator）尚待从历史 `tasks` 模块迁回。`system::Autonomy` 当前以 `NavigateDirectToPose` 直驱 `PlannerServer::GetPlan()`。详见 [§5 架构 · 实现状态](05_architecture.md#52-实现状态)。

### 1.3 源码结构

**当前工作区（已实现）**

```
autonomy/navigator/
├── common/
│   ├── interface.hpp                 # NavigatorInterface
│   └── behavior_tree_navigator.hpp   # BehaviorTreeNavigator + NavigatorMuxer
├── constants.hpp                     # 节点名、导航模式、运行时常量
├── options.hpp / options.cpp         # Lua → NavigatorOptions
└── proto/
    └── navigator_options.proto       # 配置 Protobuf
```

**设计目标（待恢复）**

```
autonomy/navigator/
├── behavior_tree/
│   ├── bt_engine.*                   # BT.CPP 工厂与 tick 循环
│   ├── bt_action_server.hpp          # Action ↔ BT 桥接
│   ├── bt_context.hpp                # 共享 planner/controller/TF
│   └── plugins/                      # 52 个 BT 节点插件
└── navigators/
    ├── bt_navigator.*                # 顶层 BtNavigator
    ├── navigate_to_pose_navigator.*
    └── navigate_through_poses_navigator.*
```

**配置与 BT 定义**

```
config/navigator/
├── navigator.lua
└── behavior_tree/
    ├── navigate_to_pose.xml
    ├── navigate_through_poses.xml
    └── autonomy_tree_nodes.xml
```

### 1.4 导航栈数据流

```
用户目标 ──→ Navigator BT ──→ ComputePath* ──→ planning
                  │                │
                  │                └──→ SmoothPath ──→ smoother
                  │
                  ├──→ IsPathValid ──→ planner
                  │
                  └──→ FollowPath ──→ control ──→ cmd_vel ──→ 底盘
                           ↑
                    TF (map→base_link) ← localization
```

### 1.5 相关模块

| 模块 | 关系 |
|------|------|
| `autonomy/planning` | BT 节点 `ComputePathToPose` 调用 `PlannerServer` |
| `autonomy/control` | BT 节点 `FollowPath`、`Spin`、`BackUp` 调用 `ControllerServer` |
| `autonomy/map/costmap_2d` | `ClearEntireCostmap`、`IsPathValid` 依赖代价地图 |
| `autonomy/localization` | 间接通过 TF 影响 `TransformAvailable` |
| `autonomy/system` | `Autonomy::NavigateToPose` 为顶层入口 |

### 1.6 与 Nav2 对照

| Nav2 组件 | Autonomy 对应 | 备注 |
|-----------|---------------|------|
| `nav2_bt_navigator::BtNavigator` | `navigator::BtNavigator` | 待恢复 |
| `nav2_core::BehaviorTreeNavigator` | `navigator::BehaviorTreeNavigator` | ✅ |
| `nav2_core::NavigatorMuxer` | `navigator::NavigatorMuxer` | ✅ |
| `navigate_to_pose` action | `NavigateToPoseAction` | proto 已定义 |
