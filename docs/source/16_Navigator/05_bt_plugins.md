# 5. BT 插件节点

**目录：** `autonomy/task/behavior_tree/plugins/`（action / condition / control / decorator）。
命名：`autonomy_behavior_tree_{category}_{stem}`，与 `navigator.pb.txt` 里 `plugin_lib_names` 一致。  
环境变量：`AUTONOMY_BT_PLUGIN_PATH` → 通常为 `build/lib`。

加节点、改 XML，或 `plugin_lib_names` 对不上 `.so`。

BT 在 autonomy/task/；TaskServer 编排。

### 数量（约）

| 类别 | 数量（以 pb 列表为准） |
|------|------------------------|
| action | ~24 |
| condition | ~16 |
| control | 3 |
| decorator | ~7 |

### Action（代表）

| 节点 | 下游 | 主要端口 |
|------|------|----------|
| `ComputePathToPose` / `ComputePathThroughPoses` | Planner | `goal`/`goals`, `path`, `planner_id` |
| `SmoothPath` | Smoother | `unsmoothed_path`, `smoothed_path` |
| `FollowPath` | Controller | `path`, `controller_id`, `goal_checker_id` |
| `Spin` / `BackUp` / `DriveOnHeading` | Controller | 距离、速度、时限 |
| `ClearEntireCostmap` | map 服务 | local/global |
| `*Selector` | 选 planner/controller/checker | `selected_*` |
| `*Cancel` | 取消对应 Action | 通常无额外端口 |

### Condition（代表）

| 节点 | 判定 |
|------|------|
| `GoalReached` | XY 容差（与 FollowPath 内 checker 可能不同） |
| `TransformAvailable` | TF 可用 |
| `IsPathValid` / `IsStuck` | 路径与卡住 |
| `InitialPoseReceived` | 是否收到初定位 |

### Control / Decorator

| 类型 | 节点 |
|------|------|
| Control | `PipelineSequence`, `RecoveryNode`, `RoundRobin` |
| Decorator | `RateController`, `SingleTrigger`, … |

### 多点树

文件：`autonomy/task/conf/behavior_tree/navigation/navigate_through_poses.xml`  
流水线：ComputePathThroughPoses(更高频) → Smooth → IsPathValid → FollowPath + 航点修剪节点。  
**无** 单点树的 LocalSurvival / GoalReached 前置。

### 扩展步骤

1. 实现 BT 节点并 `BT_REGISTER_NODES`  
2. CMake 安装 `.so` 到 lib  
3. 追加 `plugin_lib_names`  
4. 更新 Groot / tree nodes XML（若有）  
5. 重启 `autonomy.task`
