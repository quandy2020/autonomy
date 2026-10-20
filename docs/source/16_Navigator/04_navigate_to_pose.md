# 4. 单点导航行为树

文件：`config/navigator/behavior_tree/navigate_to_pose.xml`。

### 4.1 目标

| 目标 | 实现 |
|------|------|
| nav2 流水线 | 规划→平滑→验证→跟踪 |
| 定位韧性 | TF 丢失 → 局部生存 |
| 快速 SUCCESS | `GoalReached` 为 ReactiveFallback 首子 |
| 失败恢复 | 清图+后退+旋转，最多 8 次 |

### 4.2 树形结构

```text
RecoveryNode [SafeNavigate, retries=8]
├── ReactiveFallback [AdaptiveNavigation]
│ ├── GoalReached ← XY 到达 → SUCCESS
│ ├── Sequence [GlobalMode] ← TF 可用
│ │ ├── InitialPoseReceived
│ │ ├── TransformAvailable(map←base)
│ │ └── PipelineSequence
│ │ ├── Controller/Planner/Smoother Selector
│ │ ├── RateController(5Hz) → ComputePathToPose
│ │ ├── SmoothPath → IsPathValid → FollowPath
│ └── RecoveryNode [LocalSurvivalMode] ← TF 不可用
│ └── RoundRobin → DriveOnHeading / Spin / BackUp
│ + ClearEntireCostmap(local) + Wait
└── Sequence [NavigationRecovery]
├── ClearEntireCostmap(local+global)
├── BackUp(0.25m) → Spin(1.57)
```

### 4.3 关键分支

| 优先级 | 分支 | 条件 |
|--------|------|------|
| 1 | GoalReached | $d_{xy}\le\varepsilon_{xy}$（默认 0.25 m） |
| 2 | GlobalMode | 初位姿 + TF OK → Pipeline |
| 3 | LocalSurvival | TF 丢 → 相对运动 + 清局部图，待 TF 恢复 |

`SafeNavigate`：主分支 FAILURE → `NavigationRecovery` → 重试，最多 8 次。

### 4.4 调优

| 参数 | 位置 | 建议 |
|------|------|------|
| `goal_reached_tol` | common / navigator | 与 Control GoalChecker 对齐 |
| ComputePath Hz | RateController | 5（单点）/ 10（多点） |
| `local_survival_timeout` | navigator.lua | 默认 120 s |
| Recovery retries | XML | 默认 8 |

与 Nav2 默认树差异：增加 LocalSurvivalMode 与 GoalReached 前置。
