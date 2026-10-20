# 4. 单点导航行为树

**默认 XML：** `autonomy/task/conf/behavior_tree/navigation/navigate_to_pose.xml`。

**什么时候改：** 到点判据、Recovery 太 aggressive、或 TF 丢时行为不对。

BT 在 autonomy/task/；TaskServer 编排。

### 设计目标

| 目标 | 做法 |
|------|------|
| Nav2 主链 | 规划 → 平滑 → 验证 → FollowPath |
| TF 丢 | LocalSurvival 相对运动 + 清局部 costmap |
| 早停 | `GoalReached` 放在 ReactiveFallback 最前 |
| 失败 | RecoveryNode：清图 + 后退 + 旋转，次数见 XML |

### 结构（摘要）

```text
RecoveryNode [SafeNavigate]
├── ReactiveFallback
│   ├── GoalReached
│   ├── Sequence [GlobalMode] → Pipeline：Selector → Rate(ComputePath) → Smooth → Valid → Follow
│   └── LocalSurvival → RoundRobin(DriveOnHeading / Spin / BackUp) + 清局部图
└── NavigationRecovery（清图、BackUp、Spin）
```

### 分支优先级

| 顺序 | 分支 | 条件 |
|------|------|------|
| 1 | GoalReached | XY 容差（常与 navigator conf 一致，**不一定**等于 control GoalChecker 的 yaw） |
| 2 | GlobalMode | 初位姿 + TF OK |
| 3 | LocalSurvival | TF 不可用 |

### 调优

| 参数 | 位置 | 提示 |
|------|------|------|
| `goal_reached_tol` | common / navigator pb | 与 [control GoalChecker](../../09_Control/00_guide.md) 一起对 |
| ComputePath 频率 | XML RateController | 单点常见 5 Hz |
| `local_survival_timeout` | navigator 配置 | 默认量级约 120 s |
| Recovery retries | XML | 默认 8 |

与 Nav2 默认树差异：**LocalSurvival** + **GoalReached 前置**。
