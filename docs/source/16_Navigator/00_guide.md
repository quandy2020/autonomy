(navigator-guide)=
# 0. Navigator 指南

`autonomy/navigator`：导航编排层，对齐 nav2 `bt_navigator`。用 BT 协调 planning / control / map / localization。

### 0.2 快速开始

1. 配置：`config/navigator/navigator.lua`（帧、容差、BT XML、插件）
2. BT XML：`config/navigator/behavior_tree/`
3. 启动：`autolink_launch autonomy.launch`，经 Bridge / Action 发令

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
autolink_launch autonomy.launch
```

`global_frame` / `robot_base_frame` / 默认 planner·controller id 须与 planning、control 一致。

### 0.3 形式化（摘要）

给定 $q_s,q_g\in\mathrm{SE}(2)$，编排层通过 BT 调度 ComputePath → FollowPath → Recovery，不直接解路径。 
`GoalReached` 仅检 XY（$\varepsilon_{xy}=0.25\,\mathrm{m}$）；`FollowPath` 内 GoalChecker 检 XY+航向。

TF 分支：GoalReached → GlobalMode（TF OK）→ LocalSurvivalMode（TF 丢失）。详见 [§4](04_navigate_to_pose.md)。

### 0.4 能力边界

| 能力 | 状态 |
|------|------|
| Options / Interface / Muxer / BT XML | — |
| 52 插件清单 + Groot 模型 | `.so` 演进中 |
| `BtEngine` / `BtActionServer` | — |
| 直驱 `NavigateDirectToPose` | — |

### 0.5 配置

| 字段 | 默认 |
|------|------|
| `bt_loop_duration` | 10 ms |
| `default_server_timeout` | 20000 ms |
| `local_survival_timeout` | 120 s |
| `goal_reached_tolerance` | 0.25 m |
| `plugin_lib_names` | 52 |
| `navigate_to_pose.behavior_tree_file` | `navigate_to_pose.xml` |

自定义 BT：改 XML → 更新 `behavior_tree_file` 或 Goal 字段 → 同步 `autonomy_tree_nodes.xml` 与 `plugin_lib_names`。 
插件路径：`export AUTONOMY_BT_PLUGIN_PATH=.../lib`。

### 0.6 故障排查

| 码 | 含义 | 处理 |
|----|------|------|
| 9001 | 未初始化 | 查 Configure |
| 9005 | 加载 BT 失败 | XML / `AUTONOMY_BT_PLUGIN_PATH` |
| 9006 | TF 错误 | localization / map→base |
| 9008–9010 | 规划/控制失败 | Planning / Control 排错 |
| 9013 | GoalChecker | 增大 yaw 容差 |
