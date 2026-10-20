(navigator-guide)=
# 0. Navigator 指南

## 背景

Action 成功但车不动、一直 Recovery、到点不结束——很多在 **行为树编排层**：BT 调度 ComputePath → FollowPath → Recovery，不算路径、不发电机指令。实现主路径在 `autonomy/task/navigation/`（文档章名仍叫 Navigator）；任务类型与 TaskServer 见 [17 Tasks](../17_Tasks/00_guide.md)。

## 思路

- 角色接近 Nav2 `bt_navigator`；**GoalReached 与 FollowPath 内 GoalChecker 容差可能不同**（XY-only vs XY+yaw，见 [04 NavigateToPose](04_navigate_to_pose.md)）。  
- **TF 丢了会进生存模式** — 超时与行为在 conf，别当成「规划坏了」。  
- BT 插件 `.so` 依赖 **`AUTONOMY_BT_PLUGIN_PATH`** 指到 `build/lib`。  
- `global_frame` / `robot_base_frame` / 默认 planner·controller id 必须与 08/09 一致。


## 步骤

### 1. 按问题选页

| 文档 | 解决什么 |
|------|----------|
| [01 架构](01_architecture.md) | 编排与域进程关系 |
| [02 BT 算法](02_bt_algorithms.md) | 树怎么设计 |
| [03 BT 引擎](03_bt_engine.md) | 引擎与 tick |
| [04 NavigateToPose](04_navigate_to_pose.md) | 主任务行为（常翻） |
| [05 插件](05_bt_plugins.md) | 插件清单 |
| [06 综述](06_survey.md) | 边界与对照 |

**改 NavigateToPose 树** → [04](04_navigate_to_pose.md)。  
**加 BT 插件** → [05](05_bt_plugins.md)。  
**从 Bridge 发目标** → [15 Bridge](../15_Bridge/00_guide.md) + Tasks。

### 2. 最短启动

1. 配帧、容差、BT XML、插件列表（`autonomy/task/conf/navigator.pb.txt`）  
2. BT XML 在 `autonomy/task/conf/behavior_tree/navigation/`  
3. `autolink_launch autonomy.launch`，经 Bridge / Action 发 Goal  

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
autolink_launch autonomy.launch
```

### 3. 常见坑

| 现象 | 先查 |
|------|------|
| 树加载失败 | XML 路径；plugin_lib_names；`.so` 路径 |
| 一直 Recovery | costmap / TF / 控制是否失败 |
| 到点不结束 | GoalReached 容差 vs FollowPath GoalChecker |
| 和 task 文档对不上目录 | 实现多在 `autonomy/task/navigation/` |
