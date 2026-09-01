# Task

任务编排层：订阅目标通道 / Action，调度行为树，驱动 planning / control 完成导航等任务。

## 一键启动（planning + control + task）

`launch/task.launch` 会同时拉起三个进程：

| 进程 | 二进制 | 作用 |
|------|--------|------|
| planning | `autonomy.planning` | 全局规划（`compute_path_to_pose` 等） |
| control | `autonomy.control` | 局部跟踪（`follow_path` 等） |
| task | `autonomy.task` | TaskServer + BT（默认 `navigate_to_pose.xml`） |

### 前置条件

- 已编译，且 `PATH` 能找到 `autonomy.planning` / `autonomy.control` / `autonomy.task`
- 配置可读：`config/autonomy.lua`、`config/task/**`（相对当前工作目录）
- 运行时有 TF（至少 `map` → `base_link`）与代价地图所需输入

### 启动（colcon / Docker：`/workspace/autonomy`）

工作区根目录下 **先进入源码包**，再设 launch 搜索路径（否则会报 `Cannot find launch file`）：

```bash
cd /workspace/autonomy/src/autonomy

export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/task/launch

autolink launch start task.launch
```

也可不设 `AUTOLINK_LAUNCH_PATH`，直接给绝对路径：

```bash
cd /workspace/autonomy/src/autonomy
export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
autolink launch start $PWD/autonomy/task/launch/task.launch
```

停止：

```bash
autolink launch stop task.launch
```

不要与 `autonomy/system/launch/autonomy.launch` 同时开（会重复拉起 planning / control / task）。

### 仅起 task（已有 planning / control）

```bash
cd /workspace/autonomy/src/autonomy
export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
autonomy.task --config_directory=config
```

## Autoviz 点目标（A→B 导航）

1. 按上文启动 `task.launch`
2. 打开 Autoviz，**fixed_frame = `map`**
3. 工具 **2D Goal Pose**（默认 Topic `/goal_pose`）
4. 地面点击并拖拽朝向 → 发布 `PoseStamped` 到 `/goal_pose`
5. TaskServer 转为 `NavigationGoal(START, SINGLE_POSE)` → 加载  
   `config/task/behavior_tree/navigation/navigate_to_pose.xml`

接口名见 `common/names.hpp`：`kGoalPose`、`navigate_to_pose`、`navigate_through_poses`。

## 相关路径

| 路径 | 说明 |
|------|------|
| `launch/task.launch` | 一键启动文件 |
| `config/task/behavior_tree/` | 各域 BT XML（以 XML 节点名为准） |
| `task_server.*` / `scheduler/` | 任务入口与互斥调度 |
| `navigation/` | 导航任务与 BT 插件 |
