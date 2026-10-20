# 2. 快速开始

**什么时候用：** 本机第一次从 launch 到「发出一个导航 Goal」。BT 细节见 [16 Navigator](../16_Navigator/00_guide.md)；Bridge 见 [15](../15_Bridge/00_guide.md)。

### 1. 起栈

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
autolink_launch autonomy.launch
```

确认 `autonomy.task`、`autonomy.planning`、`autonomy.control` 等进程在（具体列表以 launch 为准）。

### 2. 发令（二选一）

| 方式 | 说明 |
|------|------|
| Bridge | `NavigationService/Navigate`，先 `GetCapabilities` |
| Action Client | 对 task 暴露的 nav action（与 commsg 定义一致） |

没有进程内 `CreateAutonomy` 单进程入口。

### 3. 配置前提

| 项 | 典型路径 |
|----|----------|
| Navigator conf | `autonomy/task/conf/navigator.pb.txt` |
| BT XML | `autonomy/task/conf/behavior_tree/navigation/` |
| 帧名 / 容差 | 各模块 `common` 段 |

`global_frame` / `robot_base_frame` / 默认 `planner_id`、`controller_id` 必须与 planning、control 一致。

### 常见坑

| 现象 | 先查 |
|------|------|
| task 起但立即退 | `--conf=`；BT 插件路径 |
| Bridge 成功车不动 | planning/control/map 是否同 launch |
| 树加载失败 | `AUTONOMY_BT_PLUGIN_PATH`；XML 路径 |
