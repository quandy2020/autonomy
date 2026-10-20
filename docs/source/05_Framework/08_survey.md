# 8. 综述

## 8.1 能力矩阵

| 能力 | 状态 | 说明 |
|------|------|------|
| 多进程 launch | — | `autolink_launch autonomy.launch` |
| `CreateOptions` / `AutonomyOptions` | — | 共享 conf 快照 |
| Map / Planner / Controller + 插件 | — | |
| TransformServer | — | 静态 TF |
| TaskServer + BT | / | 持续演进 |
| 进程内 `CreateAutonomy` | 已移除 | — |

## 8.2 vs Nav2

| 维度 | Autonomy Framework | Nav2 |
|------|-------------------|------|
| 配置 | Protobuf / 文本 conf | YAML + 参数服务器 |
| 通信 | Autolink + commsgs | ROS 2 DDS |
| 编排 | TaskServer + BT | BehaviorTree.CPP |
| 插件 | PluginManager + .so | pluginlib |
| 入口 | `autolink_launch` + `*_main` | `nav2_bringup` |

## 8.3 限制

1. 发令需 Bridge / Action 
2. 多进程为主，无单进程整栈聚合 
3. 通信 API 见 [03 Communication](../03_Communication/index.rst)

## 8.4 源码入口

| 路径 | 说明 |
|------|------|
| `autonomy/system/options.*` | `CreateOptions` |
| `autonomy/system/launch/autonomy.launch` | 多进程入口 |
| `autonomy/system/conf/autonomy.pb.txt` | 共享快照 |
| `autonomy/{planning,control,task}/*_main.cpp` | 各进程 |
