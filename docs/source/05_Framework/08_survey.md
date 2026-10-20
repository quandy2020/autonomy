# 8. 综述

和 Nav2 对照、哪些能力已经定型、哪些入口已删除。不确定该改 Framework 还是 Communication 时先看这里。

## 框架里已经定型的部分

- **多进程 launch**：`autolink_launch autonomy.launch`
- **共享配置快照**：`CreateOptions` / `AutonomyOptions`（`autonomy/system/options.*`、`autonomy.pb.txt`）
- **三大导航 Server**：Map / Planner / Controller + 插件
- **TransformServer**：静态 TF
- **TaskServer + BT**：行为树与 Action 细节见 [16 Navigator](../16_Navigator/00_guide.md)

进程内整栈 `CreateAutonomy` 已删除，不要再找单进程聚合入口。

## 和 Nav2 差在哪

| | Autonomy | Nav2 |
|--|----------|------|
| 配置 | Protobuf / 文本 conf | YAML + 参数服务器 |
| 通信 | Autolink + commsgs | ROS 2 DDS |
| 编排 | TaskServer + BT | BehaviorTree.CPP |
| 插件 | PluginManager + `.so` | pluginlib |
| 起栈 | `autolink_launch` + `*_main` | `nav2_bringup` |

## 使用上的限制

1. 场外发令一般要 Bridge 或 Action，没有「进程内 commander」默认路径。
2. 以多进程为主，排错要按进程查。
3. 通信 API 细节在 [03 Communication](../03_Communication/index.rst)。

## 源码入口

| 路径 | 用途 |
|------|------|
| `autonomy/system/options.*` | CreateOptions |
| `autonomy/system/launch/autonomy.launch` | 多进程入口 |
| `autonomy/system/conf/autonomy.pb.txt` | 共享快照 |
| `autonomy/{planning,control,task}/*_main.cpp` | 各域进程 |
