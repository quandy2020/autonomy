# 7. commsgs 集成

业务消息与配置 proto 的定义层；Autolink 只负责序列化传输，不定义域语义。 commsgs 定义域语义与内存布局，Autolink 负责传输序列化；Bridge/Foxglove 也要对准同一套 struct/proto。新增字段先改 proto 再同步 C++，避免 wire 与 struct 不一致。

## 角色

| 层次 | 用途 |
|------|------|
| 配置 | `AutonomyOptions` 及子 Options |
| Server API | `Path`、`PoseStamped`、`TwistStamped` 等 |
| Autolink | Writer/Reader/Action 载荷 |
| ROS 2 Bridge | commsgs ↔ rosidl |

## 目录（示意）

```text
commsgs/ → core · geometry · sensor · map · planning · nav …
```

边界：算法内用 C++ struct，跨进程 `ToProto()`。

## 与 Autolink

Autolink **不**定义业务消息。示例：

```text
PlannerServer ──Writer<Path>──► /plan ──► 订阅方
──ActionServer──► compute_path
```

Framework 不依赖 rclcpp；ROS 互操作经 [Bridge](../15_Bridge/00_guide.md)。
