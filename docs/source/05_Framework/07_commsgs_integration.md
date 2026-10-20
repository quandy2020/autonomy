# 7. commsgs 集成

Protobuf 消息层：配置、Server API、Autolink 载荷。

## 7.1 角色

| 层次 | 用途 |
|------|------|
| 配置 | `AutonomyOptions` 及子 Options |
| Server API | `Path`、`PoseStamped`、`TwistStamped` 等 |
| Autolink | Writer/Reader/Action 序列化 |
| ROS 2 Bridge | commsgs ↔ rosidl |

## 7.2 目录（示意）

```text
commsgs/ → core · geometry · sensor · map · planning · nav …
```

边界：算法内用 C++ struct，跨进程 `ToProto()`。

## 7.3 与 Autolink

Autolink **不**定义业务消息。示例：

```text
PlannerServer ──Writer<Path>──► /plan ──► 订阅方
──ActionServer──► compute_path
```

Framework 不依赖 rclcpp；ROS 互操作经 [Bridge](../15_Bridge/00_guide.md)。
