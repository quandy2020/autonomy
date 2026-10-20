# 13. 综述 Survey

## 13.1 ROS 2 / Cyber / autolink

| 维度 | ROS 2 | Cyber RT | autolink |
|------|-------|----------|----------|
| 发现 | DDS | 自研 UDP | 本机文件；跨机可选 FastDDS |
| 消息 | rosidl | protobuf | protobuf + commsgs |
| 发布订阅 | topic | channel | channel |
| RPC | Service | Service | Service |
| 长任务 | Action | — | Action |
| 参数 | Parameter | — | Parameter |
| 模块形态 | Node + launch | Component + DAG | 同左 + Launch |
| 插件 | pluginlib | — | PluginManager |
| 构建 | colcon | Bazel | CMake |

跨栈互通走 [Bridge](../15_Bridge/index.rst)。

## 13.2 模式选型

| 场景 | 推荐 | 文档 |
|------|------|------|
| 传感器 / 路径 / cmd_vel | Channel（`pending_queue_size=1`） | [§3](03_channel.md) |
| 一次性查询 | Service | [§4](04_service.md) |
| 长任务、可取消 | Action | [§5](05_action.md) |
| 运行时改参 | Parameter | [§6](06_parameter.md) |
| 可插拔算法 | Plugin | [§7](07_plugin.md) |
| 无输入周期任务 | TimerComponent | [§11](11_timer.md) |
| 生产算法模块 | Component + DAG | [§10](10_component.md) |
| 多进程编排 | Launch | [§9](09_launch.md) |
| 离线复现 | Record / Play | [§3](03_channel.md) |

## 13.3 Binary vs Component

| — | Binary | Component + DAG |
|---|--------|-----------------|
| 开发速度 | 快 | 需 `.so` + DAG |
| 可配置性 | 硬编码 | DAG 改配置 |
| 调度 | 自建 `Rate` | Scheduler |
| 用途 | 示例、工具 | Server、车载 |

## 13.4 性能

| 要点 | 做法 |
|------|------|
| 同机 | 默认 SHM |
| 跨机 | 可选 RTPS（[§14](14_fastdds.md)） |
| 控制环 | `pending_queue_size=1` |
| 延迟敏感 | choreography 绑核 |
| 回调 | 避免阻塞 I/O；少打日志 |
