# 13. 通信框架综述

与 ROS 2 / Cyber RT 对比及通信模式选型。用法与示例见 [§0](00_guide.md)；模块专题见 §2–§12。

| 本文 §13 | 相关文档 |
|---------|----------|
| 对比、选型 | [§0 指南](00_guide.md) · [§1 架构](01_architecture.md) |

---

## 13.1 框架定位

继承 Cyber RT 设计（Component + DAG + SHM），CMake 构建，扩展 Action 与 Plugin，与 `commsgs` 深度集成。非 ROS 发行版，而是 Autonomy 内置通信层。

## 13.2 与 ROS 2 / Cyber RT

| 维度 | ROS 2 | Cyber RT | autolink |
|------|-------|----------|----------|
| 发现 | DDS | 自研 UDP | 自研 UDP |
| 消息 | rosidl | protobuf | protobuf + commsgs |
| 发布订阅 | topic | channel | channel |
| RPC | Service | Service | Service |
| 长任务 | Action | — | Action |
| 参数 | Parameter | — | Parameter |
| 模块形态 | Node + launch | Component + DAG | 同左 + Launch |
| 插件 | pluginlib | — | PluginManager |
| 构建 | colcon | Bazel | CMake |

`cyber::` 与 `autolink::` API 大体可互换。跨栈互通走 [Bridge](../15_Bridge/index.rst)。

## 13.3 通信模式选型

| 场景 | 推荐 | 文档 |
|------|------|------|
| 传感器、路径、cmd_vel | Channel（`pending_queue_size=1`） | [§3](03_channel.md) |
| 一次性查询 | Service | [§4](04_service.md) |
| 导航长任务、可取消 | Action | [§5](05_action.md) |
| 运行时改参 | Parameter | [§6](06_parameter.md) |
| 可插拔规划器/控制器 | Plugin | [§7](07_plugin.md) |
| 消息时间戳、稳频循环 | Time / Rate | [§11](11_timer.md) |
| 无输入周期任务 | TimerComponent | [§11](11_timer.md) |
| 生产算法模块 | Component + DAG | [§10](10_component.md) |
| 多进程编排 | Launch + mainboard | [§9](09_launch.md) |
| 离线复现 / 联调 | Record / Play | [§3.9](03_channel.md#39-调试工具) |
| 仿真时钟 | Clock `MODE_MOCK` | [§11.3](11_timer.md#113-clockc) |

## 13.4 Binary vs Component

| | Binary | Component + DAG |
|---|--------|-----------------|
| 开发速度 | 快，适合验证 | 需写 `.so` + DAG |
| 可配置性 | channel 硬编码 | DAG 改配置即可 |
| 调度 | 自建 `Rate` 循环 | Scheduler 统一调度 |
| 典型用途 | examples、工具 | Autonomy Server、车载部署 |

## 13.5 性能要点

- 同机默认 **SHM**；跨机 **RTPS**
- 控制环 **`pending_queue_size=1`**，避免队列堆积
- 延迟敏感链路：**choreography 绑核**（[§12](12_scheduler.md)）
- `Proc` / Reader 回调内**避免阻塞 I/O**
- 热路径**少打日志**（[§8](08_log.md)）

## 13.6 延伸阅读

| 资源 | 路径 |
|------|------|
| 快速开始 | `autolink/docs/source/autolink_quick_start_cn.md` |
| API 手册 | `autolink/docs/source/autolink_api_for_developers.md` |
| 调度 | `autolink/docs/source/autolink_scheduler_cn.md` |
| POD 消息 | `autolink/docs/source/autolink_pod_message_cn.md` |
| C++ 示例 | `autolink/examples/cpp/README.md` |
| Python 示例 | `autolink/examples/python/README.md` |
| 消息体系 | [commsgs 综述](../14_Commsgs/09_survey.md) |

---

**导航**：[← §12 Scheduler](12_scheduler.md) · [§0 指南](00_guide.md)
