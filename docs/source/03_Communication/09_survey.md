# 9. 通信框架综述

本章从宏观视角对比 autolink 与业界方案，并给出 Autonomy 项目中的选型建议。

## 9.1 框架定位

autolink 是 Autonomy 的**自研通信运行时**，继承 Apollo Cyber RT 的核心设计，面向：

- **车内多进程**算法协同（感知、规划、控制）
- **Protobuf** 统一序列化，与 `commsgs` 无缝对接
- **组件化 + DAG** 部署，支持动态库热插拔
- **确定性调度**，满足软实时控制环路

它不是 ROS 2 的替代发行版，而是**去 ROS 运行时依赖**后的等价通信层。

## 9.2 与 ROS 2 对比

| 维度 | ROS 2 (rclcpp + rmw) | autolink |
|------|----------------------|----------|
| 发现 | DDS 标准发现 | 自研 UDP + 管理器 |
| 消息 | rosidl 生成 | protobuf + commsgs struct |
| 组件模型 | Node + 多线程 Executor | Node + Component + Scheduler |
| 长任务 | rclcpp_action | `autolink::action` |
| 录制 | rosbag2 (sqlite/mcap) | autolink record 分段文件 |
| 参数 | rclcpp::Parameter | ParameterServer |
| 生态 | 极丰富 | Autonomy 内部 + Bridge 对外 |
| 学习曲线 | 社区资料多 | 与 Cyber RT 文档相近 |

**迁移要点**：概念一一对应，主要差异在消息类型（`commsgs` vs `rosidl`）与启动方式（`mainboard`+DAG vs `ros2 launch`）。

## 9.3 与 Apollo Cyber RT 对比

| 维度 | Cyber RT | autolink |
|------|----------|----------|
| 代码 lineage | 原版 | fork / 演进自 Cyber |
| API 风格 | 高度一致 | `autolink::` 命名空间 |
| 构建 | Bazel 为主 | CMake + 可选 Bazel 示例 |
| Action | 无内建 | **扩展** Action 支持 |
| 文档语言 | 中英 | 子模块中文文档 + 本章 |

阅读 Cyber RT 官方文档时，可将 `cyber::` 替换为 `autolink::` 理解绝大部分 API。

## 9.4 与 DDS 的关系

autolink **不是**纯 DDS 封装：

- 默认同机通信走 **SHM**，绕过 DDS 数据路径以降低延迟
- RTPS 模式底层使用 **FastDDS** 作为跨机传输
- 发现协议为**自研**，与 DDS 发现独立

这种「混合栈」在自动驾驶中间件中常见（Cyber、部分 OEM 自研方案）。

## 9.5 Autonomy 栈通信全景

```
┌──────────────────────────────────────────────────────────┐
│ Layer 4: 应用算法                                         │
│  localization · mapping · planning · control · perception │
└────────────────────────────┬─────────────────────────────┘
                             │
┌────────────────────────────▼─────────────────────────────┐
│ Layer 3: 消息语义 (commsgs)                               │
│  ROS 对齐 struct + proto schema                           │
└────────────────────────────┬─────────────────────────────┘
                             │
┌────────────────────────────▼─────────────────────────────┐
│ Layer 2: 通信运行时 (autolink)  ← 本章                     │
│  Node · Channel · Service · Action · Scheduler            │
└────────────────────────────┬─────────────────────────────┘
                             │
┌────────────────────────────▼─────────────────────────────┐
│ Layer 1: 桥接 (Bridge)                                    │
│  gRPC · MQTT ↔ 外部云 / ROS / 仿真                        │
└──────────────────────────────────────────────────────────┘
```

## 9.6 通信模式选型指南

| 场景 | 推荐模式 | 理由 |
|------|----------|------|
| 传感器流（IMU、激光） | Channel + SHM | 高频、单向 |
| 地图 / 点云 | Channel + SHM | 大 payload |
| 速度指令 cmd_vel | Channel, depth=1 | 只取最新 |
| 查询地图元数据 | Service | 短 RPC |
| 跟随路径 FollowPath | Action | 长任务 + 取消 |
| 导航到点 | Action | 反馈 + 结果 |
| 运行时改参 | Parameter | 全局 KV |
| 离线分析 | Record → Play | 复现 |

## 9.7 性能考量摘要

| 因素 | 影响 | 对策 |
|------|------|------|
| 跨 NUMA 访问 | 延迟抖动 | classic group 隔离 |
| 线程竞争 | 回调饥饿 | choreography 绑核 |
| 序列化 | CPU 占用 | Arena、POD 字段设计 |
| 队列积压 | 控制滞后 | depth=1 |
| 发现风暴 | 启动慢 | 固定拓扑、减少动态注册 |

## 9.8 工具链一览

| 工具 | 功能 |
|------|------|
| `mainboard` | 加载 DAG 启动进程 |
| `autolink_launch` | 解析 launch 启多模块 |
| `autolink_monitor` | 实时消息监控 |
| `autolink_channel` | 拓扑查询 |
| `autolink_recorder` | 录制 / 回放 |
| Python `autolink` 包 | 脚本与原型 |

详见 `autolink/docs/source/autolink_developer_tools.md`。

## 9.9 进一步阅读

| 资源 | 路径 |
|------|------|
| 子模块快速开始 | `autolink/docs/source/autolink_quick_start_cn.md` |
| 开发者 API | `autolink/docs/source/autolink_api_for_developers.md` |
| 调度详解 | `autolink/docs/source/autolink_scheduler_cn.md` |
| Python API | `autolink/docs/source/autolink_python_api_cn.md` |
| 消息体系 | [commsgs §9](../14_Commsgs/09_survey.md) |
| 外部桥接 | [Bridge §9](../15_Bridge/08_survey.md) |
| Framework 集成 | [Framework](../05_Framework/index.rst) |

## 9.10 演进方向（展望）

| 方向 | 说明 |
|------|------|
| ROS 2 Bridge 完善 | 通过 Bridge 模块双向映射 topic/action |
| 零拷贝 commsgs | 扩展 traits 支持 loaned message |
| 安全通信 | TLS / 认证（跨机 RTPS） |
| 可观测性 | 与 profiler、event 模块深度集成 |

> 实现状态以源码为准；本章综述随版本迭代更新。
