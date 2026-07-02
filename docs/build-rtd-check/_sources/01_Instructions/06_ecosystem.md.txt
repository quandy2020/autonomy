# 6. 生态集成

Autonomy 以独立 C++ 框架为核心，通过消息兼容层与桥接模块与外部生态互操作。

## 6.1 集成方式总览

| 方式 | 组件 | 用途 | 状态 |
|------|------|------|------|
| 进程内 API | `system::Autonomy` | 嵌入式 / 单机部署 | ✅ |
| Autolink 通信 | `autolink` | 模块间 DDS 通信 | ✅ |
| ROS 2 消息兼容 | `commsgs` | 与 ROS 消息结构对齐 | ✅ |
| gRPC Bridge | `bridge` | 云端 / 远程调度 | ⏳ |
| ROS 2 节点包装 | 外部 `autonomy_ros`（可选） | RViz2 / ros2 CLI | 视部署而定 |

## 6.2 commsgs 消息层

`autonomy/commsgs` 提供与 ROS 2 对齐的消息类型（`geometry_msgs`、`nav_msgs`、`planning_msgs` 等），算法模块使用 C++ struct，跨进程时经 Protobuf 序列化。

```
算法模块 (C++ struct)
      │ commsgs::ToProto / FromProto
      ▼
autolink Writer/Reader
      │ DDS / 共享内存 / 进程内
      ▼
其他模块 / 桥接层
```

详见 [14 Commsgs](../14_Commsgs/00_guide.md)。

## 6.3 Autolink 通信

Autolink 是 Autonomy 的分布式通信运行时，提供：

- **Channel**（Topic）：发布/订阅
- **Service/Client**：请求/响应 RPC
- **Action**：长时间任务（Goal / Feedback / Result）

对标 Cyber RT / ROS 2 rclcpp。详见 [03 Communication](../03_Communication/00_guide.md)。

## 6.4 gRPC Bridge

`autonomy/bridge` 在启用 `BUILD_GRPC=ON` 时提供 gRPC 服务，支持远程下发导航指令、查询状态等。

| 客户端 | 状态 |
|--------|------|
| Map / Planning / Control stub | 部分实现 |
| Navigator stub | 待实现 |

详见 [15 Bridge](../15_Bridge/00_guide.md)。

## 6.5 ROS 2 互操作说明

### 6.5.1 设计定位

- Autonomy **核心不依赖** ROS 2 运行时即可编译链接
- `commsgs` 消息字段与 ROS 2 对应类型语义一致，便于转换
- 可选 ROS 2 包装层用于：RViz2 可视化、`ros2 bag` 录放、与现有 ROS 2 栈共存

### 6.5.2 典型集成模式

| 模式 | 描述 |
|------|------|
| 纯 Autonomy | 单进程 `Autonomy`，无 ROS 2 |
| 混合部署 | Autonomy 核心 + ROS 2 桥接节点转发 Topic |
| 开发调试 | Docker 内 ROS 2 Humble + Autonomy 同容器编译 |

ROS 2 环境变量与运行示例见 [04 Running §6](../04_Running/06_ros2_integration.md)。

## 6.6 可视化

| 工具 | 接入方式 |
|------|----------|
| 内置可视化模块 | [13 Visualization](../13_Visualization/index.rst) |
| RViz2 | 经 ROS 2 桥接 / Topic 转发 |
| Foxglove | 经 ROS 2 / 自定义桥接 |

## 6.7 仿真

仿真集成见 [12 Simulation](../12_Simulation/index.rst)。

## 6.8 相关文档

- [03 Communication](../03_Communication/00_guide.md)
- [15 Bridge](../15_Bridge/00_guide.md)
- [05 Framework](../05_Framework/index.rst)
