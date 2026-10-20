# 0. Commsgs 指南

## 背景

编译过、对端却解不出消息，或和 ROS 字段对不上——问题常在 **`autonomy/commsgs` 边界**：C++ struct 与 Protobuf 是否成对、ToProto 是否仍是 stub。怎么建 Node、怎么发 channel 在 [03 Communication](../03_Communication/00_guide.md)；本章管「消息长什么样、转得过不过」。

## 思路

- 语义靠近 ROS 2 `common_msgs`；Autolink 边界用 **ToProto / FromProto**。  
- **struct 与 proto 成对** — 漏一边会在边界上静默失败或链接报错。  
- **部分 ToProto 仍是 stub** — 联调前看 [09 综述](09_survey.md)、[03 Schema](03_schema.md)。  
- **RPC 消息在 automsgs.rpcs** — gRPC 字段去 [15 Bridge](../15_Bridge/00_guide.md)，不要和 commsgs 导航消息混搜。


## 步骤

### 1. 按问题选页

| 文档 | 解决什么 |
|------|----------|
| [01 概览](01_overview.md) | 包划分与设计原则 |
| [02 快速开始](02_quickstart.md) | 最小编解码示例 |
| [03 Schema](03_schema.md) | 字段与约定 |
| [04 用法](04_usage.md) | ToProto/FromProto 模式 |
| [05 架构](05_architecture.md) | 分层与生成物 |
| [06 核心消息](06_core_msgs.md) | 基础类型 |
| [07 传感地图](07_sensor_map_msgs.md) | 传感与地图 |
| [08 导航规划](08_nav_planning_msgs.md) | Path / OccupancyGrid 等 |
| [09 综述](09_survey.md) | 完整度、stub 说明 |

**加一种内部消息** → [03](03_schema.md) + [04](04_usage.md)。  
**查 Path / OccupancyGrid 字段** → [08](08_nav_planning_msgs.md) / [07](07_sensor_map_msgs.md)。

### 2. 常见坑

| 现象 | 先查 |
|------|------|
| 编译过、对端解不出 | 是否走了 stub ToProto |
| 和 ROS 字段对不上 | 语义对齐但不是 bit 级兼容 |
| 找 gRPC 字段 | Bridge / automsgs.rpcs |
