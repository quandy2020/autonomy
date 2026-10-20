# 5. Commsgs 架构

新增消息包或查「谁消费 Path/OccupancyGrid」时看分层与构建方式。

说明 commsgs 在 Autonomy 里的位置：应用用 C++ struct，跨进程/Service 用 Protobuf。

struct 与 proto 成对；大包体联调前实测。

### 分层

| 层 | 内容 |
|----|------|
| 应用 | Planning / Map / Control / localization 等 |
| 消息 | `autonomy::commsgs::*` + ToProto/FromProto |
| 序列化 | `commsgs::proto`（`protoc` 生成 `*.pb.h`） |
| 传输 | Autolink Writer / Reader / Service / Action |

### 构建与依赖

```text
proto/<pkg>.proto → <pkg>_msgs.hpp → <pkg>_msgs.cpp → libautonomy
```

依赖方向：`geometry` ← `std` ← `builtin`；sensor/map/planning 依赖 geometry。nav Action 在 `nav_msgs.proto`，带 `error_code`。

无独立 commsgs 库目标；随主库 `GLOB` 编译。

### 运行时模式

| 模式 | 行为 |
|------|------|
| 同进程 | 传 struct 引用，无 proto |
| Pub/Sub | 框架内序列化 struct |
| Service/Action | 入口 FromProto，出口 ToProto |

### 消费关系（典型）

| 模块 | 常用消息 |
|------|----------|
| planning | Path, IsPathValid |
| map / costmap | OccupancyGrid, PointCloud2, Costmap |
| control | Twist, Path |
| transform | TransformStamped |
| bridge / ROS | 转换层，不全走 commsgs 直连 ROS |
| perception base | Image, vision_msgs |

### 扩展步骤

1. 增加 `proto/*.proto`  
2. 手写 `*_msgs.hpp` struct  
3. 实现 `*_msgs.cpp` 转换  
4. 在 [§9](09_survey.md) 自测往返

### 常见坑

| 现象 | 说明 |
|------|------|
| 只加 proto 不加 cpp | 链接或转换缺失 |
| Action 用错头文件 | 用 `proto/nav_msgs.pb.h` |
| 假定全包 ToProto 完整 | 见 [§9](09_survey.md) |

### 延伸阅读

[§3 Schema](03_schema.md) · [03 Communication](../03_Communication/00_guide.md)
