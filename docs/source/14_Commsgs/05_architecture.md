# 5. Commsgs 架构

### 5.1 实现状态

| 组件 | 状态 |
|------|------|
| 16 proto + C++ struct | — |
| ToProto/FromProto | 核心全，部分 stub |
| PointCloud2 工具、Nav2 Action、error_code | — |
| 独立 CMake 目标 | （随 libautonomy） |

### 5.2 分层

| 层 | 内容 |
|----|------|
| 应用 | Planning/Map/Control… 用 C++ struct |
| 消息 | `autonomy::commsgs` + 转换 |
| 序列化 | `commsgs::proto`（*.pb.h） |
| 传输 | Autolink Writer/Reader/Service/Action |

### 5.3 文件与依赖

```
proto/<pkg>.proto → <pkg>_msgs.hpp → <pkg>_msgs.cpp
geometry → std → builtin；sensor/map/planning → geometry+std
nav_msgs.proto → planning + geometry + error_code
```

构建：`GLOB autonomy/*.cpp` + `protoc` → `libautonomy`。

### 5.4 运行时

| 模式 | 行为 |
|------|------|
| Pub/Sub | 应用写 struct；框架序列化 |
| Service/Action | 边界 proto ↔ struct |
| 同进程 | 直接传 struct，无转换 |

### 5.5 消费矩阵

| 模块 | 典型消息 |
|------|----------|
| planning | Path, IsPathValid |
| map | OccupancyGrid, PointCloud2 |
| control | Twist, Path, Pose |
| transform | TransformStamped |
| bridge | 全部 ↔ ROS |

### 5.6 扩展

| 场景 | 步骤 |
|------|------|
| 新类型/包 | proto → hpp → cpp |
| 新 Action / 错误码 | nav_msgs.proto / error_code.proto |
| ROS 互通 | bridge 转换 |

### 5.7 参见

- [§3](03_schema.md) · [§9](09_survey.md) · [03 Communication](../03_Communication/00_guide.md)
