(commsgs-overview)=
# 1. 模块概览

第一次接触 commsgs，或要在 struct 与 proto 之间选型。

`autonomy/commsgs` 提供 **C++ struct + `.proto` + ToProto/FromProto**，语义靠近 ROS 2 common_msgs / nav2_msgs / vision_msgs，线上为 Protobuf（非 rosidl/CDR）。

**包怎么选（联调把握）**

| 包 | 建议 |
|----|------|
| builtin / std / geometry | 默认优先，转换较完整 |
| sensor / map / planning | 常用；大包体与部分字段需实测 |
| nav_msgs | Action 用 proto；SpeedLimit 等看 [§8](08_nav_planning_msgs.md) |
| vision | 视觉联调前往返测 |
| visualization / trajectory / diagnostic | 多 stub，见 [§9](09_survey.md) |
| pcl / vehicle | 占位，勿依赖 |

struct 与 proto **成对维护**；RPC 探索等字段在 **automsgs.rpcs**，见 [15 Bridge](../15_Bridge/00_guide.md)。

### 数据流角色

```text
算法 struct →（边界）proto → Autolink / gRPC
传感器 → map / perception → OccupancyGrid、Path → control
geometry / TF 贯穿各域
```

### 源码布局

```text
autonomy/commsgs/
├── proto/*.proto
├── <pkg>_msgs.{hpp,cpp}
├── point_cloud2_iterator.hpp
└── impl/
```

### 常见坑

| 现象 | 说明 |
|------|------|
| 有 hpp 就能过网络 | 可能 stub ToProto |
| 与 ROS bit 级一致 | 否，经 bridge 转换 |

### 延伸阅读

[00 指南](00_guide.md) · [§4 用法](04_usage.md)
