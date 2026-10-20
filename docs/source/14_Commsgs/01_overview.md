(commsgs-overview)=
# 1. 模块概览

| 维度 | 说明 |
|------|------|
| 输出 | C++ struct、`.proto`、`ToProto`/`FromProto` |
| 下游 | planning、map、control、localization、transform、bridge |
| 类似 | ROS 2 common_msgs、nav2_msgs、vision_msgs |
| 序列化 | Protobuf（非 rosidl/CDR） |

### 1.2 能力

| 能力 | 状态 |
|------|------|
| 16 消息包 struct + proto | — |
| ToProto/FromProto | 核心较全，传感器/地图等部分 stub |
| PointCloud2 迭代器 | — |
| Nav2 Action/Service、error_code | proto |
| ROS 序列化兼容 | — |

### 1.3 源码

```
autonomy/commsgs/
├── proto/*.proto # 16 包 + error_code
├── <pkg>_msgs.{hpp,cpp} # struct + 转换
├── point_cloud2_iterator.hpp
└── impl/
```

### 1.4 数据流

```
sensor_msgs → map/perception → OccupancyGrid → Path → Control
geometry_msgs (TF) 贯穿；Autolink 边界 → proto → 网络
```

### 1.5 消息包一览

| 包 | 用途 | 完整度 |
|----|------|--------|
| builtin / std / geometry | 时间、Header、位姿/TF | 高 |
| sensor / map / planning | 激光/点云、栅格、Path | 中 |
| nav_msgs | SpeedLimit；Action 在 proto | — |
| vision / visualization | 检测、Marker | / stub |
| trajectory / diagnostic / stereo / shape | 辅助 | stub / 低 |
| pcl / vehicle | 空或占位 | — |
| error_code | 分段错误码 | proto |

### 1.6 参见
