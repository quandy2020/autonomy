(perception-overview)=
# 1. 模块概览

### 1.1 定位

| 维度 | 说明 |
|------|------|
| 感知层级 | 传感器融合 → 特征提取 → 语义/几何理解 |
| 输入 | `sensor_msgs`（激光 `LaserScan`、点云 `PointCloud2`、图像 `Image`、深度 `Depth` 等） |
| 输出 | `vision_msgs`（2D/3D 检测框、分类）、障碍列表、语义分割图（规划） |
| 上游 | `driver` / `sensor`、仿真器（Gazebo / nav_test） |
| 下游 | `prediction`（动态障碍轨迹）、`map`（costmap 更新）、`planning`（避障） |
| 对标 | Apollo Perception、ROS 2 `vision_msgs`、Nav2 costmap obstacle layer |

### 1.2 核心能力

| 能力 | 状态 | 说明 |
|------|------|------|
| `PerceptionServer` 服务骨架 | ⏳ 占位 | 类已声明，构造/析构与业务逻辑待实现 |
| 插件化感知接口 | ⏳ 待完成 | `PerceptionInterface` 仅含 `LoadOptions()` |
| Lua → Proto 配置管线 | ⏳ 空壳 | `PerceptionOptions` proto 无字段 |
| ONNX 视觉推理基础设施 | ✅ 已实现 | `autonomy/common/network/`（预处理、NMS、YOLO 解码） |
| `vision_msgs` 消息类型 | ✅ 部分 | 2D/3D 检测框、分类消息已定义 |
| 激光/点云障碍写入 costmap | ✅ 已实现 | `map/costmap_2d` 的 `ObstacleLayer` / `VoxelLayer` |
| 系统启动集成 | ❌ 未接入 | `Autonomy::Start()` 未构造 `PerceptionServer` |

> **当前阶段**：Perception 模块处于**扩展占位**阶段；实际导航栈中的障碍感知由 `map/costmap_2d` 承担，视觉推理基础设施在 `common/network` 中已就绪，等待 Perception 模块接线。

### 1.3 源码结构

```
autonomy/perception/
├── perception_server.*           # 感知服务入口（占位）
├── common/
│   └── perception_interface.*    # LoadOptions + 未来插件接口
└── proto/
    └── perception_options.proto  # 空 PerceptionOptions

# 相关基础设施（非 perception 包内）
autonomy/common/network/          # ONNX / TensorRT 推理管线
autonomy/commsgs/vision_msgs.*    # 感知输出消息
autonomy/map/costmap_2d/layers/   # ObstacleLayer, VoxelLayer
config/perception/perception.lua  # 空配置表
```

### 1.4 导航栈数据流

```
传感器 ──→ PerceptionServer ──→ vision_msgs / 障碍列表 ──→ Prediction
   │              │                                              │
   │              └──────────────→ map/costmap_2d ──────────────┘
   │                                      │
   └──────────────────────────────────────┴──→ Planning → Control
```

**当前实际数据流**（Perception 未启动时）：

```
激光/点云 ──→ ObstacleLayer ──→ local/global costmap ──→ Planning → Control
```

### 1.5 相关模块

| 模块 | 关系 |
|------|------|
| `autonomy/common/network` | ONNX 推理、图像预处理、检测后处理（NMS） |
| `autonomy/commsgs/vision_msgs` | 感知输出消息定义 |
| `autonomy/map/costmap_2d` | 激光/点云障碍层，当前承担主要障碍感知 |
| `autonomy/prediction` | 消费动态障碍，预测未来轨迹 |
| `autonomy/driver` | 相机 `forward_targets` 预留转发至 perception |
| `autonomy/system` | `AutonomyOptions.perception_options` 字段已预留 |

### 1.6 与业界方案对照

| 业界组件 | Autonomy 对应 | 备注 |
|----------|---------------|------|
| Apollo `PerceptionComponent` | `PerceptionServer` | 骨架阶段 |
| ROS 2 `vision_msgs` | `commsgs::vision_msgs` | 消息已对齐 |
| Nav2 `ObstacleLayer` | `map/costmap_2d/ObstacleLayer` | 已实现 |
| YOLO / ONNX Runtime | `common/network` | 推理管线就绪 |
| OpenPCDet / PointPillars | — | 待引入 |
