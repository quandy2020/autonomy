(prediction-overview)=
# 1. 模块概览

### 1.1 定位

| 维度 | 说明 |
|------|------|
| 预测层级 | 动态障碍运动学/行为预测（Trajectory / Behavior Prediction） |
| 输入 | 障碍列表、`vision_msgs` 检测、历史轨迹、地图/车道信息 |
| 输出 | 预测轨迹集 $\{\hat{\mathcal{T}}_i\}$、意图/行为标签、碰撞风险评分 |
| 上游 | `perception`（检测/跟踪）、`localization`（自车位姿）、`map`（静态约束） |
| 下游 | `planning`（交互式路径规划）、`control`（避障机动） |
| 对标 | Apollo Prediction、Waymo Motion Forecasting、ROS 2 预测生态 |

### 1.2 核心能力

| 能力 | 状态 | 说明 |
|------|------|------|
| `PredictionServer` 服务骨架 | ⏳ 占位 | 继承 `PredictionInterface`，构造/析构未实现 |
| `PredictionInterface` 基类 | ⏳ 占位 | 仅虚析构 + `LoadOptions()` |
| Lua → Proto 配置管线 | ⏳ 空壳 | `PredictionOptions` proto 无字段 |
| 运动模型库 | ✅ 部分 | `common/motion_model/`（Linear, DiffDrive, Stationary） |
| 卡尔曼滤波 | ✅ 部分 | `common/state_estimation/kalman_filter/` |
| 系统启动集成 | ❌ 未接入 | `Autonomy::Start()` 未构造 `PredictionServer` |

> **当前阶段**：Prediction 模块处于**扩展占位**阶段；运动模型与卡尔曼基础设施在 `common/` 中已就绪，等待 Prediction 模块接线。

### 1.3 源码结构

```
autonomy/prediction/
├── prediction_server.*           # 预测服务入口（占位）
├── common/
│   └── prediction_interface.*    # 基类 + LoadOptions
└── proto/
    └── prediction_options.proto  # 空 PredictionOptions

# 相关基础设施
autonomy/common/motion_model/     # 运动模型（CV, DiffDrive, Stationary）
autonomy/common/state_estimation/ # 卡尔曼滤波
config/prediction/prediction.lua  # 空配置表
```

### 1.4 导航栈数据流

```
Perception ──→ PredictionServer ──→ 预测轨迹 ──→ Planning
     ↑                ↑                              │
  检测/跟踪      运动模型 + 行为推理                   │
     │                ↑                              ▼
Localization ─────────┴──────────────────────→ Control
```

**当前实际数据流**（Prediction 未启动时）：

```
Perception(未启动) / costmap ──→ Planning ──→ Control
```

静态障碍由 costmap 处理；动态障碍预测尚未接入。

### 1.5 相关模块

| 模块 | 关系 |
|------|------|
| `autonomy/perception` | 提供检测/跟踪结果 |
| `autonomy/common/motion_model` | 轨迹前向传播的运动学模型 |
| `autonomy/planning` | 消费预测轨迹做交互规划 |
| `autonomy/system` | `AutonomyOptions.prediction_options` 字段已预留 |

### 1.6 与业界方案对照

| 业界组件 | Autonomy 对应 | 备注 |
|----------|---------------|------|
| Apollo `PredictionComponent` | `PredictionServer` | 骨架阶段 |
| Waymo VectorNet | — | 待引入 |
| CTRV / CV 模型 | `common/motion_model` | 部分就绪 |
| TEB + prediction | Control 调研提及 | 非已实现功能 |
