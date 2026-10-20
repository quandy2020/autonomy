# 5. Prediction 模块架构

评估「预测模块还要补什么」或画 Perception→Planning 未来图；**不是**当前运行拓扑。

说明 `autonomy/prediction/` 目录里有什么、真正可复用的算法在哪（`common/motion_model`）。

| 目的 | 文档 |
|------|------|
| 写前向传播 | [§6](06_motion_models.md) |
| 看公式 | [§3](03_math.md) |
| 确认能不能上车 | [§1](01_overview.md)、[§7](07_survey.md) |

### 现状（对照代码）

| 组件 | 说明 |
|------|------|
| `prediction_server.hpp` | 类定义，继承空 `PredictionInterface` |
| `prediction_server.cpp` | 空翻译单元，无 `Start`/订阅 |
| `prediction_options.proto` | 配置字段预留 |
| `common/motion_model` | **已实现**，见 [§6](06_motion_models.md) |
| `Autonomy::Start()` 等主进程 | **未** `#include` / 构造 PredictionServer |

### 目标分层（未闭合）

```text
Perception 检测/跟踪 → 关联 + 滤波 → MotionModel 多步 → 轨迹 → Planning
```

当前 Planning 输入仍是 costmap + Path，无预测轨迹 channel。

### 目录角色（规划 vs 已有）

| 层 | 规划组件 | 已有替代 |
|----|----------|----------|
| 应用 | Planning 消费动态轨迹 | costmap |
| 服务 | `PredictionServer` | 无 |
| 算法 | Predictor 插件 | motion_model 单步 |
| 基础 | KF | `kalman_filter/` |
| 输入 | Detection2DArray | base tracks（未接到 prediction） |

### 数据流（目标）

```text
Detection2DArray → TrackManager → MotionModel::predict → PredictedTrajectory[] → Planning
```

TrackManager、Predictor 插件、Writer **均不在** `autonomy/prediction/` 现树中。

### 常见坑

| 误解 | 事实 |
|------|------|
| prediction 目录里有跟踪 | 只有 server 壳 + interface |
| Lua 里 ctrv 会生效 | 无读取与 Predictor |
| 必须等预测才能导航 | 否，costmap 即可 |

### 延伸阅读

[§7 综述](07_survey.md) · [10 Perception](../10_Perception/00_guide.md)
