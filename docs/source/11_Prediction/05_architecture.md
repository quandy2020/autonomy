# 5. Prediction 模块架构

### 5.1 实现状态

| 组件 | 状态 |
|------|------|
| `PredictionServer` / Interface / Proto | — |
| `common/motion_model`、卡尔曼 | — |
| Autonomy 接入 | — |

### 5.2 分层

| 层 | 组件 | 职责 |
|----|------|------|
| 应用 | Planning / Control | 轨迹、碰撞风险 |
| 服务 | `PredictionServer` | 关联、跟踪、前向传播 |
| 算法 | Predictor 插件 | CV / CTRV / Interactive / ML（规划） |
| 基础 | motion_model + Kalman | 状态传播、滤波 |
| 输入 | Perception / Localization | 检测、自车位姿 |

### 5.3 数据流

```
Detection2DArray → TrackManager → MotionModel::predict → PredictedTrajectory[] → Planning
```

### 5.4 插件接口（规划）

```cpp
class PredictionInterface {
    virtual void Configure(const PredictionOptions& opts) = 0;
    virtual void Update(const Detection2DArray& detections) = 0;
    virtual std::vector<PredictedTrajectory> Predict(double horizon) = 0;
};
```

### 5.5 路线图

| 阶段 | 内容 |
|------|------|
| P0 | Proto + Lua + Autonomy |
| P1 | CV/CTRV + 卡尔曼跟踪 |
| P2 | 对接 Perception |
| P3–P4 | 交互式 / 学习型预测 |

### 5.6 参见
