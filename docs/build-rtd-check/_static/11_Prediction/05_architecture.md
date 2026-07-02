# 5. Prediction 模块架构设计

本文描述 `autonomy/prediction` 的逻辑架构、核心组件关系与运行时数据流。

## 5.1 设计目标

1. **插件化扩展**：各预测算法以 `PredictionInterface` 插件形式注册
2. **运动模型复用**：统一使用 `common/motion_model` 做轨迹前向传播
3. **多模态输出**：支持多条假设轨迹及概率，供规划器决策
4. **与 Perception 解耦**：通过标准消息接口接收检测/跟踪结果
5. **实时性**：预测频率 10–20 Hz，时域 3–8 s

## 5.2 实现状态

| 组件 | 实现度 | 说明 |
|------|--------|------|
| `PredictionServer` 声明 | ⏳ | 构造/析构未实现 |
| `PredictionInterface` | ⏳ | 仅 `LoadOptions()` |
| `PredictionOptions` proto | ⏳ | 空 message |
| `common/motion_model` | ✅ | Linear, DiffDrive, Stationary |
| 卡尔曼滤波 | ✅ | `state_estimation/kalman_filter` |
| 系统启动集成 | ❌ | 未接入 `Autonomy` |

## 5.3 分层架构

<div class="plan-arch-diagram">

  <div class="plan-arch-layer plan-arch-app">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">应用层</span>
      <span class="plan-arch-title">Planning / Control</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-chip-list">
        <span class="nav-chip">PredictedTrajectory</span>
        <span class="nav-chip">CollisionRisk</span>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>预测轨迹与风险</span></div>

  <div class="plan-arch-layer plan-arch-server">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">服务层</span>
      <span class="plan-arch-title">PredictionServer</span>
    </div>
    <div class="plan-arch-body plan-arch-body-cols">
      <div class="nav-body-block">
        <div class="nav-body-label">核心职责</div>
        <ul>
          <li>接收 Perception 检测/跟踪</li>
          <li>数据关联与轨迹维护</li>
          <li>运动模型前向传播</li>
          <li>多模态/行为预测（可选）</li>
        </ul>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">关键话题</div>
        <div class="nav-chip-list">
          <span class="nav-chip">/perception/detections</span>
          <span class="nav-chip">/prediction/trajectories</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>调度预测</span></div>

  <div class="plan-arch-split">
    <div class="plan-arch-layer plan-arch-plugin">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">算法层</span>
        <span class="plan-arch-title">Predictor 插件</span>
      </div>
      <div class="plan-arch-body">
        <div class="nav-chip-list">
          <span class="nav-chip">CVPredictor</span>
          <span class="nav-chip">CTRVPredictor</span>
          <span class="nav-chip">InteractivePredictor</span>
          <span class="nav-chip">MLPredictor</span>
        </div>
      </div>
    </div>

    <div class="plan-arch-link">
      <span class="plan-arch-link-text">复用</span>
      <span class="plan-arch-link-arrow">↔</span>
    </div>

    <div class="plan-arch-layer plan-arch-map">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">基础层</span>
        <span class="plan-arch-title">motion_model + Kalman</span>
      </div>
      <div class="plan-arch-body">
        <div class="nav-chip-list">
          <span class="nav-chip">LinearMotionModel</span>
          <span class="nav-chip">KalmanFilter</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>检测/跟踪输入</span></div>

  <div class="plan-arch-layer plan-arch-post">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">输入层</span>
      <span class="plan-arch-title">Perception / Localization</span>
    </div>
  </div>

</div>

## 5.4 运行时数据流

```
Perception.Detection2DArray
        │
        ▼
PredictionServer
    ├─ TrackManager（关联 + 卡尔曼）
    ├─ MotionModel::predict（多步前向）
    └─ PredictedTrajectory[] ──→ Planning
```

## 5.5 插件接口设计（规划）

```cpp
class PredictionInterface {
public:
    virtual void Configure(const PredictionOptions& opts) = 0;
    virtual void Update(const Detection2DArray& detections) = 0;
    virtual std::vector<PredictedTrajectory> Predict(double horizon) = 0;
};
```

## 5.6 扩展路线图

| 阶段 | 内容 |
|------|------|
| P0 | `PredictionOptions` + Lua 加载 + `Autonomy` 接入 |
| P1 | CV/CTRV 预测器 + 卡尔曼跟踪 |
| P2 | 与 Perception 消息对接 |
| P3 | 交互式预测（考虑自车动作） |
| P4 | 学习型预测（VectorNet 等） |
