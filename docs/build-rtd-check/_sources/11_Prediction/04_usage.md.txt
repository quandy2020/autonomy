(prediction-usage)=
# 4. 使用指南

### 4.1 配置

**入口文件**：`config/prediction/prediction.lua` → `config/autonomy.lua`

> **注意**：当前 `AUTONOMY_PREDICTION` 为空表，未挂入 `AUTONOMY` 根表。

**规划字段**：

| 字段 | 说明 | 默认建议 |
|------|------|----------|
| `enabled` | 是否启动 PredictionServer | `false` |
| `frequency` | 预测循环频率 (Hz) | `10.0` |
| `prediction_horizon` | 预测时域 $T_p$ (s) | `5.0` |
| `prediction_dt` | 预测步长 $\Delta t$ (s) | `0.1` |
| `motion_model` | 默认运动模型 | `"cv"` |
| `plugins` | 预测插件列表 | 待实现 |

### 4.2 PredictionServer API（规划）

| API | 用途 | 状态 |
|-----|------|------|
| `Start()` / `Shutdown()` | 启停预测循环 | ❌ |
| `OnDetections(detections)` | 接收感知检测 | ❌ |
| `OnObstacles(obstacles)` | 接收障碍列表 | ❌ |
| `GetPredictedTrajectories()` | 获取预测轨迹 | ❌ |
| `GetCollisionRisks()` | 获取碰撞风险评分 | ❌ |

### 4.3 运动模型 API（当前可用）

```cpp
#include "autonomy/common/motion_model/motion_model_interface.hpp"

// 统一接口
template <typename Derived, typename StateT>
class MotionModelInterface {
    StateT predict(const StateT& state, const std::chrono::nanoseconds& dt);
    typename StateT::Matrix jacobian(const StateT& state, const std::chrono::nanoseconds& dt);
};
```

| 模型 | 头文件 | 适用 |
|------|--------|------|
| `LinearMotionModel` | `linear_motion_model.hpp` | 行人、一般动态障碍 |
| `DifferentialDriveMotionModel` | `differential_drive_motion_model.hpp` | 差速机器人障碍 |
| `StationaryMotionModel` | `stationary_motion_model.hpp` | 静态障碍 |

### 4.4 预测轨迹消息（规划）

建议输出结构（待 `prediction_msgs` 定义）：

```cpp
struct PredictedTrajectory {
    int32_t obstacle_id;
    double probability;                    // 多模态时
    std::vector<PoseStamped> trajectory;   // 离散轨迹点
    std::vector<Covariance2D> uncertainty; // 可选不确定性
};
```

### 4.5 系统集成（规划）

```cpp
// system::Autonomy 构造流程（规划）
prediction_ = std::make_shared<prediction::PredictionServer>(options_.prediction_options());
prediction_->Start();

// Perception 回调
perception_->OnDetections([&](const Detection2DArray& dets) {
    prediction_->OnDetections(dets);
});

// Planning 查询
auto trajectories = prediction_->GetPredictedTrajectories();
planner_->SetDynamicObstacles(trajectories);
```

### 4.6 与 Planning 的接口（规划）

| 接口 | 说明 |
|------|------|
| `SetDynamicObstacles(trajectories)` | 注入预测轨迹供时空规划 |
| `GetOccupancyAt(t, x, y)` | 查询 $t$ 时刻 $(x,y)$ 是否被动态障碍占据 |
| TEB / DWA 扩展 | 在代价函数中加入动态障碍项 |

### 4.7 故障排查

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| PredictionServer 未启动 | 模块未接入 | 当前预期；静态规划仍可用 |
| 轨迹发散 | 运动模型不匹配 | 行人用 CV，车辆用 CTRV |
| 关联错误 | 检测 ID 跳变 | 启用匈牙利 + 门控 |
| 预测滞后 | frequency 过低 | 提高至 10–20 Hz |
