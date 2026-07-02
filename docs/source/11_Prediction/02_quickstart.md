# 2. 快速开始

### 2.1 当前可用能力

Prediction 模块本体尚未接入，但以下**基础设施**可独立使用：

| 能力 | 入口 | 说明 |
|------|------|------|
| 线性运动模型 | `common/motion_model/linear_motion_model.hpp` | CV/CA 状态前向传播 |
| 差速运动模型 | `common/motion_model/differential_drive_motion_model.hpp` | 非完整约束预测 |
| 静止模型 | `common/motion_model/stationary_motion_model.hpp` | 静态障碍假设 |
| 卡尔曼滤波 | `common/state_estimation/kalman_filter/` | 状态估计与预测步 |

### 2.2 运动模型直接使用（C++）

```cpp
#include "autonomy/common/motion_model/linear_motion_model.hpp"
#include "autonomy/common/state_vector/generic_state.hpp"

using namespace autonomy::common::motion_model;

// 定义 2D 位置+速度状态
using State2D = state_vector::GenericState<4>;  // x, y, vx, vy

LinearMotionModel<State2D> model;
State2D state;
state.Set(0, 1.0);  // x
state.Set(1, 2.0);  // y
state.Set(2, 0.5);  // vx
state.Set(3, 0.0);  // vy

auto dt = std::chrono::milliseconds(100);
State2D predicted = model.predict(state, dt);
// predicted: x' = x + vx*dt, y' = y + vy*dt
```

### 2.3 Prediction 模块（未来接入步骤）

1. 编辑 `config/prediction/prediction.lua`
2. 在 `config/autonomy.lua` 中设置 `prediction = AUTONOMY_PREDICTION`
3. 在 `system/options.cpp` 中加载 prediction 配置
4. 在 `Autonomy::Start()` 中构造并启动 `PredictionServer`

```lua
-- config/prediction/prediction.lua（规划示例）
AUTONOMY_PREDICTION = {
    enabled = true,
    frequency = 10.0,
    prediction_horizon = 5.0,      -- 预测时域 (s)
    prediction_dt = 0.1,           -- 预测步长 (s)
    plugins = {
        -- "cv_predictor:ConstantVelocityPredictor",
        -- "interactive:InteractivePredictor",
    },
    motion_model = "cv",           -- cv | ctrv | diffdrive | stationary
}
```

```lua
-- config/autonomy.lua
AUTONOMY = {
    prediction = AUTONOMY_PREDICTION,  -- 待实现后启用
}
```

### 2.4 与 Perception 的数据流（规划）

```
Perception.Detection2DArray
        │
        ▼
PredictionServer::OnDetections()
        │
        ├─ 数据关联（匈牙利）
        ├─ 卡尔曼更新
        ├─ 运动模型前向传播
        └─ PredictedTrajectory[] → Planning
```

需先完成 [10 Perception](../10_Perception/index.rst) 检测输出。

### 2.5 单元测试参考

运动模型已有单元测试：

```bash
# 构建后运行（若已接入测试目标）
./autonomy.common.motion_model.linear_motion_model_test
./autonomy.common.motion_model.differential_motion_model_test
./autonomy.common.motion_model.stationary_motion_model_test
```
