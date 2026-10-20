# 2. 快速开始

### 2.1 可用能力

| 能力 | 入口 |
|------|------|
| Linear / DiffDrive / Stationary | `common/motion_model/*.hpp` |
| 卡尔曼 | `common/state_estimation/kalman_filter/` |

### 2.2 运动模型示例

```cpp
#include "autonomy/common/motion_model/linear_motion_model.hpp"
using namespace autonomy::common::motion_model;

using State2D = state_vector::GenericState<4>; // x, y, vx, vy
LinearMotionModel<State2D> model;
State2D state;
state.Set(0, 1.0); state.Set(1, 2.0);
state.Set(2, 0.5); state.Set(3, 0.0);
State2D predicted = model.predict(state, std::chrono::milliseconds(100));
```

### 2.3 未来接入（规划）

1. 编辑 `config/prediction/prediction.lua`
2. `autonomy.lua` 设 `prediction = AUTONOMY_PREDICTION`
3. `options.cpp` 加载；`Autonomy::Start()` 构造 `PredictionServer`

```lua
AUTONOMY_PREDICTION = {
    enabled = true,
    frequency = 10.0,
    prediction_horizon = 5.0,
    prediction_dt = 0.1,
    motion_model = "cv", -- cv | ctrv | diffdrive | stationary
}
```

### 2.4 规划数据流

```
Detection2DArray → 关联 → KF 更新 → MotionModel 多步 → PredictedTrajectory[] → Planning
```

依赖 [10 Perception](../10_Perception/00_guide.md) 检测输出。

### 2.5 测试

```bash
./autonomy.common.motion_model.linear_motion_model_test
./autonomy.common.motion_model.differential_motion_model_test
./autonomy.common.motion_model.stationary_motion_model_test
```

### 2.6 参见
