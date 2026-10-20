# 2. 快速开始

若需在单元测试或离线工具里调用运动模型，而不是启动「预测服务」。

仓库里**能跑起来的**是 `common/motion_model` 单测与示例调用；没有 `prediction_main` 一类可交互服务。

- **不要**按下面 Lua 片段期望车上有预测进程；`prediction.lua` 未接入主配置加载。  
- Perception→Prediction→Planning 整链**未实现**；检测话题来自 [10 Base](../10_Perception/10_base.md)，与 prediction 无自动连接。

| 需求 | 做法 |
|------|------|
| 验证 CV 传播 | 跑 motion_model 测试 |
| 车上动态障碍 | costmap + 感知刷新 |
| 完整预测栈 | 自研或等 Server 落地 |

### 运动模型示例

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

差速、静止模型见 [§6](06_motion_models.md)。

### 测试目标

```bash
./autonomy.common.motion_model.linear_motion_model_test
./autonomy.common.motion_model.differential_motion_model_test
./autonomy.common.motion_model.stationary_motion_model_test
```

（二进制名以构建目录实际产物为准。）

### 预留配置（仅供将来接线参考）

```lua
-- config/prediction/prediction.lua（当前未挂 AUTONOMY）
AUTONOMY_PREDICTION = {
    enabled = true,
    frequency = 10.0,
    prediction_horizon = 5.0,
    prediction_dt = 0.1,
    motion_model = "cv",
}
```

`ctrv` 等字符串在代码中**无**对应实现类。

### 常见坑

| 现象 | 说明 |
|------|------|
| 链接 PredictionServer | 构造/析构可能未定义，勿依赖 |
| 等 `/prediction/*` 话题 | 不存在 |

### 延伸阅读

[§6](06_motion_models.md) · [§4](04_usage.md)
