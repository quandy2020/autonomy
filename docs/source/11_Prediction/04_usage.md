(prediction-usage)=
# 4. 使用指南

实验里要用运动模型或卡尔曼积木，或想确认 `PredictionServer` 能不能订阅检测——后者目前不行。

| 块 | 能否直接用 |
|----|------------|
| `common/motion_model/*` | 能，`predict` + `jacobian` |
| `common/state_estimation/kalman_filter/` | 能，自行组合 |
| `PredictionServer` | 不能：仅有类声明，`.cpp` 无实现，主进程未构造 |
| `config/prediction/prediction.lua` | 占位，未挂 `AUTONOMY` |

- 导航栈**不依赖**预测话题；动态障碍靠 costmap 刷新。  
- Lua 里 `motion_model = "ctrv"` 等选项**没有**对应 Predictor 实现。  
- `PredictionInterface` 是空虚基类，没有 `OnDetections` 等虚函数落地。

| 目标 | 建议 |
|------|------|
| 仿真/工具里滚几个障碍点 | 直接用 [§6](06_motion_models.md) |
| 车上要预测话题 | 自建节点或等 Server 接线 |
| Planning 要时空障碍 | 当前用静态 costmap + 重规划 |

### 运动模型（可用）

```cpp
#include "autonomy/common/motion_model/linear_motion_model.hpp"
using namespace autonomy::common::motion_model;

using State2D = state_vector::GenericState<4>; // x, y, vx, vy
LinearMotionModel<State2D> model;
State2D predicted = model.predict(state, std::chrono::milliseconds(100));
auto F = model.jacobian(state, std::chrono::milliseconds(100));
```

| 模型 | 头文件 | 适用 |
|------|--------|------|
| `LinearMotionModel` | `linear_motion_model.hpp` | 恒速（状态维由 `StateT` 决定） |
| `DifferentialDriveMotionModel` | `differential_drive_motion_model.hpp` | 差速 |
| `StationaryMotionModel` | `stationary_motion_model.hpp` | 静态/保守 |

仓库**没有**独立 CTRV/CA 类；CA 需自己扩状态维并用 `LinearMotionModel`，或自写 `MotionModelInterface`。

### proto 预留字段（未驱动 Server）

| 字段 | 含义 |
|------|------|
| `enabled` | 预留启停 |
| `prediction_horizon` / `prediction_dt` | 预留时域 |
| `motion_model` | 字符串占位，无读取逻辑 |

### 规划中的输出形态（未实现）

业务若需要统一轨迹结构，可参考 Nav2 风格自行定义；仓库内**没有**已发布的 `PredictedTrajectory` Writer。

### 常见坑

| 现象 | 说明 |
|------|------|
| 找不到预测话题 | 正常 |
| 开了 `prediction.enabled` 无进程 | 主栈未接 PredictionServer |
| 文档里的匈牙利/CTRV 类 | 数学在 [§3](03_math.md)，代码未全 |

### 延伸阅读

[§2](02_quickstart.md) · [§6 运动模型](06_motion_models.md) · [§7 综述](07_survey.md)
