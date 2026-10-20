(prediction-usage)=
# 4. 使用指南

### 4.1 配置（规划）

入口：`config/prediction/prediction.lua`。当前空表，未挂入 `AUTONOMY`。

| 字段 | 说明 | 建议 |
|------|------|------|
| `enabled` | 启动 PredictionServer | `false` |
| `frequency` | Hz | `10.0` |
| `prediction_horizon` | $T_p$ (s) | `5.0` |
| `prediction_dt` | $\Delta t$ (s) | `0.1` |
| `motion_model` | 默认模型 | `"cv"` |
| `plugins` | 插件列表 | 待实现 |

### 4.2 API

| API | 用途 | 状态 |
|-----|------|------|
| `Start` / `Shutdown` | 启停 | — |
| `OnDetections` / `OnObstacles` | 输入 | — |
| `GetPredictedTrajectories` / `GetCollisionRisks` | 输出 | — |

### 4.3 运动模型（可用）

```cpp
// MotionModelInterface::predict(state, dt) / jacobian(state, dt)
```

| 模型 | 头文件 | 适用 |
|------|--------|------|
| `LinearMotionModel` | `linear_motion_model.hpp` | 行人等 |
| `DifferentialDriveMotionModel` | `differential_drive_motion_model.hpp` | 差速障碍 |
| `StationaryMotionModel` | `stationary_motion_model.hpp` | 静态 |

### 4.4 输出结构（规划）

```cpp
struct PredictedTrajectory {
    int32_t obstacle_id;
    double probability;
    std::vector<PoseStamped> trajectory;
};
```

### 4.5 与 Planning（规划）

| 接口 | 说明 |
|------|------|
| `SetDynamicObstacles(trajectories)` | 注入时空规划 |
| `GetOccupancyAt(t, x, y)` | 查询动态占据 |

### 4.6 故障排查

| 现象 | 处理 |
|------|------|
| Server 未启动 | 预期；静态规划仍可用 |
| 轨迹发散 | 行人 CV、车辆 CTRV |
| 关联错误 | 匈牙利 + 门控 |
| 滞后 | `frequency` → 10–20 Hz |

### 4.7 参见
