# 6. 运动模型

库：`autonomy/common/motion_model`。供 Prediction 前向传播。

### 6.1 接口

```cpp
template <typename Derived, typename StateT>
class MotionModelInterface {
    StateT predict(const StateT& state, const std::chrono::nanoseconds& dt);
    typename StateT::Matrix jacobian(const StateT& state, const std::chrono::nanoseconds& dt);
};
```

| 方法 | 含义 |
|------|------|
| `predict` | $\mathbf x_{k+1}=f(\mathbf x_k,\Delta t)$ |
| `jacobian` | $F=\partial f/\partial\mathbf x$（KF） |

### 6.2 已实现模型

| 模型 | 文件 | 状态 / 动力学 | 适用 |
|------|------|---------------|------|
| Linear | `linear_motion_model.*` | CV：$F$ 含 $\Delta t$ | 行人、短期 |
| DiffDrive | `differential_drive_motion_model.*` | $\dot x=v\cos\theta$，$\dot\theta=\omega$ | 差速障碍 |
| Stationary | `stationary_motion_model.hpp` | $\mathbf x_{k+1}=\mathbf x_k$ | 静态 / 保守 |

```cpp
State predicted = model.predict(state, std::chrono::milliseconds(100));
```

公式见 [§3.2](03_math.md)。

### 6.3 与卡尔曼配合

```cpp
auto x_pred = motion_model.predict(x_est, dt);
auto F = motion_model.jacobian(x_est, dt);
P_pred = F * P_est * F.transpose() + Q;
// 更新：K, x_est, P_est
```

### 6.4 选型

| 障碍 | 模型 | 时域 |
|------|------|------|
| 行人 | CV | 2–5 s |
| 道路车辆 | CTRV / CA（待） | 3–8 s |
| 差速机器人 | DiffDrive | 2–5 s |
| 未知/静态 | Stationary | — |

### 6.5 测试与 CTRV 扩展

| 测试 | 内容 |
|------|------|
| `linear_motion_model_test.cpp` | CV 传播 |
| `differential_motion_model_test.cpp` | 非完整约束 |
| `stationary_motion_model_test.cpp` | 不变性 |

CTRV：按 [§3.3](03_math.md) 解析积分，当前仓库无独立类。

### 6.6 参见
