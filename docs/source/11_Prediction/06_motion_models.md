# 6. 运动模型

在滤波、仿真或自研预测节点里做状态一步传播，需要 `predict` 和雅可比。

库路径：`autonomy/common/motion_model/`。与 `autonomy/prediction/` 里的 Server **无关**，可直接 `#include` 使用。

| 障碍类型 | 模型 | 时域经验 |
|----------|------|----------|
| 行人 | Linear + 4 状态 CV | 2–5 s |
| 差速机器人 | DiffDrive | 2–5 s |
| 不确定是否移动 | Stationary | 保守 |
| 道路车辆 CTRV | 自写或非线性模型 | 3–8 s |

### 接口

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
| `jacobian` | $F=\partial f/\partial\mathbf x$（线性模型下即转移矩阵） |

### 已实现模型

| 模型 | 文件 | 动力学 | 典型用途 |
|------|------|--------|----------|
| Linear | `linear_motion_model.*` | 线性转移（常用 4 维 CV） | 行人短 horizon |
| DiffDrive | `differential_drive_motion_model.*` | $\dot x=v\cos\theta$ 等 | 差速障碍 |
| Stationary | `stationary_motion_model.hpp` | $\mathbf x_{k+1}=\mathbf x_k$ | 静态假设 |

```cpp
State predicted = model.predict(state, std::chrono::milliseconds(100));
```

公式对照 [§3.2](03_math.md)。**没有**名为 `CtrvMotionModel` 的类；CTRV 公式仅作数学参考，需自实现。

### 与卡尔曼配合

```cpp
auto x_pred = motion_model.predict(x_est, dt);
auto F = motion_model.jacobian(x_est, dt);
P_pred = F * P_est * F.transpose() + Q;
// 量测更新见 common/state_estimation/kalman_filter/
```

### 常见坑

| 误区 | 说明 |
|------|------|
| 「CA 在 prediction 包里」 | 只有 Linear；CA 需扩状态维 |
| 「Server 会选 motion_model 字符串」 | Server 未实现 |
| 多线程共用一个 model | 无状态则一般安全；带内部状态时需隔离 |

### 延伸阅读

[§3](03_math.md) · [§4](04_usage.md)
