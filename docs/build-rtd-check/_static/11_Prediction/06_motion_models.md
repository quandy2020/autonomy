# 6. 运动模型

本文描述 Autonomy `common/motion_model` 库中的运动学模型，供 Prediction 模块做轨迹前向传播。

---

## 6.1 接口设计

`MotionModelInterface` 采用 CRTP 模式：

```cpp
template <typename Derived, typename StateT>
class MotionModelInterface {
public:
    StateT predict(const StateT& state, const std::chrono::nanoseconds& dt) {
        return static_cast<Derived*>(this)->crtp_predict(state, dt);
    }
    typename StateT::Matrix jacobian(const StateT& state, const std::chrono::nanoseconds& dt);
};
```

| 方法 | 说明 |
|------|------|
| `predict(state, dt)` | 状态前向传播 $\mathbf{x}_{k+1} = f(\mathbf{x}_k, \Delta t)$ |
| `jacobian(state, dt)` | 雅可比 $F = \partial f / \partial \mathbf{x}$，供卡尔曼使用 |

---

## 6.2 LinearMotionModel（恒速/恒加速）

**文件**：`linear_motion_model.hpp` / `.cpp`

**状态**：GenericState，典型为 $[x, y, v_x, v_y]$ 或含加速度扩展。

**转移矩阵**（CV）：

$$
F = \begin{bmatrix}
1 & 0 & \Delta t & 0 \\
0 & 1 & 0 & \Delta t \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

**预测**：

```cpp
State predicted = model.predict(state, std::chrono::milliseconds(100));
```

**适用**：行人、缓慢移动障碍、短期预测。

---

## 6.3 DifferentialDriveMotionModel

**文件**：`differential_drive_motion_model.hpp` / `.cpp`

**状态**：$[x, y, \theta, v, \omega]$ 或简化形式。

**动力学**：

$$
\begin{aligned}
\dot{x} &= v \cos\theta \\
\dot{y} &= v \sin\theta \\
\dot{\theta} &= \omega
\end{aligned}
$$

**离散化**（欧拉）：

$$
\begin{aligned}
x_{k+1} &= x_k + v \cos\theta_k \Delta t \\
y_{k+1} &= y_k + v \sin\theta_k \Delta t \\
\theta_{k+1} &= \theta_k + \omega \Delta t
\end{aligned}
$$

**适用**：差速移动机器人作为动态障碍时的预测。

---

## 6.4 StationaryMotionModel

**文件**：`stationary_motion_model.hpp`

**预测**：$\mathbf{x}_{k+1} = \mathbf{x}_k$（状态不变）

**适用**：静态障碍、停车车辆、未知动态障碍的保守假设。

---

## 6.5 与卡尔曼滤波的配合

```cpp
// 预测步
auto x_pred = motion_model.predict(x_est, dt);
auto F = motion_model.jacobian(x_est, dt);
P_pred = F * P_est * F.transpose() + Q;

// 更新步（观测到达后）
K = P_pred * H.transpose() * (H * P_pred * H.transpose() + R).inverse();
x_est = x_pred + K * (z - H * x_pred);
P_est = (I - K * H) * P_pred;
```

---

## 6.6 模型选型指南

| 障碍类型 | 推荐模型 | 预测时域 |
|----------|----------|----------|
| 行人 | CV | 2–5 s |
| 车辆（道路） | CTRV / CA | 3–8 s |
| 差速机器人 | DiffDrive | 2–5 s |
| 未知/静态 | Stationary | — |
| 高速机动 | CA + 大过程噪声 | 1–3 s |

---

## 6.7 单元测试

| 测试文件 | 验证内容 |
|----------|----------|
| `linear_motion_model_test.cpp` | CV 前向传播正确性 |
| `differential_motion_model_test.cpp` | 非完整约束轨迹 |
| `stationary_motion_model_test.cpp` | 状态不变性 |

---

## 6.8 扩展：CTRV 实现建议

当前仓库无独立 CTRV 类，可按以下方式扩展：

```cpp
class CTRVMotionModel : public MotionModelInterface<CTRVMotionModel, State5D> {
    State5D crtp_predict(const State5D& s, const std::chrono::nanoseconds& dt) const {
        double v = s.v(), theta = s.theta(), omega = s.omega();
        double dt_s = std::chrono::duration<double>(dt).count();
        if (std::abs(omega) < 1e-6) {
            // CV 退化
        } else {
            // 解析 CTRV 公式，见 03_math.md §3.3
        }
    }
};
```
