# 7. 轨迹预测算法

本文描述 Prediction 模块规划中的轨迹预测算法：从卡尔曼跟踪到多步前向传播。

---

## 7.1 预测流水线

```
检测输入 z_k
    │
    ▼
数据关联（匈牙利 + 门控）
    │
    ▼
卡尔曼更新 → 状态估计 x̂_k|k
    │
    ▼
运动模型多步 predict → {x̂_{k+1|k}, ..., x̂_{k+K|k}}
    │
    ▼
PredictedTrajectory 输出
```

---

## 7.2 跟踪与关联

### 7.2.1 轨迹管理

每个动态障碍维护一条轨迹：

| 字段 | 说明 |
|------|------|
| `id` | 唯一标识 |
| `state` | 卡尔曼状态 $\hat{\mathbf{x}}$ |
| `covariance` | $P$ |
| `age` | 存活帧数 |
| `coast_count` | 连续无观测帧数 |

### 7.2.2 关联算法

1. 构造代价矩阵 $C_{ij}$（IoU 或马氏距离）
2. 匈牙利算法求最优匹配
3. 未匹配检测 → 新轨迹；未匹配轨迹 → `coast_count++`，超阈值删除

### 7.2.3 门控

马氏距离：

$$
d_M^2 = (\mathbf{z} - H\hat{\mathbf{x}})^\top S^{-1} (\mathbf{z} - H\hat{\mathbf{x}}) < \chi^2_{0.95, 2} \approx 5.99
$$

---

## 7.3 恒速预测器（CV Predictor）

**算法**：

1. 用 CV 运动模型 + 卡尔曼滤波估计 $[x, y, v_x, v_y]$
2. 对 $k = 1, \ldots, K$，$\hat{\mathbf{x}}_{k+j|k} = F^j \hat{\mathbf{x}}_{k|k}$

**输出**：直线轨迹，不确定性椭圆随 $j$ 增大。

**公式**：见 [03_math.md §3.2](03_math.md#32-恒速模型constant-velocity-cv)。

---

## 7.4 CTRV 预测器

**算法**：

1. 状态 $[x, y, v, \theta, \omega]$
2. 解析积分（$\omega \neq 0$）或 CV 退化（$\omega \approx 0$）

**输出**：圆弧或直线轨迹，适合车辆。

**公式**：见 [03_math.md §3.3](03_math.md#33-恒转弯率模型ctrv)。

---

## 7.5 多模态预测

对意图不确定的障碍，输出 $L$ 条轨迹：

| 模式 | 概率 | 运动 |
|------|------|------|
| 直行 | $\pi_1$ | CV 沿当前方向 |
| 左转 | $\pi_2$ | CTRV $\omega > 0$ |
| 右转 | $\pi_3$ | CTRV $\omega < 0$ |
| 停车 | $\pi_4$ | Stationary |

约束：$\sum_l \pi_l = 1$。

规划器可取 $\arg\max_l \pi_l$ 或鲁棒优化（考虑所有模式）。

---

## 7.6 预测时域与步长

| 参数 | 典型值 | 说明 |
|------|--------|------|
| $T_p$ | 3–8 s | 预测时域 |
| $\Delta t$ | 0.1 s | 离散步长 |
| $K = T_p / \Delta t$ | 30–80 | 轨迹点数 |

**实时性**：$K$ 不宜过大；10 Hz 预测频率下，单障碍 CV 预测 $< 1$ ms。

---

## 7.7 不确定性传播

协方差传播：

$$
P_{k+j|k} = F^j P_{k|k} (F^j)^\top + \sum_{i=0}^{j-1} F^i Q (F^i)^\top
$$

2D 位置不确定性椭圆供规划器做 chance-constraint 避障。

---

## 7.8 与 Planning 的对接

```cpp
struct PredictedTrajectory {
    int id;
    double probability;
    std::vector<Pose2D> poses;      // t, x, y, theta
    std::vector<Eigen::Matrix2d> cov_xy;  // 可选
};

// Planning 查询 t 时刻障碍占用
bool occupied = false;
for (const auto& traj : trajectories) {
    auto pose = Interpolate(traj, t);
    if (Distance(ego_pose, pose) < r_safe) occupied = true;
}
```

---

## 7.9 实现优先级

| 优先级 | 算法 | 依赖 |
|--------|------|------|
| P0 | CV + 卡尔曼 | motion_model ✅ |
| P1 | CTRV | 需新增 CTRV 类 |
| P2 | 多模态（规则） | 地图/车道 |
| P3 | 学习型（VectorNet） | 训练数据、ONNX |
