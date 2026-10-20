(mppi-controller)=
# 2. MPPI Controller

路径积分 / 采样 MPC：每周期采样多条控制序列，Critic 加权 softmax 得输出。源码：`mppi_controller/`（含 **GridMap critic**，可接 Perception Follow）。

每周期采样大量控制序列，由 Critic 加权（softmax）得到输出，可在单环里叠路径、障碍、目标角等代价。动态环境或跟人（GridMap critic）优先考虑；CPU/GPU 明显高于 RPP/Graceful。

## 何时用

动态障碍、需要多目标代价（路径跟踪 + 障碍 + 目标角等）；跟人场景开 Grid 相关 critic。算力明显高于 RPP/Graceful。

## 模型（摘要）

滚动 horizon $H$，采样 $K$ 条序列，代价 $S$，权重 $w_k\propto\exp(-S_k/\lambda)$，$\mathbf{u}^*=\sum_k w_k\mathbf{u}^{(k)}$。

差速前向积分：$x\leftarrow x+v\cos\theta\,\Delta t$ 等。$S$ 由 Critic 插件累加。

## 典型量级

Nav2 参考：$K\sim 2000$，$H\sim 56$，$\Delta t\sim 0.05\,\mathrm{s}$（以 conf 为准）。复杂度 $O(K\cdot H\cdot|\mathrm{critics}|)$。

## 配置要点

`motion_model`：DiffDrive / Holonomic / Ackermann。Critic 权重与 `threshold_to_consider` 需和 $H\Delta t$ 一致，否则要么过保守要么撞障。

## 常见坑

| 现象 | 处理 |
|------|------|
| 跟人不避人 | Follow 是否写 GridMap；Grid critic 是否启用 |
| 抖动 | 减小 $\lambda$ 或调整 control noise |
| 算力爆 | 降 $K$ 或 $H$；Policy 路径另议 |

归属 [04](../04_controller_algorithms.md)。Follow：[Perception · Follow](../../10_Perception/11_follow.md)。
