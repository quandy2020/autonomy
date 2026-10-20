(mppi-controller)=
# 2. MPPI Controller

归属 [§4](../04_controller_algorithms.md)。路径积分采样 MPC；源码 `mppi_controller/` （含 GridMap critic，可供 Follow）。

### 定义

固定 $\Delta t$ 滚动 horizon $H$，采样 $K$ 条控制序列，代价 $S$，softmax 加权：

$$
\mathbf{u}^*=\sum_{k=1}^K w_k\mathbf{u}^{(k)},\quad
w_k\propto\exp(-S_k/\lambda)
$$

差速前向：$x\leftarrow x+v\cos\theta\,\Delta t$ 等。$S$ 由 Critic 插件累加（障碍、路径对齐、目标角等）。

### 结构

| 默认量级（Nav2） | — |
|----------------|--|
| $K$ / $H$ / $\Delta t$ | 2000 / 56 / 0.05 s |

复杂度 $O(K\cdot H\cdot|\mathcal{M}|)$。

### 用法

`motion_model`：DiffDrive / Holonomic / Ackermann。权重与 `threshold_to_consider` 需与 $H\Delta t$ 协调。
