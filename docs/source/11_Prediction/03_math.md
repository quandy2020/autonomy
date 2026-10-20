(prediction-math)=
# 3. 数学原理

自研跟踪/预测或读 motion_model 代码时查公式；**不是**已实现算法清单。

形式化多障碍预测问题，并与 [§6](06_motion_models.md) 里**已有** CV/差速模型对照。CTRV、匈牙利等下文有式子，但仓库**无**完整 Prediction 栈实现。

公式与实现一一对应；调参时区分 network 后处理与 costmap/层参数。

### 问题形式化

障碍状态 $\mathbf x_i(t)$；给定历史与时域 $T_p$：

$$
\hat{\mathbf x}_i(t+\tau)=f(\mathbf x_i(t),\tau,\mathcal H_t),\quad
\hat{\mathcal T}_i=\{\hat{\mathbf x}_i(t+k\Delta t)\}_{k=0}^{K}
$$

### 运动模型（代码对照）

| 模型 | 状态 | 仓库 |
|------|------|------|
| CV | $[x,y,v_x,v_y]$ | `LinearMotionModel` + 4 维 State |
| 差速 | $[x,y,\theta,v,\omega]$ 等 | `DifferentialDriveMotionModel` |
| 静止 | 任意 | `StationaryMotionModel` |
| CTRV / CA | 见下式 | **无**独立类；CA 需自扩状态 |

CTRV（$\omega\neq0$）：

$$
x\leftarrow x+\frac v\omega(\sin(\theta+\omega\Delta t)-\sin\theta),\ \ldots
$$

### 卡尔曼

预测：$\hat{\mathbf x}_{k|k-1}=F\hat{\mathbf x}$，$P\leftarrow FPF^\top+Q$。  
更新：$K=PH^\top(HPH^\top+R)^{-1}$，$\hat{\mathbf x}\leftarrow\hat{\mathbf x}+K(\mathbf z-H\hat{\mathbf x})$。

实现积木：`common/state_estimation/kalman_filter/`。

### 关联与风险（规划参考）

代价 $C_{ij}=1-\mathrm{IoU}$ 或马氏距离；门控 $\chi^2$。TTC、多模态轨迹为规划扩展，**无**默认 C++ 模块。

### 符号

| 符号 | 含义 |
|------|------|
| $T_p,\Delta t$ | 预测时域、步长 |
| $F,H,Q,R$ | 转移、观测、噪声 |

### 延伸阅读

[§6](06_motion_models.md) · [§7](07_survey.md)
