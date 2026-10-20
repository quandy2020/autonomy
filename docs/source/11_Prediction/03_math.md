(prediction-math)=
# 3. 数学原理

实现：[§6](06_motion_models.md)；综述：[§7](07_survey.md)。

### 3.1 问题形式化

障碍状态 $\mathbf x_i(t)\in\mathbb{R}^n$；给定历史 $\mathcal H_t$ 与时域 $T_p$：

$$
\hat{\mathbf x}_i(t+\tau)=f(\mathbf x_i(t),\tau,\mathcal H_t),\quad
\hat{\mathcal T}_i=\{\hat{\mathbf x}_i(t+k\Delta t)\}_{k=0}^{K},\quad K=T_p/\Delta t
$$

### 3.2 运动模型

| 模型 | 状态 | 转移 |
|------|------|------|
| CV | $[x,y,v_x,v_y]$ | $x\leftarrow x+v_x\Delta t$（$F$ 恒速矩阵） |
| CTRV | $[x,y,v,\theta,\omega]$ | $\omega\neq0$：$x\leftarrow x+\frac v\omega(\sin(\theta+\omega\Delta t)-\sin\theta)$ 等；$\omega\to0$ 退化为 CV |
| CA | $[x,y,v_x,v_y,a_x,a_y]$ | $x\leftarrow x+v_x\Delta t+\frac12 a_x\Delta t^2$，$v_x\leftarrow v_x+a_x\Delta t$ |

### 3.3 卡尔曼预测 / 更新

$$
\hat{\mathbf x}_{k|k-1}=F\hat{\mathbf x},\quad P\leftarrow FPF^\top+Q
$$
$$
K=PH^\top(HPH^\top+R)^{-1},\quad
\hat{\mathbf x}\leftarrow\hat{\mathbf x}+K(\mathbf z-H\hat{\mathbf x}),\quad
P\leftarrow(I-KH)P
$$

多步：$F^j\hat{\mathbf x}$；$P$ 累加过程噪声传播。

### 3.4 数据关联

代价 $C_{ij}=1-\mathrm{IoU}$ 或 $\|\hat{\mathbf x}_i-\mathbf z_j\|_2$；匈牙利最小匹配。门控：

$$
d_M^2=(\mathbf z-H\hat{\mathbf x})^\top S^{-1}(\mathbf z-H\hat{\mathbf x})<\chi^2_{\alpha},\quad S=HPH^\top+R
$$

### 3.5 不确定性与风险

| 项 | 公式 |
|----|------|
| 置信椭圆 | $(\mathbf p-\boldsymbol\mu)^\top P_{xy}^{-1}(\mathbf p-\boldsymbol\mu)=k^2$ |
| 多模态 | $\{\hat{\mathcal T}^{(l)},\pi_l\}$，$\sum\pi_l=1$ |
| TTC | $\min_{\tau>0}\{\tau:\|\mathbf p_{ego}(\tau)-\mathbf p_i(\tau)\|<r_{safe}\}$ |

### 3.6 符号表

| 符号 | 含义 |
|------|------|
| $T_p,\Delta t,F,H$ | 时域、步长、转移、观测矩阵 |
| $Q,R,\omega,\pi_l$ | 过程/观测噪声、角速度、模态概率 |
