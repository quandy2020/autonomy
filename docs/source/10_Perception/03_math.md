(perception-math)=
# 3. 数学原理

实现对照：[§7](07_vision_detection.md)、[§8](08_obstacle_perception.md)；综述：[§9](09_survey.md)。

### 3.1 坐标系与传感器

| 模型 | 公式 |
|------|------|
| 刚体变换 | $p_W = R_W^S p_S + t_W^S$，$T_W^S$ 由 `transform::Buffer` 查询 |
| 针孔投影 | $u = f_x X/Z + c_x$，$v = f_y Y/Z + c_y$；$K=\mathrm{diag}(f_x,f_y,1)$ 含主点 |
| 深度反投影 | $[X,Y,Z]^\top = d\,K^{-1}[u,v,1]^\top$ |
| 2D 激光 | $p_i^S = (r_i\cos\phi_i,\, r_i\sin\phi_i,\, 0)^\top$ |

### 3.2 点云处理

| 步骤 | 公式 / 规则 |
|------|-------------|
| 体素下采样 | $\mathrm{idx}(p)=\lfloor(p-p_{min})/v\rfloor$；体素质心 $\bar p_v=\frac1{\|V\|}\sum_{p\in V}p$ |
| RANSAC 地面 | 平面 $\mathbf n^\top p+d=0$；内点 $\|\mathbf n^\top p+d\|<\tau$ |
| 欧氏聚类 | KD-Tree + BFS：$\|p_i-p_j\|_2<r_{cluster}$；滤除 $\|C\|<N_{min}$ |

### 3.3 2D 检测（YOLO）

**解码**（网格 $(i,j)$，锚框 $(p_w,p_h)$）：

$$
b_x=\sigma(t_x)+j,\quad b_y=\sigma(t_y)+i,\quad
b_w=p_w e^{t_w},\quad b_h=p_h e^{t_h},\quad
\mathrm{score}_k=p_{obj}\,p_{c_k}
$$

保留 $\mathrm{score}_k>\tau_{conf}$。

**NMS / IoU**：按 score 降序；抑制 $\mathrm{IoU}>\tau_{nms}$。轴对齐框：

$$
A_{int}=\max(0,x_2^{int}-x_1^{int})\cdot\max(0,y_2^{int}-y_1^{int}),\quad
\mathrm{IoU}=A_{int}/(A_A+A_B-A_{int})
$$

实现：`common/network/detail/postprocess/nms.cpp`。

### 3.4 3D / BEV

| 项 | 公式 |
|----|------|
| BEV 栅格 | $i=\lfloor(x-x_{min})/\Delta x\rfloor$，$j=\lfloor(y-y_{min})/\Delta y\rfloor$ |
| 3D 框 | $(x,y,z,l,w,h,\theta)$；角点 $c_k=(x,y,z)^\top+R_z(\theta)(\pm l/2,\pm w/2,\pm h/2)^\top$ |

### 3.5 多目标跟踪（CV + KF）

状态 $\mathbf x=[x,y,v_x,v_y]^\top$；$F$ 为恒速转移；$H=[I_2\,0]$。

$$
\hat{\mathbf x}_{k|k-1}=F\hat{\mathbf x}_{k-1},\quad
P_{k|k-1}=FPF^\top+Q,\quad
K=PH^\top(HPH^\top+R)^{-1}
$$

关联：代价 $C_{ij}=1-\mathrm{IoU}$，匈牙利最小匹配。

### 3.6 代价地图

射线（Bresenham）：原点→障碍点标自由，终点标 lethal。膨胀：

$$
\mathrm{cost}(d)=\mathrm{INSCRIBED}\,e^{-\lambda(d-r_{robot})}\quad(d\le r_{infl})
$$

### 3.7 符号表

| 符号 | 含义 |
|------|------|
| $K$ | 相机内参 |
| $T_W^S$ | 传感器→世界 |
| $\tau_{conf},\tau_{nms}$ | 置信度 / NMS IoU 阈值 |
| $v,\tau,r_{cluster}$ | 体素边长、RANSAC 阈值、聚类半径 |
| $Q,R$ | 过程 / 观测噪声 |
