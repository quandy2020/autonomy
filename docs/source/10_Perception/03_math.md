(perception-math)=
# 3. 数学原理

调 YOLO 阈值、costmap 膨胀或跟踪关联时查公式；实现见 [§7](07_vision_detection.md)、[§8](08_obstacle_perception.md)。

公式与实现一一对应；调参时区分 network 后处理与 costmap/层参数。

### 坐标系与传感器

| 模型 | 公式 |
|------|------|
| 刚体变换 | $p_W = R_W^S p_S + t_W^S$，$T_W^S$ 由 `transform::Buffer` 查询 |
| 针孔投影 | $u = f_x X/Z + c_x$，$v = f_y Y/Z + c_y$ |
| 深度反投影 | $[X,Y,Z]^\top = d\,K^{-1}[u,v,1]^\top$ |
| 2D 激光 | $p_i^S = (r_i\cos\phi_i,\, r_i\sin\phi_i,\, 0)^\top$ |

### 点云处理

| 步骤 | 公式 / 规则 |
|------|-------------|
| 体素下采样 | $\mathrm{idx}(p)=\lfloor(p-p_{min})/v\rfloor$ |
| RANSAC 地面 | 平面 $\mathbf n^\top p+d=0$；内点 $\|\mathbf n^\top p+d\|<\tau$ |
| 欧氏聚类 | $\|p_i-p_j\|_2<r_{cluster}$ |

### 2D 检测（YOLO）

网格 $(i,j)$，锚框 $(p_w,p_h)$：

$$
b_x=\sigma(t_x)+j,\quad b_y=\sigma(t_y)+i,\quad
b_w=p_w e^{t_w},\quad b_h=p_h e^{t_h},\quad
\mathrm{score}_k=p_{obj}\,p_{c_k}
$$

NMS 轴对齐 IoU：实现 `common/network/detail/postprocess/nms.cpp`。

### 多目标跟踪（CV + KF）

状态 $\mathbf x=[x,y,v_x,v_y]^\top$；关联代价 $C_{ij}=1-\mathrm{IoU}$。Base `TASK_TRACK` 与 follow 消费 tracks，**不是** prediction 模块。

### 代价地图

射线 Bresenham marking/clearing；膨胀：

$$
\mathrm{cost}(d)=\mathrm{INSCRIBED}\,e^{-\lambda(d-r_{robot})}\quad(d\le r_{infl})
$$

### 符号

| 符号 | 含义 |
|------|------|
| $K$ | 相机内参 |
| $\tau_{conf},\tau_{nms}$ | 置信度 / NMS |
| $Q,R$ | KF 噪声（若自研跟踪） |

### 延伸阅读

[§9 综述](09_survey.md)
