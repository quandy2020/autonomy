(perception-math)=
# 3. 数学原理

> 本文以 **Step-by-Step** 方式推导 Perception 模块涉及的核心公式。实现对照见 [07_vision_detection.md](07_vision_detection.md)、[08_obstacle_perception.md](08_obstacle_perception.md)；算法综述见 [09_survey.md](09_survey.md)。

---

### 3.1 坐标系与传感器模型

#### Step 1：齐次变换

传感器坐标系 $S$ 到世界坐标系 $W$ 的刚体变换：

$$
T_W^S = \begin{bmatrix} R_W^S & t_W^S \\ 0 & 1 \end{bmatrix}, \quad
p_W = R_W^S \, p_S + t_W^S
$$

其中 $R_W^S \in SO(3)$，$t_W^S \in \mathbb{R}^3$。Autonomy 通过 `transform::Buffer` 查询 $T_W^S$。

#### Step 2：针孔相机投影

相机内参矩阵 $K$：

$$
K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}
$$

三维点 $P_C = (X, Y, Z)^\top$（相机坐标系）投影到像素 $(u, v)$：

$$
\lambda \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
= K \begin{bmatrix} X \\ Y \\ Z \end{bmatrix}
\quad \Rightarrow \quad
u = f_x \frac{X}{Z} + c_x, \; v = f_y \frac{Y}{Z} + c_y
$$

#### Step 3：深度反投影

给定像素 $(u, v)$ 与深度 $d$（米），恢复相机坐标系三维点：

$$
\begin{bmatrix} X \\ Y \\ Z \end{bmatrix}
= d \cdot K^{-1} \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
$$

#### Step 4：激光雷达极坐标模型

2D 激光第 $i$ 束测量 $(r_i, \phi_i)$，传感器坐标系下点：

$$
p_i^S = \begin{bmatrix} r_i \cos\phi_i \\ r_i \sin\phi_i \\ 0 \end{bmatrix}
$$

变换到世界坐标：$p_i^W = R_W^S p_i^S + t_W^S$。

---

### 3.2 点云处理

#### 3.2.1 体素下采样（Voxel Grid Filter）

**Step 1**：将空间划分为边长 $v$ 的体素网格，体素索引：

$$
\mathrm{idx}(p) = \left\lfloor \frac{p - p_{min}}{v} \right\rfloor
$$

**Step 2**：每个非空体素保留**质心**（或第一个点）：

$$
\bar{p}_v = \frac{1}{|V|} \sum_{p \in V} p
$$

复杂度从 $O(N)$ 降至 $O(N/v^3)$ 量级（稀疏体素）。

#### 3.2.2 地面分割（RANSAC 平面拟合）

**Step 1**：随机采样 3 点确定平面 $\pi: \mathbf{n}^\top p + d = 0$，法向量 $\mathbf{n} = (p_2 - p_1) \times (p_3 - p_1)$，归一化后 $d = -\mathbf{n}^\top p_1$。

**Step 2**：内点判定（距离阈值 $\tau$）：

$$
\mathrm{inlier}(p) \Leftrightarrow |\mathbf{n}^\top p + d| < \tau
$$

**Step 3**：迭代 $K$ 次，取内点数最多的平面为地面；点 $p$ 满足 $|\mathbf{n}^\top p + d| < \tau_{ground}$ 标记为地面点。

#### 3.2.3 欧氏聚类（Euclidean Clustering）

**Step 1**：对非地面点构建 KD-Tree。

**Step 2**：BFS/DFS 扩展：若 $\|p_i - p_j\|_2 < r_{cluster}$，则 $p_i, p_j$ 同属一簇。

**Step 3**：过滤点数 $< N_{min}$ 的噪声簇，输出障碍聚类 $\{C_1, \ldots, C_M\}$。

---

### 3.3 2D 目标检测（YOLO 系列）

#### 3.3.1 网格预测解码

YOLO 在特征图网格 $(i, j)$ 上预测边界框，设网格数为 $S \times S$，锚框 $A$ 个。

**Step 1**：网络输出 $(t_x, t_y, t_w, t_h, p_{obj}, p_{c_1}, \ldots, p_{c_K})$。

**Step 2**：解码中心与尺寸（YOLOv5/v8 风格）：

$$
\begin{aligned}
b_x &= \sigma(t_x) + j \\
b_y &= \sigma(t_y) + i \\
b_w &= p_w \cdot e^{t_w} \\
b_h &= p_h \cdot e^{t_h}
\end{aligned}
$$

其中 $(p_w, p_h)$ 为锚框先验尺寸，$\sigma$ 为 Sigmoid。

**Step 3**：类别置信度：

$$
\mathrm{score}_k = p_{obj} \cdot p_{c_k}
$$

保留 $\mathrm{score}_k > \tau_{conf}$ 的检测框。

#### 3.3.2 非极大值抑制（NMS）

**Step 1**：按置信度降序排列检测框 $\{B_1, \ldots, B_N\}$。

**Step 2**：取最高分框 $B_i$，抑制与其 IoU 超过阈值 $\tau_{nms}$ 的框：

$$
\mathrm{IoU}(B_i, B_j) = \frac{|B_i \cap B_j|}{|B_i \cup B_j|}
$$

**Step 3**：重复直至无剩余框。Autonomy 实现在 `common/network/detail/postprocess/nms.cpp`。

#### 3.3.3 IoU 计算（轴对齐框）

框 $B = (x_1, y_1, x_2, y_2)$：

$$
\begin{aligned}
x_1^{int} &= \max(x_1^A, x_1^B), \quad y_1^{int} = \max(y_1^A, y_1^B) \\
x_2^{int} &= \min(x_2^A, x_2^B), \quad y_2^{int} = \min(y_2^A, y_2^B) \\
A_{int} &= \max(0, x_2^{int} - x_1^{int}) \cdot \max(0, y_2^{int} - y_1^{int}) \\
\mathrm{IoU} &= \frac{A_{int}}{A_A + A_B - A_{int}}
\end{aligned}
$$

---

### 3.4 3D 检测与 BEV 表示

#### 3.4.1 鸟瞰图（Bird's Eye View）投影

点云 $p = (x, y, z)$ 投影到 BEV 栅格 $(i, j)$：

$$
i = \left\lfloor \frac{x - x_{min}}{\Delta x} \right\rfloor, \quad
j = \left\lfloor \frac{y - y_{min}}{\Delta y} \right\rfloor
$$

栅格特征可取最大高度 $z_{max}$、点密度或占用概率。

#### 3.4.2 3D 边界框参数化

中心 $(x, y, z)$，尺寸 $(l, w, h)$，航向 $\theta$（绕 $z$ 轴）：

$$
B_{3D} = \{(x, y, z, l, w, h, \theta)\}
$$

8 个角点由旋转矩阵 $R_z(\theta)$ 与半尺寸偏移计算：

$$
c_k = (x, y, z)^\top + R_z(\theta) \cdot (\pm l/2, \pm w/2, \pm h/2)^\top
$$

---

### 3.5 卡尔曼滤波跟踪（多目标）

#### Step 1：状态向量（恒速模型 CV）

$$
\mathbf{x}_k = [x, y, v_x, v_y]^\top
$$

#### Step 2：状态转移

$$
\mathbf{x}_{k+1} = F \mathbf{x}_k + \mathbf{w}_k, \quad
F = \begin{bmatrix}
1 & 0 & \Delta t & 0 \\
0 & 1 & 0 & \Delta t \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

#### Step 3：观测模型（检测框中心）

$$
\mathbf{z}_k = H \mathbf{x}_k + \mathbf{v}_k, \quad
H = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}
$$

#### Step 4：预测与更新

$$
\begin{aligned}
\hat{\mathbf{x}}_{k|k-1} &= F \hat{\mathbf{x}}_{k-1|k-1} \\
P_{k|k-1} &= F P_{k-1|k-1} F^\top + Q \\
K_k &= P_{k|k-1} H^\top (H P_{k|k-1} H^\top + R)^{-1} \\
\hat{\mathbf{x}}_{k|k} &= \hat{\mathbf{x}}_{k|k-1} + K_k (\mathbf{z}_k - H \hat{\mathbf{x}}_{k|k-1})
\end{aligned}
$$

#### Step 5：数据关联（匈牙利算法）

检测 $\{D_j\}$ 与轨迹 $\{T_i\}$ 的代价矩阵 $C_{ij} = 1 - \mathrm{IoU}(D_j, T_i)$，求最小代价匹配。

---

### 3.6 代价地图障碍标记

#### Step 1：射线追踪（Bresenham / 栅格 DDA）

从传感器原点 $o$ 到障碍点 $p$，沿射线标记**自由空间**（cost = 0）；终点标记**障碍**（cost = lethal）。

#### Step 2：膨胀层

对障碍栅格做形态学膨胀，膨胀半径 $r_{infl}$（米）：

$$
\mathrm{cost}(q) = \max_{p \in \mathcal{O}, \|q - p\| \leq r_{\mathrm{infl}}} \mathrm{inflationCost}(\|q - p\|)
$$

常用指数衰减：

$$
\mathrm{cost}(d) = \mathrm{INSCRIBED} \cdot e^{-\lambda (d - r_{robot})}
$$

其中 $d$ 为到最近障碍的距离，$r_{robot}$ 为机器人内切圆半径。

---

### 3.7 符号表

| 符号 | 含义 |
|------|------|
| $K$ | 相机内参矩阵 |
| $T_W^S$ | 传感器到世界的齐次变换 |
| $\tau_{conf}$ | 检测置信度阈值 |
| $\tau_{nms}$ | NMS IoU 阈值 |
| $v$ | 体素边长 |
| $\tau$ | RANSAC 内点距离阈值 |
| $r_{cluster}$ | 欧氏聚类距离阈值 |
| $Q, R$ | 卡尔曼过程/观测噪声协方差 |
