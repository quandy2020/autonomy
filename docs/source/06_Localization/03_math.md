(localization-math)=
# 3. 数学原理

> 完整算法对比与工程背景见 [09_survey.md](09_survey.md)；Atlas 实现细节见 [06_atlas.md](06_atlas.md)、[05_architecture.md](05_architecture.md)。

### 3.1 问题形式化

机器人在世界坐标系 $\mathcal{W}$ 中的位姿为 $T_{wb} \in SE(3)$（或 2D 情况下 $T_{wb} \in SE(2)$）。定位即估计：

$$
\hat{T}_{wb}(t) = \arg\max_{T} \; p(T \mid z_{1:t}, u_{1:t})
$$

其中 $z_t$ 为传感器观测（图像特征、激光扫描等），$u_t$ 为控制/里程计输入。

**Atlas（视觉 SLAM）** 同时维护：

- 相机位姿序列 $\{T_{cw}^{(i)}\}$（关键帧）
- 稀疏 3D 路标集 $\mathcal{L} = \{\mathbf{X}_j \in \mathbb{R}^3\}$
- 共视图 $\mathcal{G} = (\mathcal{K}, \mathcal{E})$（关键帧为节点，共视边为边）

**AMCL（粒子滤波）** 用粒子集 $\{(\mathbf{x}^{(m)}, w^{(m)})\}_{m=1}^M$ 近似后验，详见 [§3.10](#310-amcl-粒子滤波)。

---

### 3.2 相机投影模型

#### 3.2.1 针孔模型

3D 点 $\mathbf{P}_c = (X, Y, Z)^\top$（相机系）投影到像素 $(u, v)$：

$$
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
= \frac{1}{Z}
\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}
\begin{bmatrix} X \\ Y \\ Z \end{bmatrix}
= \frac{1}{Z} \mathbf{K} \mathbf{P}_c
$$

内参矩阵（`camera::perspective`）：

$$
\mathbf{K} =
\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}
$$

#### 3.2.2 畸变模型（Brown-Conrady）

归一化平面坐标 $(x, y) = (X/Z, Y/Z)$，径向 + 切向畸变：

$$
\begin{aligned}
r^2 &= x^2 + y^2 \\
x_d &= x(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + 2 p_1 xy + p_2(r^2 + 2x^2) \\
y_d &= y(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + p_1(r^2 + 2y^2) + 2 p_2 xy
\end{aligned}
$$

像素坐标：$u = f_x x_d + c_x$，$v = f_y y_d + c_y$。

#### 3.2.3 Bearing Vector（Atlas 内部表示）

Atlas 将特征点转为**单位 bearing vector** $\mathbf{b} \in S^2$：

$$
\mathbf{b} = \frac{\mathbf{K}^{-1} \tilde{\mathbf{p}}}{\|\mathbf{K}^{-1} \tilde{\mathbf{p}}\|}, \quad \tilde{\mathbf{p}} = (u, v, 1)^\top
$$

重投影误差在 bearing 空间计算，对尺度变化更鲁棒。

#### 3.2.4 世界坐标变换

路标 $\mathbf{X}_w$ 在相机系下：

$$
\mathbf{P}_c = \mathbf{R}_{cw} \mathbf{X}_w + \mathbf{t}_{cw}
= \mathbf{T}_{cw} \begin{bmatrix} \mathbf{X}_w \\ 1 \end{bmatrix}
$$

其中 $\mathbf{T}_{cw} = \begin{bmatrix} \mathbf{R}_{cw} & \mathbf{t}_{cw} \\ \mathbf{0}^\top & 1 \end{bmatrix}$ 为 Atlas 存储的 `pose_cw`。

---

### 3.3 ORB 特征与金字塔

#### 3.3.1 图像金字塔

尺度因子 $s = 1.2$（`scale_factor_`），第 $\ell$ 层尺度：

$$
\sigma_\ell = s^\ell, \quad \ell = 0, 1, \ldots, L-1
$$

第 $\ell$ 层图像尺寸约为原图的 $s^{-\ell}$。`orb_params` 预计算：

$$
\sigma[\ell] = s^\ell, \quad \sigma^{-1}[\ell] = s^{-\ell}
$$

对应实现中的 `scale_factors`、`inv_scale_factors` 数组。

#### 3.3.2 FAST 角点 + BRIEF 描述子

1. **FAST-9** 检测角点，阈值 `ini_fast_threshold`（初始化）/ `min_fast_threshold`（跟踪）
2. **Orientation**：强度质心法计算主方向 $\theta$
3. **rBRIEF**：旋转 BRIEF 描述子，256 bit 二进制向量 $\mathbf{d} \in \{0,1\}^{256}$

#### 3.3.3 Hamming 距离

两描述子 $\mathbf{d}_1, \mathbf{d}_2$ 的匹配代价：

$$
d_H(\mathbf{d}_1, \mathbf{d}_2) = \operatorname{popcount}(\mathbf{d}_1 \oplus \mathbf{d}_2)
$$

ORB 匹配采用 **ratio test**（Lowe）：

$$
\frac{d_H(\mathbf{d}_q, \mathbf{d}_n)}{d_H(\mathbf{d}_q, \mathbf{d}_b)} < \rho, \quad \rho \approx 0.8
$$

其中 $\mathbf{d}_n$ 为最近邻，$\mathbf{d}_b$ 为次近邻。

---

### 3.4 两视图几何

#### 3.2.1 对极约束与本质矩阵

两帧 bearing $\mathbf{b}_1, \mathbf{b}_2$ 对应同一 3D 点，满足：

$$
\mathbf{b}_2^\top \mathbf{E}_{21} \mathbf{b}_1 = 0
$$

本质矩阵 $\mathbf{E}_{21} = [\mathbf{t}_{21}]_\times \mathbf{R}_{21}$，其中 $[\mathbf{v}]_\times$ 为反对称矩阵：

$$
[\mathbf{v}]_\times =
\begin{bmatrix} 0 & -v_z & v_y \\ v_z & 0 & -v_x \\ -v_y & v_x & 0 \end{bmatrix}
$$

**5 点法**（`essential_5pt`）：5 对匹配 → 最多 10 个 $\mathbf{E}$ 候选，RANSAC 选最优。

**8 点法**（`compute_E_21_nonminimal`）：SVD 分解求解，RANSAC 后精化。

#### 3.4.2 单应矩阵（平面场景）

共面场景用单应 $\mathbf{H}_{21}$：

$$
\mathbf{b}_2 \sim \mathbf{H}_{21} \mathbf{b}_1
$$

初始化时同时估计 $\mathbf{E}$ 与 $\mathbf{H}$，选内点更多、视差更大者（`initialize::perspective`）。

#### 3.4.3 从 $\mathbf{E}$ 恢复 $R, t$

SVD：$\mathbf{E} = \mathbf{U} \mathbf{\Sigma} \mathbf{V}^\top$，得到 4 组 $(\mathbf{R}, \mathbf{t})$ 候选，通过 **cheirality check**（3D 点在两相机前方）确定唯一解。

#### 3.4.4 三角化

两帧位姿 $\mathbf{T}_{c1w}, \mathbf{T}_{c2w}$，匹配 bearing $\mathbf{b}_1, \mathbf{b}_2$，线性三角化（DLT）：

$$
\mathbf{A} \mathbf{X}_w = \mathbf{0}, \quad
\mathbf{A} =
\begin{bmatrix}
\mathbf{b}_1^\top \mathbf{P}_1 \\
\mathbf{b}_2^\top \mathbf{P}_2
\end{bmatrix}
$$

$\mathbf{P}_i = \mathbf{K}_i [\mathbf{R}_{ciw} \mid \mathbf{t}_{ciw}]$ 为投影矩阵。SVD 取 $\mathbf{A}$ 最小奇异值对应向量，归一化得 $\mathbf{X}_w$。

**深度比检验**（`mapping_module::create_new_landmarks`）：

$$
\frac{\|\mathbf{t}_1 - \mathbf{X}_w\|}{\|\mathbf{t}_2 - \mathbf{X}_w\|} \in [\rho_{\min}, \rho_{\max}]
$$

基线距离阈值：

$$
\|\mathbf{t}_1 - \mathbf{t}_2\| > \max\left\{ \tau_b, \; \eta \cdot d_{\mathrm{med}} \right\}
$$

默认 $\eta = 0.02$（`baseline_dist_thr_ratio_`）。

---

### 3.5 PnP 位姿估计

已知 3D–2D 对应 $\{(\mathbf{X}_w^{(j)}, \mathbf{b}^{(j)})\}$，求 $\mathbf{T}_{cw}$。

#### 3.5.1 EPnP + RANSAC

`pnp_solver::find_via_ransac` 流程：

1. 随机采样最小集（4 点）
2. EPnP 求解候选位姿
3. 重投影检验内点，Huber 加权代价
4. 迭代至收敛，Gauss-Newton 精化（`gauss_newton_num_iter = 10`）

#### 3.5.2 重投影误差（bearing 空间）

$$
e_j = \arccos\left( \mathbf{b}_j^\top \hat{\mathbf{b}}_j \right)
$$

其中 $\hat{\mathbf{b}}_j$ 为将 $\mathbf{X}_w^{(j)}$ 用当前 $\mathbf{T}_{cw}$ 投影后的 bearing。内点判定：$e_j < \tau_{\text{rad}}$（默认 0.2°）。

---

### 3.6 位姿图优化（g2o）

Atlas 后端统一使用 **g2o**，优化目标为 **非线性最小二乘**：

$$
\mathbf{T}^*, \mathbf{X}^* = \arg\min_{\mathbf{T}, \mathbf{X}} \sum_k \rho\left( \| \mathbf{r}_k(\mathbf{T}, \mathbf{X}) \|_{\mathbf{\Omega}_k}^2 \right)
$$

其中 $\rho$ 为鲁棒核（Huber / Tukey），$\mathbf{\Omega}_k$ 为信息矩阵。

#### 3.6.1 重投影边（SE3）

对观测 $(\mathbf{b}_{obs}, \mathbf{X}_w, \mathbf{T}_{cw})$：

$$
\mathbf{r}_{\text{reproj}} = \mathbf{b}_{obs} - \pi(\mathbf{T}_{cw}, \mathbf{X}_w)
$$

信息矩阵与特征金字塔层相关：

$$
\mathbf{\Omega} = \frac{1}{\sigma_\ell^2} \mathbf{I}_2, \quad \sigma_\ell = s^\ell
$$

#### 3.6.2 位姿优化（Tracking 每帧）

`pose_optimizer::optimize` 仅优化当前帧 $\mathbf{T}_{cw}$，路标固定：

$$
\min_{\mathbf{T}_{cw}} \sum_{j \in \mathcal{O}} \rho\left( \|\mathbf{r}_j\|^2 \right)
$$

Levenberg-Marquardt 迭代，外点剔除后重优化。

#### 3.6.3 局部 Bundle Adjustment（LBA）

`local_bundle_adjuster::optimize` 优化局部窗口 $\mathcal{W}$ 内关键帧位姿与路标：

$$
\min_{\{\mathbf{T}_i\}, \{\mathbf{X}_j\}} \sum_{(i,j) \in \mathcal{E}} \rho\left( \|\mathbf{r}_{ij}\|_{\mathbf{\Omega}_{ij}}^2 \right)
$$

局部窗口：当前关键帧 + 共视关键帧（默认最多 60 帧 `max_num_local_keyfrms_`）。

#### 3.6.4 全局 Bundle Adjustment（Loop BA）

回环闭合后在独立线程执行 GBA，优化全部（或子集）关键帧与路标，消除累积漂移。

---

### 3.7 回环检测与 Sim3 校正

#### 3.7.1 Bag-of-Words（BoW）

ORB 描述子量化到视觉词袋，关键帧 $i$ 的 BoW 向量：

$$
\mathbf{v}_i = (w_1, w_2, \ldots, w_V)^\top, \quad w_k = \mathrm{tfidf}(k)
$$

其中 $\mathrm{tfidf}(k)$ 为词 $k$ 的 TF-IDF 权重。
`loop_detector::detect_loop_candidates` 用 BoW 数据库检索相似关键帧，再几何验证。

#### 3.7.2 Sim3 变换（单目回环）

单目 SLAM 存在尺度漂移，回环估计 **Sim(3)**：

$$
\mathbf{S} =
\begin{bmatrix} s \mathbf{R} & \mathbf{t} \\ \mathbf{0}^\top & 1 \end{bmatrix}, \quad s > 0
$$

`transform_optimizer` 优化 Sim3 使当前关键帧与候选关键帧的 3D–3D 匹配对齐。

#### 3.7.3 回环校正

检测到回环后（`global_optimization_module::correct_loop`）：

1. 计算校正前/后 Sim3：$\mathbf{S}_{nw}^{\text{before}}, \mathbf{S}_{nw}^{\text{after}}$
2. 传播校正到共视关键帧与路标
3. 融合重复路标，更新共视图
4. 异步启动 Loop BA

共视关键帧 $n$ 的路标校正：

$$
\mathbf{X}_w' = s \mathbf{R}_{\Delta} \mathbf{X}_w + \mathbf{t}_{\Delta}
$$

---

### 3.8 运动模型与跟踪状态机

#### 3.8.1 恒速运动模型

相邻帧位姿增量（`twist_`）：

$$
\mathbf{T}_{c_{k}w} = \Delta \mathbf{T} \cdot \mathbf{T}_{c_{k-1}w}
$$

其中 $\Delta \mathbf{T} = \mathbf{T}_{c_{k-1}w} \mathbf{T}_{c_{k-2}w}^{-1}$（`update_motion_model`）。

#### 3.8.2 跟踪状态

| 状态 | 条件 | 行为 |
|------|------|------|
| `Initializing` | 地图未建立 | 两帧初始化 / 5 点法 |
| `Tracking` | 足够内点 | 正常跟踪 + 局部地图优化 |
| `Lost` | 内点不足 | BoW 重定位 / 用户 `relocalize_by_pose` |

丢失判定：跟踪路标数 $N_{\text{track}} < N_{\min}$ 且重定位失败。

---

### 3.9 立体与 RGB-D 深度

#### 3.9.1 立体视差

校正后对应点 $(u_L, v)$ 与 $(u_R, v)$，视差 $d = u_L - u_R$，深度：

$$
Z = \frac{f_x \cdot b}{d}
$$

其中 $b$ 为基线，`focal_x_baseline_ = f_x \cdot b`（EuRoC 配置 `47.91`）。

#### 3.9.2 RGB-D

深度图值 $d_{\text{pix}}$ 转真实深度：

$$
Z = \frac{d_{\text{pix}}}{f_{\mathrm{dmap}}}
$$

其中 $f_{\mathrm{dmap}}$ 为 `depthmap_factor_`。

TUM RGB-D 典型 `depthmap_factor_ = 5000`（mm → m）。

---

### 3.10 AMCL 粒子滤波

> AMCL 配置见 `config/localization/amcl/amcl.lua`，算法实现待集成。以下为 nav2_amcl 对齐的数学模型。

#### 3.10.1 贝叶斯滤波

$$
p(\mathbf{x}_t \mid z_{1:t}, u_{1:t}) \propto p(z_t \mid \mathbf{x}_t) \int p(\mathbf{x}_t \mid u_t, \mathbf{x}_{t-1}) p(\mathbf{x}_{t-1} \mid z_{1:t-1}, u_{1:t-1}) \, d\mathbf{x}_{t-1}
$$

#### 3.10.2 运动模型（差速驱动）

里程计增量 $(\delta_{\text{rot1}}, \delta_{\text{trans}}, \delta_{\text{rot2}})$，噪声参数 $\alpha_1 \ldots \alpha_4$：

$$
\begin{aligned}
\delta_{\text{rot1}}' &\sim \mathcal{N}(\delta_{\text{rot1}}, \alpha_1 |\delta_{\text{rot1}}| + \alpha_2 \|\delta_{\text{trans}}\|) \\
\delta_{\text{trans}}' &\sim \mathcal{N}(\delta_{\text{trans}}, \alpha_3 \|\delta_{\text{trans}}\| + \alpha_4(|\delta_{\text{rot1}}| + |\delta_{\text{rot2}}|)) \\
\delta_{\text{rot2}}' &\sim \mathcal{N}(\delta_{\text{rot2}}, \alpha_1 |\delta_{\text{rot2}}| + \alpha_2 \|\delta_{\text{trans}}\|)
\end{aligned}
$$

#### 3.10.3 激光观测模型（Likelihood Field）

预计算距离场 $\mathrm{dist}(x, y)$（到最近占据栅格），光束端点 $(x_e, y_e)$ 似然：

$$
p(z_t \mid \mathbf{x}_t) \propto \sum_i \left( w_{\text{hit}} \mathcal{N}(d_i; 0, \sigma_{\text{hit}}^2) + w_{\text{rand}} \frac{1}{r_{\max}} \right)
$$

默认 `z_hit=0.95`, `z_rand=0.05`, `sigma_hit=0.2`。

#### 3.10.4 粒子更新

1. **预测**：按运动模型采样 $M$ 个粒子
2. **更新**：激光似然加权 $w^{(m)} \leftarrow w^{(m)} \cdot p(z_t \mid \mathbf{x}^{(m)})$
3. **重采样**：$N_{\text{eff}} = 1 / \sum (w^{(m)})^2 < N_{\text{th}}$ 时系统重采样
4. **输出**：加权均值或 MAP 粒子作为 $\hat{\mathbf{x}}_t$，广播 `map→odom` TF

---

### 3.11 算法流水线总览

$$
\mathrm{Image}
\xrightarrow{\mathrm{ORB}}
\mathrm{Features}
\xrightarrow{\mathrm{Matching}}
\mathrm{Match}_{2\mathrm{D}/3\mathrm{D}}
\xrightarrow{\mathrm{PnP/BA}}
\hat{\mathbf{T}}_{cw}
\xrightarrow{\mathrm{Keyframe}}
\mathrm{LBA}
\xrightarrow{\mathrm{BoW}}
\mathrm{Loop\ Closure}
\xrightarrow{\mathrm{Sim3+GBA}}
\mathrm{Globally\ Consistent\ Map}
$$

| 阶段 | 核心公式 | Atlas 模块 |
|------|----------|------------|
| 特征 | FAST + rBRIEF | `feature/orb_extractor` |
| 初始化 | $\mathbf{E}, \mathbf{H}$ + 三角化 | `initialize/` |
| 跟踪 | PnP + 位姿优化 | `tracking_module` |
| 建图 | 三角化 + LBA | `mapping_module` |
| 回环 | BoW + Sim3 + GBA | `global_optimization_module` |
