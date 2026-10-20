(localization-math)=
# 3. 数学原理

> 对比选型见 [§9](09_survey.md)；架构见 [§5](05_architecture.md)。

### 3.1 问题形式化

位姿 $T_{wb}\in SE(3)$（2D 为 $SE(2)$）：

$$
\hat{T}_{wb}(t)=\arg\max_T\,p(T\mid z_{1:t},u_{1:t})
$$

视觉 SLAM 同时维护关键帧 $\{T_{cw}^{(i)}\}$、路标 $\mathcal{L}$、共视图 $\mathcal{G}$。粒子滤波用粒子集近似后验（仓库未合入独立 AMCL 后端）。

---

### 3.2 相机投影

**针孔**：$\tilde{\mathbf{p}}=\frac{1}{Z}\mathbf{K}\mathbf{P}_c$。

**畸变**（Brown-Conrady）：径向 $k_{1..3}$ + 切向 $p_{1,2}$，再乘内参得像素。

**Bearing**：$\mathbf{b}=\mathbf{K}^{-1}\tilde{\mathbf{p}}/\|\cdot\|\in S^2$。路标：$\mathbf{P}_c=\mathbf{R}_{cw}\mathbf{X}_w+\mathbf{t}_{cw}$。

---

### 3.3 ORB

尺度 $\sigma_\ell=s^\ell$（$s=1.2$）。流程：FAST-9 → 强度质心方向 → rBRIEF（256 bit）。

Hamming：$d_H=\mathrm{popcount}(\mathbf{d}_1\oplus\mathbf{d}_2)$；Lowe ratio $\rho\approx 0.8$。

---

### 3.4 两视图几何

对极：$\mathbf{b}_2^\top\mathbf{E}_{21}\mathbf{b}_1=0$，$\mathbf{E}=[\mathbf{t}]_\times\mathbf{R}$。5 点 / 8 点 + RANSAC。平面场景用 $\mathbf{H}$。

SVD 分解 $\mathbf{E}$ 得 4 组 $(R,t)$，cheirality 选解。三角化 DLT：$\mathbf{A}\mathbf{X}_w=\mathbf{0}$；深度比与基线阈值见实现 `baseline_dist_thr_ratio_`（默认 $0.02$）。

---

### 3.5 PnP

EPnP + RANSAC（4 点最小集）→ 重投影内点 → Gauss-Newton 精化。

Bearing 误差：$e_j=\arccos(\mathbf{b}_j^\top\hat{\mathbf{b}}_j)$，阈值约 $0.2^\circ$。

---

### 3.6 位姿图 / BA

$$
\mathbf{T}^*,\mathbf{X}^*=\arg\min\sum_k\rho\big(\|\mathbf{r}_k\|_{\mathbf{\Omega}_k}^2\big)
$$

| 优化 | 变量 | 场景 |
|------|------|------|
| 位姿优化 | 仅 $T_{cw}$ | Tracking 每帧 |
| LBA | 局部窗口关键帧 + 路标 | Mapping（$\le 60$ 帧） |
| GBA | 全局（子集） | 回环后 |

信息矩阵：$\mathbf{\Omega}=\sigma_\ell^{-2}\mathbf{I}$。

---

### 3.7 回环与 Sim3

BoW（TF-IDF）检索候选 → 几何验证。单目估计 Sim(3)（含尺度 $s$），校正共视关键帧与路标后异步 Loop BA。

---

### 3.8 跟踪状态

恒速模型：$\mathbf{T}_{c_k w}=\Delta\mathbf{T}\cdot\mathbf{T}_{c_{k-1}w}$。

| 状态 | 行为 |
|------|------|
| Initializing | 两帧 SfM |
| Tracking | PnP + 局部优化 |
| Lost | BoW / `relocalize_by_pose` |

---

### 3.9 立体 / RGB-D

立体：$Z=f_x b/d$。RGB-D：$Z=d_{\mathrm{pix}}/f_{\mathrm{dmap}}$（TUM 典型 $f_{\mathrm{dmap}}=5000$）。

---

### 3.11 视觉流水线

```
Image → ORB → Match → PnP/BA → Keyframe → LBA → BoW Loop → Sim3+GBA
```

---

### 3.12 Cartographer

**扫描匹配**（子图 $M_i$）：

$$
\xi^*=\arg\min_\xi\sum_{\mathbf{p}\in\mathcal{Z}}\big(1-M_i(T_\xi\mathbf{p})\big)^2+\lambda_{\mathrm{trans}}\|\Delta t\|^2+\lambda_{\mathrm{rot}}\Delta\theta^2
$$

累积 `num_range_data` 后冻结子图。占据更新用 log-odds。

**位姿图**：局部约束 + Fast CSM 回环；每 `optimize_every_n_nodes` 优化；Huber 核。纯定位加载冻结子图，`pure_localization_trimmer` 限制活跃子图数。

用法：[07_cartographer/guide.md](07_cartographer/guide.md)。
