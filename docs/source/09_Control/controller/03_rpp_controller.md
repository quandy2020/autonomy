(rpp-controller)=
# 3. Regulated Pure Pursuit

归属 [§4](../04_controller_algorithms.md)。几何跟踪；源码 `pure_pursuit_controller/` 。

### 定义

路径变换到机器人系，弧长 $L_d$ 取 lookahead $(x_l,y_l)$：

$$
\kappa=\frac{2y_l}{x_l^2+y_l^2}\approx\frac{2y_l}{L_d^2},\quad
\omega=\kappa\,v
$$

$L_d=\mathrm{clamp}(\alpha|v|+L_{\mathrm{base}},L_{\min},L_{\max})$。

Regulated：

$$
v\leftarrow\min\big(v_{\mathrm{des}},\,\omega_{\max}/(|\kappa|+\epsilon),\,\alpha_{\mathrm{obs}}d_{\mathrm{obs}},\,\alpha_{\mathrm{goal}}d_{\mathrm{goal}}\big)
$$

### 结构

`GetLookAheadPoint`（弧长插值）· `CircleSegmentIntersection`（圆-线段交点辅助）。

### 用法 / 调参

| 场景 | 调整 |
|------|------|
| 窄道 | 减小 $L_d$；开 cost regulated |
| 开阔 | 增大 $L_d$ |
| 振荡 | 增大 $L_{\min}$ |
