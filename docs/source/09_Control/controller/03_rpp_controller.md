(rpp-controller)=
# 3. Regulated Pure Pursuit

几何路径跟踪：弧长 lookahead 取点，算曲率 $\kappa$，再经 **regulated** 限幅 $v$。源码：`pure_pursuit_controller/`。

弧长 lookahead 取点算曲率，再经 regulated 规则限速度与角速度，几何直观、调参面小。室内差速默认首选之一；需要学习距离场或硬时域约束时再上 NeuPAN/NMPC。

## 何时用

室内默认首选之一；调参面小、CPU 低。要更强动态避障换 MPPI/NeuPAN。

## 公式（摘要）

Lookahead $(x_l,y_l)$，$\kappa\approx 2y_l/(x_l^2+y_l^2)$，$\omega=\kappa v$。

$L_d=\mathrm{clamp}(\alpha|v|+L_{\mathrm{base}},L_{\min},L_{\max})$。

Regulated：$v\leftarrow\min(v_{\mathrm{des}},\,\omega_{\max}/(|\kappa|+\epsilon),\,\alpha_{\mathrm{obs}}d_{\mathrm{obs}},\,\alpha_{\mathrm{goal}}d_{\mathrm{goal}})$。

## 调参

| 场景 | 调整 |
|------|------|
| 窄道 | 减小 $L_d$；开 cost regulated |
| 开阔 | 增大 $L_d$ |
| 振荡 | 增大 $L_{\min}$ |

## 常见坑

- costmap 未 current 时 regulated 障碍项失效或乱限速。
- 路径点过稀时 lookahead 跳点，表现为抖或切弯。

归属 [04](../04_controller_algorithms.md)。
