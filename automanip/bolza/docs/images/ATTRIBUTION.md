# Image attribution

Textbook schematics for [`04_robotic_examples.md`](../04_robotic_examples.md)（§4.3–§4.7 各一图）。

## Pipeline

Source of truth: `*.tex` (TikZ + `amsmath`, Computer Modern via [tectonic](https://tectonic-typesetting.github.io/)).

```bash
# requires: tectonic, pymupdf (pip install pymupdf)
./render_all.sh
```

Outputs per figure: `.pdf` (compile), `.png` / `.svg` (raster/vector for Markdown).

## Figures

| Source | Content |
|:---|:---|
| `double_integrator.tex` | 1D mass: $p$, $\dot{p}$, $u$, $\ddot{p}=u$ |
| `cartpole.tex` | Cart-pole: $m_c$, $m_p$, $\ell$, $p$, $\theta$, $F$, $g$ |
| `ballbot.tex` | Ballbot: $r_b$, $p_{b,x}$, $\theta$, $\tau_i$, $q$, $x$, $u$ |
| `quadrotor.tex` | Quadrotor schematic: $XYZ$, $p$, airframe, $F_z$, $g$, $\phi,\theta,\psi$ |

`cartpole` geometry inspired by [Cart-pendulum.svg](https://commons.wikimedia.org/wiki/File:Cart-pendulum.svg) (CC0).

## External figures

| File | Source |
|:---|:---|
| `legged_robot.png` | Adapted from Fig. 1 of DOI [10.13973/j.cnki.robot.230304](https://doi.org/10.13973/j.cnki.robot.230304)（机体 $\{B\}$、世界系 $\{W\}$、足端 $p_1$、摩擦锥 $\Sigma_4$ / $F_4$）. Original: `legged_robot_source.jpg`. |
