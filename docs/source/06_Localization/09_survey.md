# 9. 定位算法综述

> 公式 [§3](03_math.md)；用法 [§4](04_usage.md)。

### 9.1 后端对照

| 能力 | Cartographer | Atlas | Argus |
|------|--------------|-------|-------|
| 2D 激光 SLAM | 默认 | 可选 | — |
| 3D / LIO | — | | — |
| 视觉 / VIO | — | — | LIVO |
| 输出 TF / 地图 | — | | — |

数据流：传感器 → Cartographer / Atlas / Argus → TF → Map / Planning / Control。

### 9.2 选型

| 场景 | 推荐 |
|------|------|
| 室内 2D 建图定位 | Cartographer |
| 3D 激光 + IMU | Atlas / Argus |
| 多相机 / 异构传感器 | Atlas |

### 9.3 参见

[用法](04_usage.md) · [数学](03_math.md) · [Cartographer](07_cartographer/guide.md) · [Atlas](08_atlas/guide.md) · [Argus](06_argus.md) · [架构](05_architecture.md)
