# 概览

## 1. 问题域

激光–惯性–视觉里程计在开放环境需同时满足：

| 痛点 | 对策（模块） |
|------|--------------|
| 隧道 / 长廊弱几何 | FIM 退化检测 + 各向异性协方差 / 强度辅助 |
| 行人 / 车辆动态污染残差 | 前端动态过滤（时空法向 / 语义） |
| 多传感器置信度不一 | ConfidenceGating 准入与加权 |
| 长距离漂移 | Scan Context + 异步 PoseGraph |
| 跨天同路线 | 跨会话链式回环 + 地图包先验 |
| 跟踪丢失 | BnB / 3D-BBS 专用池重定位 |
| 长期地图污染 | 体素置信度 + 异步自清洁 |

算法核移植自 FAST-LIVO2（`atlas/livo`），**无** ROS/catkin、vikit、Sophus；**无** 反向依赖 atlas。

## 2. 生产模态

| 模态 | 传感器 | 说明 |
|------|--------|------|
| **LIVO** | LiDAR + IMU + Camera | 完整路径：体素 ESIKF + `VIOManager` 直接法 |
| **LIO** | LiDAR + IMU | 默认稳健基线 |
| **LO** | LiDAR only | 无 IMU 时 `ImuProcess` 降级传播；精度通常差于 LIO |
| ONLY_VIO | Camera (+IMU) | **加载拒绝**：VIO 依赖体素深度，禁止上线 |

配置：`odometry.type` + `odometry.modalities`（见 [09_config.md](09_config.md)）。

## 3. 系统边界

**库内（libargus）**

- 状态估计、地图、回环、重定位、配置加载。
- 输入：`AddLidar` / `AddImu` / `AddImage`；输出：`OdometryOutput`（状态、`ProcessStatus`）。

**库外**

- 数据集 I/O：`apps/datasets/`（KITTI / Oxford Spires / Newer College）。
- Bag/mcap 回放、ROS 桥：由宿主提供。
- 评测脚本：`scripts/`（下载、ATE、Open3D）。

## 4. 能力一览

| 能力 | 默认 | 入口 |
|------|------|------|
| LO / LIO / LIVO | — | `LivoOdometry::Process` |
| 退化 + 各向异性 | on | `robustness.degeneracy` |
| 强度辅助 | on | `intensity_aided` |
| 动态过滤 | off | `frontend.dynamic_filter` |
| Confidence gating | off | `confidence_gating` |
| 异步回环 + SE(3) PG | off | `loop_closure` |
| 跨会话 SC | 随回环 | `cross_session` / `RegisterPriorSession` |
| BnB 重定位 | on（无先验则无结果） | `relocalization` |
| 地图包 v3 | on | `mapping` / `MapBuilder` |
| 自清洁 | off | `map.self_cleaning` |

## 5. 相关文档

- 分层与依赖 → [02_architecture.md](02_architecture.md)
- 线程与时序 → [03_sequence.md](03_sequence.md)
- 公式 → [04_math.md](04_math.md)
- 宿主与脚本 → [10_usage.md](10_usage.md)
