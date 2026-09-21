# Atla2 文档中心

本目录为 Atla2（统一 VO / VIO / LIO / LIVO）的设计与工程文档。总览原稿见 [framework.md](framework.md)；专题文档按「设计 → 需求 → 接口 → 部署 → 测试 → 标定」组织。

## 文档地图

| 分类 | 文档 | 说明 |
|------|------|------|
| 总览 | [framework.md](framework.md) | 重构目标、完整目录树、模式对比、嵌入式要点 |
| 结构设计 | [design/structure.md](design/structure.md) | 模块拆分、依赖关系、目录职责 |
| 架构设计 | [design/architecture.md](design/architecture.md) | 分层架构、状态机、配置驱动 |
| 坐标系 | [design/coordinate_frames.md](design/coordinate_frames.md) | body / imu / cam / lidar / ENU |
| 数据流 | [design/data_flow.md](design/data_flow.md) | 传感器 → 前端 → 后端 → 地图 |
| 融合设计 | [design/sensor_fusion.md](design/sensor_fusion.md) | 松/紧耦合与降级策略 |
| 需求分析 | [requirements/analysis.md](requirements/analysis.md) | 功能 / 非功能 / 约束 |
| 应用需求 | [requirements/application.md](requirements/application.md) | 机型场景与平台配置映射 |
| 接口分析 | [interface/analysis.md](interface/analysis.md) | Sensor / Frontend / Backend / SlamSystem |
| Proto 消息 | [../proto/README.md](../proto/README.md) | VO/VIO/LO/LIO/LIVO，复用 automsgs |
| 部署验证 | [deployment/verification.md](deployment/verification.md) | 构建、安装、交叉编译、上板检查 |
| 测试评测 | [testing/evaluation.md](testing/evaluation.md) | 单测 / 集成 / 指标 / CI |
| 标定指南 | [calibration/guide.md](calibration/guide.md) | 相机–IMU / 激光–IMU / 联合标定 |

## 阅读顺序（新人）

1. [requirements/analysis.md](requirements/analysis.md) — 为什么做
2. [design/structure.md](design/structure.md) + [design/architecture.md](design/architecture.md) — 怎么拆
3. [interface/analysis.md](interface/analysis.md) — 怎么接
4. [design/data_flow.md](design/data_flow.md) — 怎么跑
5. [deployment/verification.md](deployment/verification.md) + [testing/evaluation.md](testing/evaluation.md) — 怎么验

## 与代码对应

| 代码路径 | 文档 |
|----------|------|
| `common/` `sensor/` `frontend/` `backend/` `map/` `fusion/` `pipeline/` | 结构设计 / 架构 |
| `config/platforms/*.yaml` | 应用需求 |
| `apps/` `scripts/` | 部署验证 |
| `test/` `tools/benchmark/` | 测试评测 |
| `apps/calibration_tool/` | 标定指南 |
