# Argus 文档指南

ROS-free **FAST-LIVO2** 算法栈 + 鲁棒定位感知层（退化 / 动态 / 门控 / 回环 / 跨会话 / 自清洁 / BnB 重定位）。

## 阅读路径

| 角色 | 顺序 |
|------|------|
| 集成宿主 | [01 概览](01_overview.md) → [10 用法](10_usage.md) → [09 配置](09_config.md) → [12 生产](12_production.md) |
| 算法研发 | [02 架构](02_architecture.md) → [03 时序](03_sequence.md) → [04 数学](04_math.md) → [05–08 专题](05_frontend_fusion.md) |
| 评测 / 数据 | [10 用法](10_usage.md) → [`scripts/README.md`](../scripts/README.md) → [11 测试](11_test.md) |

## 文档索引

| 编号 | 文件 | 内容 |
|------|------|------|
| 00 | 本文 | 入口与阅读路径 |
| 01 | [01_overview.md](01_overview.md) | 问题域、模态、硬约束 |
| 02 | [02_architecture.md](02_architecture.md) | 分层、依赖、双后端 |
| 03 | [03_sequence.md](03_sequence.md) | 线程模型与时序图 |
| 04 | [04_math.md](04_math.md) | ESIKF / VIO / FIM / SC / BnB |
| 05 | [05_frontend_fusion.md](05_frontend_fusion.md) | 动态过滤、退化、门控、强度 |
| 06 | [06_map.md](06_map.md) | 体素 / 建图包 / 自清洁 |
| 07 | [07_loop_closure.md](07_loop_closure.md) | 回环、PoseGraph、跨会话 |
| 08 | [08_relocalization.md](08_relocalization.md) | BnB / 3D-BBS |
| 09 | [09_config.md](09_config.md) | YAML 与 calibration |
| 10 | [10_usage.md](10_usage.md) | 构建、宿主 API、离线评测 |
| 11 | [11_test.md](11_test.md) | 单测、场景、性能预算 |
| 12 | [12_production.md](12_production.md) | 生产清单与发布检查 |

历史任务拆解：[`plans/2026-10-17-argus-robustness.md`](plans/2026-10-17-argus-robustness.md)。  
源码头文件跳转：[`api/README.md`](api/README.md)。

## 硬约束（全文适用）

1. ESIKF **单线程串行**；退化 / 动态 / 门控在更新前完成。
2. 重定位专用线程池 **仅** 服务 BnB，不进实时估计链路。
3. 配置驱动模态：生产为 **LIVO / LIO / LO**；禁止无 LiDAR 的 ONLY_VIO 上线。
4. 无 ROS；宿主单线程 `Process`，见 [12_production.md](12_production.md)。
