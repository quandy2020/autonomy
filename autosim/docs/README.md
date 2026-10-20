# AutoSim 文档

多后端传感器–执行器桥（Habitat / Fake / Isaac）的维护文档。  
包级快速入口见 [`../README.md`](../README.md)；演示动图见 [`../images/`](../images/)。

## 阅读顺序

| 角色 | 建议路径 |
|------|----------|
| 新手 | [00 指南](00_guide.md) → [03 快速开始](03_quickstart.md) → [06 话题](06_topics.md) |
| 集成 Autoviz / 导航 | [05 配置](05_configuration.md) → [08 地图与可视化](08_map_and_viz.md) |
| Isaac / USD | [07 Isaac](07_isaac.md) → [10 排错](10_troubleshooting.md) |
| 改代码 | [02 架构](02_architecture.md) → [04 后端](04_backends.md) → [09 测试](09_testing.md) |
| 模仿学习 | [11 数据采集](11_dataset.md) → [02 架构](02_architecture.md) |

## 文档目录

| § | 文件 | 内容 |
|---|------|------|
| 00 | [00_guide.md](00_guide.md) | 文档地图、边界、术语 |
| 01 | [01_overview.md](01_overview.md) | 目标、能力、非目标 |
| 02 | [02_architecture.md](02_architecture.md) | 包树 / Runner / Backend / 数据流 |
| 03 | [03_quickstart.md](03_quickstart.md) | 安装与三种后端启动 |
| 04 | [04_backends.md](04_backends.md) | Habitat · Fake · Isaac 对照 |
| 05 | [05_configuration.md](05_configuration.md) | YAML、mode、噪声、档案 |
| 06 | [06_topics.md](06_topics.md) | Autolink 通道一览 |
| 07 | [07_isaac.md](07_isaac.md) | USD、PhysX、相机、真值地图 |
| 08 | [08_map_and_viz.md](08_map_and_viz.md) | `/map`、`/overall/map`、Autoviz |
| 09 | [09_testing.md](09_testing.md) | pytest 与冒烟 |
| 10 | [10_troubleshooting.md](10_troubleshooting.md) | 常见问题 |
| 11 | [11_dataset.md](11_dataset.md) | LeRobot 模仿学习采集 |

## 维护约定

- 文件名：`NN_topic.md`（两位序号 + 英文短词），序号即阅读主序。
- 每篇开头：一句话职责 +「相关文档」表；正文用短节，少重复，细节链到源码路径。
- 配置真相以 `config/*.yaml` 与 `autosim/runtime/settings.py` 为准；文档只写稳定约定。
- 大体积场景资源不入库，说明写在 [07 Isaac](07_isaac.md) 与 [`../usd/README.md`](../usd/README.md)。
