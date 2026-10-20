# 0. Tools 指南

## 背景

离线 bag 播不出、Docker 里缺数据集、`No module named autonomy.tools`——开发和联调时真正要敲的命令散在 `scripts/`、`tools/python/` 和各子包里。整机怎么起见 [04 Running](../04_Running/00_guide.md)；本章收 **可复用的命令与脚本入口**。

## 思路

- **bag_convert**：源码在 `autonomy/autonomy/tools/bag_convert/`，调用仍是 `python -m autonomy.tools.bag_convert`。  
- **数据集发布** 与 Running 的 **`AUTONOMY_DATASETS`** 一起看（[08 数据集](08_datasets.md)）。  
- **`evaluator` 目录几乎是空 README** — 不要当成可用评测套件。  
- 顶层包（autoviz / autodriver / automanip）文档不在本树 → [09 其它包](09_ecosystem.md)。


## 步骤

### 1. 按问题选页

| 文档 | 解决什么 |
|------|----------|
| [01 概览](01_overview.md) | 工具落点一览 |
| [02 快速开始](02_quickstart.md) | 常用命令串 |
| [03 开发脚本](03_dev_scripts.md) | `scripts/`、`tools/python/` |
| [04 bag_convert](04_bag_convert.md) | ROS1 bag → autolink record |
| [05 监控](05_monitor.md) | system monitor |
| [06 Autolink 工具](06_autolink_tools.md) | CLI / recorder |
| [07 Docker 脚本](07_docker_scripts.md) | `run_autonomy.py` 等 |
| [08 数据集](08_datasets.md) | 离线序列按时间发布（Argus 等） |
| [09 其它包](09_ecosystem.md) | autoviz / autodriver / automanip |

**Cartographer 示例 bag** → [04](04_bag_convert.md) + [Localization](../06_Localization/00_guide.md)。  
**Argus 离线** → [08](08_datasets.md)。  
**容器里跑** → [07](07_docker_scripts.md) + [Running · Docker](../04_Running/05_docker_runtime.md)。

### 2. 常见坑

| 现象 | 先查 |
|------|------|
| `No module named autonomy.tools...` | `PYTHONPATH`；源码根；python 包路径 |
| record 播不出 | 话题名 leading `/`；preset 是否匹配 |
| Docker 无数据 | 挂载字符串格式 |
