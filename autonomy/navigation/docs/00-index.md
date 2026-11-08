# 00 · Navigation 文档

本目录是 `autonomy/navigation` 的文档入口。根目录 [README](../README.md) 只保留
项目定位、快速开始和文档索引。

## 命名约定

- 文档统一 `NN-topic.md`：`00` 索引，`01` 起为内容文档。
- 新增文档使用下一个未占用编号；不要插入中间编号。
- 根目录 [README](../README.md) 不编号，只做入口。

## 阅读顺序

| 文档 | 内容 | 适合 |
| --- | --- | --- |
| [01-requirements.md](01-requirements.md) | 设计目的、28 条功能/28 条非功能需求、追溯与优先级 | 需求评审 |
| [02-architecture.md](02-architecture.md) | 进程/组件/分层、线程模型、插件扩展点 | 第一次接触代码 |
| [03-data-flow.md](03-data-flow.md) | 从 autosim 传感器到 `/cmd_vel` 的完整数据流 | 联调、排障 |
| [04-design.md](04-design.md) | v2 最终设计基线（契约、proto、DAG、验收） | 设计评审 |
| [05-simulation.md](05-simulation.md) | autosim 仿真接入、目标/探索/恢复 | 仿真联调 |
| [06-models.md](06-models.md) | DeFM 制品、ONNX I/O、VNTPA 模型状态 | 模型部署 |
| [07-build.md](07-build.md) | 构建、测试、安装、Sanitizer、benchmark | 开发/CI |
| [08-acceptance.md](08-acceptance.md) | 验收矩阵与测试映射 | 验收 |
| [09-style.md](09-style.md) | Doxygen/命名/注释规范 | 提交前 |
| [10-tools.md](10-tools.md) | 配置校验、benchmark、目标发送、脚本 | 工具使用 |
| [12-configuration.md](12-configuration.md) | 配置分层与模块化约定 | 配置维护 |
| [13-todo.md](13-todo.md) | TODO、闭环分级与验收顺序 | 排期/验收 |
| [14-config-reference.md](14-config-reference.md) | 配置参数含义、调参、调试命令 | 配置维护/联调 |
| [15-vntpa-alignment.md](15-vntpa-alignment.md) | VNT-PA 对齐差异与优化清单 | VNTPA 开发 |
| [11-vntpa-training.md](11-vntpa-training.md) | VNTPA 离线训练/导出 | 模型训练 |

## 当前状态

- 上次已验证：**69/69 CTest**，ASan 通过，install 通过。
- CMake 当前注册 **69** 个测试（删除 `time_utils_test` 后的基线）。
- autosim 仿真链路、`/navigate_to_pose` Action（cancel/抢占）已接线，待 Linux/autolink 实编验证。
- VNTPA planner 权重未发布；`conf/models/` 已有 DeFM ViT-S/14。

## 文档维护约定

1. 设计契约变更先改 [04-design.md](04-design.md)，再改实现与 [08-acceptance.md](08-acceptance.md)。
2. 新增 topic/字段必须同时更新 [03-data-flow.md](03-data-flow.md) 的 topic 表。
3. 新增模型/权重必须更新 [06-models.md](06-models.md) 的 sha256 与 I/O。
4. 文档内路径使用相对路径；跨目录引用以 `../` 开头。
