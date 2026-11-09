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
| [16-linux-validation.md](16-linux-validation.md) | SpaceHero 构建、模型与组件验收证据 | Linux 验收 |
| [17-frontiernet.md](17-frontiernet.md) | FrontierNet 式探索与 DeFM 编码器替换 | 探索/模型开发 |
| [18-autosim-validation.md](18-autosim-validation.md) | 真实 Habitat、多进程 Action、停车与故障恢复证据 | 仿真验收 |
| [19-behavior-recovery-validation.md](19-behavior-recovery-validation.md) | 默认 XML、真实自主脱困、取消/重建与耗尽验收 | 行为/仿真验收 |
| [11-vntpa-training.md](11-vntpa-training.md) | VNTPA 离线训练/导出 | 模型训练 |

## 当前状态

- Linux 五组件、导航测试与 DeFM C++ ORT 后端已实际构建，具体结果见 [16-linux-validation.md](16-linux-validation.md)。
- 真实 Habitat 基础场景与独立进程 kill/重启已通过，见 [18-autosim-validation.md](18-autosim-validation.md)；默认 BT 脱困见 [19](19-behavior-recovery-validation.md)，长期探索仍待验收，本轮未跑 ASan/TSan。
- VNTPA planner 权重尚缺；DeFM 已验证 hash、IO 和 CPU C++/Python ORT 数值一致性。

## 文档维护约定

1. 设计契约变更先改 [04-design.md](04-design.md)，再改实现与 [08-acceptance.md](08-acceptance.md)。
2. 新增 topic/字段必须同时更新 [03-data-flow.md](03-data-flow.md) 的 topic 表。
3. 新增模型/权重必须更新 [06-models.md](06-models.md) 的 sha256 与 I/O。
4. 文档内路径使用相对路径；跨目录引用以 `../` 开头。
