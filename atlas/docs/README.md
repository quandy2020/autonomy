# Atlas 文档导航与重构准则

本文档是 `atlas/docs/` 的唯一入口，说明阅读顺序、文档分类和设计约束。目标是让后续代码重构从同一套系统模型出发，并能区分已实现事实、待定方案与论文描述。

## 推荐阅读顺序

按“理解现状 → 确认缺口 → 形成目标 → 拆分模块 → 实施变更”的顺序阅读：

1. [Atlas README](../README.md)：构建方式、程序入口和当前运行方法。
2. [当前源码架构与数据流程](architecture/current-architecture-and-dataflow.md)：原始 Voxel-SLAM 与 Atlas 代码的入口、调用、线程和数据归属。
3. [重构评估与缺口](architecture/refactor-assessment.md)：哪些能力已经落地、部分完成或仍需核实。
4. [重构原则与 Cartographer 借鉴](architecture/refactor-principles.md)：领域对象、依赖方向、所有权、并发和命名的总约束。
5. [架构设计索引](architecture/design/README.md)：按模块深入目录、文件、类和配置设计。
6. [代码重构分阶段路线图](plans/2026-10-24-voxel-slam-code-refactor-roadmap.md)：每阶段列出代码产出、验证门槛、回退点和进入下一阶段的条件。
7. [论文中文翻译与数学解读](reference/voxel-slam-paper-zh.md)：理解算法动机和数学模型；论文内容不能代替源码实现证据。

## 文档分类

```text
docs/
├── README.md                         # 本入口：阅读路径、权威层级和全局约束
├── architecture/                     # 当前架构事实、评估和目标原则
│   ├── current-architecture-and-dataflow.md
│   ├── refactor-assessment.md
│   ├── refactor-principles.md
│   └── design/                        # 模块、目录、类、依赖和配置详细设计
├── reference/                         # 论文、算法及外部背景材料
├── plans/                             # 有序实施计划和阶段检查点
└── ...                                # 后续新增文档必须归入明确类别
```

- `architecture/` 回答系统现在是什么、目标要变成什么、为何这样划分。
- `architecture/design/` 将目标落实到模块职责、文件/类、配置、依赖和线程边界。
- `reference/` 解释论文和算法来源，不声明 Atlas 已具备相应功能。
- `plans/` 把目标拆成有序可审查的代码变更；计划不是长期架构规范。

## 文档权威层级

发生不一致时，按下列规则判断，不把不同证据混为一谈：

1. **当前实现事实**：以代码和构建配置为准；测试只证明它覆盖的行为。架构现状文档用于导航，不能覆盖源码。
2. **目标设计约束**：以用户确认的架构原则和模块设计为准。设计尚未落地时必须标为“目标/待实现”，不能写成当前行为。
3. **算法依据**：论文与原始 Voxel-SLAM 源码分别作为理论动机和移植行为参照。若两者不同，记录差异；不得由论文推断 Atlas 已实现。
4. **实施顺序**：计划用于安排工作，若与架构约束冲突，先修订计划，不通过执行计划悄悄改变设计。

所有文档应显式使用“当前实现”“目标设计”“待确认”状态词。参数、目录、类型名和算法能力凡未在代码中接线，不得描述为可运行。

## 跨文档一致性约束

- **系统链路一致**：输入适配与同步 → 前端初始化/估计/局部建图 → 地图状态 → 回环与全局优化 → 结果提交/持久化。模块顺序用于表达数据流，不改变状态所有权。
- **依赖单向**：`app` 组装模块；传感器值类型不依赖算法；前端不依赖后端实现；后端通过Atlas 因子图 API请求求解；GTSAM、文件格式和消息类型仅留在各自边界。
- **状态唯一归属**：每个可变状态只有一个 owner。跨模块/线程传递明确的数据对象；后台计算消费不可变快照，以 trajectory/map version 校验并提交结果。
- **设计与代码双向追踪**：每项原始算法能力都映射到目标模块；每个新抽象都要有算法、数据流或生命周期依据。禁止为目录对称而增加空 facade。
- **配置与类模型一致**：整个系统只维护一个 YAML，内部按模块分区；每个参数映射到唯一强类型 options 和消费点，不复制场景配置。
- **性能与兼容约束**：C++17；前端热路径有界且不等待全局优化；保持算法行为的重构与算法改动分阶段进行；核心不引入 ROS。
- **目录和名称一致**：代码目录、配置模块、文档模块使用同一领域边界；新名字表达真实职责，避免同义重复和不必要缩写。

## 模块设计阅读顺序

从系统边界向算法核心逐层深入：

1. [项目目录树与迁移映射](architecture/design/project-tree.md)
2. [应用入口与流水线编排](architecture/design/application-and-pipeline.md)
3. [轨迹、节点与地图模型](architecture/design/trajectory-and-map.md)
4. [初始化、里程计与局部建图](architecture/design/frontend-and-local-mapping.md)
5. [地图构建与数据管理](architecture/design/mapping-and-data.md)，再读 [体素地图/HBA](architecture/design/mapping.md)、[持久化](architecture/design/mapping/serialization.md) 和 [可视化](architecture/design/mapping/visualization.md)
6. [优化公共抽象](architecture/design/optimization.md)
7. [回环、位姿图与全局建图](architecture/design/backend-and-global-mapping.md)，再读 [BTC 回环](architecture/design/backend/loop-closure.md)、[位姿图](architecture/design/backend/pose-graph.md) 和 [多会话全局建图](architecture/design/backend/global-mapping.md)
8. [运行时、持久化和验证边界](architecture/design/runtime-and-storage.md)
9. [YAML 配置契约](architecture/design/configuration.md) 与 [第三方依赖边界](architecture/design/third-party-dependencies.md)
10. [文件与类的一对一设计](architecture/design/file-class-design.md)：作为汇总清单，必须以之前的模块契约为依据。

## 新增或修改文档的要求

- 先标明文档类别、读者问题和“当前/目标/待确认”状态；避免在一份文档里混写论文、代码事实与目标方案。
- 修改模块设计时，同步核对总目录树、文件/类清单、配置归属、依赖图、数据流程及实施计划的相关链接。
- 参数、类或目录调整必须说明 owner、输入/输出、调用方、线程上下文和迁移来源；不确定项列为待确认，不用猜测填补。
- 每个新文档都从本入口或相应分类索引链接；移动文档时更新全部相对链接和内文路径。
- 不重复维护多个“权威目录树”或参数清单；若保留摘要，只链接到唯一完整设计。
