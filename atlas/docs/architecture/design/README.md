# Voxel-SLAM 目标架构与模块设计索引

本目录承载目标设计的细化内容。先读本索引，再按[系统模块顺序](../current-architecture-and-dataflow.md)和根目录[文档导航](../../README.md)阅读；这里的文件树、字段、类名均为目标设计，除非对应章节明确标为当前实现。

主目录树、原始 Voxel-SLAM 能力映射和依赖顺序见[项目目录树](project-tree.md)。目录以 Voxel-SLAM 算法链为基准，Cartographer 风格仅用于文件命名、领域边界和 `internal/` 实现隔离。

## 命名风格

采用 Cartographer 源码中的工程命名习惯，而不仅是 `snake_case`：稳定领域对象使用短名，如 `trajectory_node.hpp`、`pose_graph.hpp`；确有多实现需求时才引入接口契约；内部优化/约束实现归入 `internal/optimization/`、`internal/constraints/`。文件名由目录上下文补足语义，不堆叠工程名、传感器名和模块名，也不随意加 `manager`、`helper`、`engine`。

目标代码按现有算法职责命名，例如 `trajectory_builder.hpp`、`local_trajectory_builder.hpp`、`constraint_builder.hpp`、`optimization_problem.hpp`。文件名和类型名不包含 `SLAM` 字样或项目全名；仅在确有同一领域的多个实现需要区分时添加限定词。

变量、函数和类型继续遵守 Google C++ 风格。新增目录名、文件名和类型名优先写全单词；公认算法缩写仅在必要上下文中保留。表中名称都是**目标设计名**，不代表当前实现已经存在。

下文代码路径以 Atlas 工程根目录为基准：`app/...`、`config/...` 是工程直接子目录；`atlas/...` 是 C++ 源码树 `atlas/atlas/...` 的相对部分。

## 阅读顺序

1. [项目目录树与迁移映射](project-tree.md)：先统一模块边界和代码依赖顺序。
   当前源码的 include 方向和版本写入点见[模块依赖审计](dependency-audit.md)。
2. [应用与流水线编排](application-and-pipeline.md)：确定系统入口与装配边界。
3. [Trajectory、Node 与 Submap 模型](trajectory-and-map.md)：确定核心数据对象和所有权。
4. [前端初始化、里程计和局部建图](frontend-and-local-mapping.md)。
5. [地图模块总览](mapping-and-data.md)，再看[体素/HBA](mapping.md)、[持久化](mapping/serialization.md)、[可视化](mapping/visualization.md)。
6. [优化公共抽象与求解器适配](optimization.md)。
7. [Backend 总览](backend-and-global-mapping.md)，再看[BTC 回环](backend/loop-closure.md)、[位姿图](backend/pose-graph.md)、[多会话与全局建图](backend/global-mapping.md)。
8. [运行时、持久化与测试](runtime-and-storage.md)、[YAML 配置契约](configuration.md)、[第三方依赖边界](third-party-dependencies.md)。
9. 最后核对[文件与类的一对一设计](file-class-design.md)，其中的类名和文件名不得超出前述模块契约。

## 拆分准则

- 有独立状态所有权、算法边界、替换需求或线程/资源生命周期时，才拆成独立类和文件。
- 同一领域下的小型 ID、枚举和 options 可以放在一个短领域头文件中；不要为每个 struct 生成机械的一对 `.hpp/.cpp`。
- 文件名先短而明确；目录已说明上下文时，不重复项目全名、传感器名或模块名。
- 先保留已有调用和配置兼容，再分批重命名；不要把算法变化和纯文件布局变化混在一起。
