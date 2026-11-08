# Navigation

C++17 端到端导航子模块：Component（autolink 薄入口）+ Runner（业务核心）。
无显式 costmap；VNTPA 用位姿索引关键帧作为环境表示；安全链与 guardian 独立进程。

## 文档

完整文档入口：[docs/00-index.md](docs/00-index.md)

| 文档 | 内容 |
| --- | --- |
| [01-requirements.md](docs/01-requirements.md) | 设计目的、功能/非功能需求、追溯与优先级 |
| [02-architecture.md](docs/02-architecture.md) | 进程/组件/分层、线程模型、插件扩展点 |
| [03-data-flow.md](docs/03-data-flow.md) | autosim 传感器 → Observation → Runner → 安全链 → `/cmd_vel` |
| [04-design.md](docs/04-design.md) | v2 设计基线、proto、DAG、验收契约 |
| [05-simulation.md](docs/05-simulation.md) | autosim 仿真接入、目标/探索/恢复 |
| [06-models.md](docs/06-models.md) | DeFM 制品、ONNX I/O、VNTPA 模型状态 |
| [07-build.md](docs/07-build.md) | 构建、测试、安装、Sanitizer、benchmark |
| [08-acceptance.md](docs/08-acceptance.md) | 验收矩阵与测试映射 |
| [09-style.md](docs/09-style.md) | Doxygen/命名/注释规范 |
| [10-tools.md](docs/10-tools.md) | 配置校验、benchmark、目标发送 |
| [11-vntpa-training.md](docs/11-vntpa-training.md) | VNTPA 离线训练/导出 |
| [12-configuration.md](docs/12-configuration.md) | 配置分层、模块化约定 |
| [13-todo.md](docs/13-todo.md) | TODO、闭环分级与验收顺序 |
| [14-config-reference.md](docs/14-config-reference.md) | 配置参数含义、调参、调试 |

## 快速开始

```sh
cmake -S /path/to/autonomy -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DAUTONOMY_BUILD_NAVIGATION=ON \
  -DBUILD_TEST=ON

cmake --build build --target navigation
ctest --test-dir build --output-on-failure
```

Linux autolink/component：

```sh
tools/build_linux.sh /path/to/automsgs/build build-linux
```

## 外部接口

- Action：`/navigate_to_pose`（`automsgs.actions.NavigateToPoseAction`），feedback/result/cancel/抢占已接。
- Topic：`/navigation/goal`（`GoalSpec`）用于脚本/调试。
- 输出：`/navigation/cmd_vel_raw` → safety → `/cmd_vel`（`TwistStamped`）。

## 当前状态

- 上次已验证：**69/69 CTest**，ASan 通过，install 通过。
- CMake 当前注册 **69** 个测试。
- autosim 仿真链路已接线，待 Linux/autolink 实编验证。
- VNTPA planner 权重未发布；`conf/models/` 已有 DeFM ViT-S/14（pth + ONNX）。

## 目录

| 目录 | 职责 |
| --- | --- |
| `common/` | 值类型、状态、配置、时间、平台模型 |
| `perception/` | 深度预处理、DeFM 编码、Observation 融合 |
| `memory/` | 体素、关键帧、位姿索引、快照、语义图 |
| `planning/` | 策略/反射接口、VNTPA、场景、任务、规则、Blender |
| `safety/` | 三级安全链（独立进程） |
| `behavior/` | BT 节点、任务原语、意图转换 |
| `guardian/` | 心跳、故障分类、降级、诊断 |
| `proto/` | observation/navigation/lifecycle/config/plugins |
| `conf/` | 三份主配置 + 4 个平台 preset + models |
| `dag/` | navigation/safety/guardian 拓扑 |
| `launch/` | navigation/simulation/replay |
| `tools/` | 配置校验、benchmark、目标发送、VNTPA 训练 |
| `docs/` | 架构、数据流、设计、仿真、模型、构建、验收 |

## 复用

- `autonomy/common/math` / `helper_functions`（Clamp、wrap_angle、abs_eq）
- `autonomy/common/file.hpp::JoinPaths`
- `automsgs/msgs/time_utils.hpp`
- `automsgs` `Pose2D`/`Twist2D`/`OccupancyGrid`/`PointCloud`
- `autonomy/common/network/Engine`（模型推理适配器）

## 运行约定

- navigation 只写 `/navigation/cmd_vel_raw`；safety 是唯一 `/cmd_vel` 出口。
- `/cmd_vel` 为 `TwistStamped`，与 autosim 一致。
- 所有时间戳为会话纳秒；Safety 使用 depth stamp 做新鲜度。
- 配置切换策略/底盘/场景不改代码。
