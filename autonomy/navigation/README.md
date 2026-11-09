# Navigation

C++20 端到端导航子模块：Component（autolink 薄入口）+ Runner（业务核心）。
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
| [15-vntpa-alignment.md](docs/15-vntpa-alignment.md) | VNT-PA 对齐与优化清单 |
| [16-linux-validation.md](docs/16-linux-validation.md) | SpaceHero 构建、模型与组件验收证据 |
| [17-frontiernet.md](docs/17-frontiernet.md) | FrontierNet 式探索与 DeFM 编码器替换 |
| [18-autosim-validation.md](docs/18-autosim-validation.md) | 真实 Habitat、多进程 Action、停车与故障恢复证据 |
| [19-behavior-recovery-validation.md](docs/19-behavior-recovery-validation.md) | 默认 XML、真实自主脱困、取消/重建与耗尽验收 |

## Python

Python 工程名与包名均为 **`vntpa`**：

```text
python/
├── pyproject.toml
├── vntpa/
├── scripts/
└── tests/
```

包含 vendored VNT-PA / DeFM、DeFM 预处理/编码、VNT-PA ONNX 推理、
训练/评估入口、ONNX 导出和 autosim/autolink 桥，支持完全离线单仓库训练。
详见 [python/README.md](python/README.md)。

## 快速开始

```sh
cd /workspace/autonomy
colcon build --packages-up-to autonomy --cmake-args -DNAVIGATION_BUILD_TESTS=ON
ctest --test-dir build/autonomy/autonomy/navigation --output-on-failure --no-tests=error
```

Linux autolink/component：

```sh
tools/build_linux.sh /workspace/autonomy 8
```

## 外部接口

- Action：`/navigate_to_pose`（`automsgs.actions.NavigateToPoseAction`），feedback/result/cancel/抢占已接。
- Topic：`/navigation/goal`（`GoalSpec`）用于脚本/调试。
- 输出：`/navigation/cmd_vel_raw` → safety → `/cmd_vel`（`TwistStamped`）。

## 当前状态

- 五个 Linux 组件、导航测试与 DeFM C++ ORT 后端已实际构建。
- 2026-11-09 SpaceHero：完整 colcon 构建与 76/76 导航 CTest 通过，最新安装与仿真证据见 [19](docs/19-behavior-recovery-validation.md)。
- 当前验证结果见 [16-linux-validation.md](docs/16-linux-validation.md)；本轮未运行 ASan/TSan。
- 真实 Habitat 九项场景检查与独立进程 kill/重启通过，见 [19](docs/19-behavior-recovery-validation.md)；基础验收历史见 [18](docs/18-autosim-validation.md)。
- 默认行为树已接入保留障碍的自主脱困，见 [19](docs/19-behavior-recovery-validation.md)；长期探索仍待推进。
- VNTPA planner 权重尚缺；DeFM CPU C++/Python ORT 数值验证已通过。

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
| `python/` | `vntpa` Python 训练/部署/autosim 桥 |
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
- 输出 header 使用 depth 时间；Safety 按 depth stamp 进展和每次 raw 指令接收记录 steady 租约，200 ms 断流进入紧急零速；实际停车包含执行器减速。心跳使用 steady clock。
- 配置切换策略/底盘/场景不改代码。
