# 0. 运行指南

## 背景

编过之后仍起不来栈、Docker 里缺数据、两个 launch 互相抢职责——这类问题不在 CMake，而在 **多进程编排、环境变量和 launch 选择**。装依赖与编译仍看 [02 Installation](../02_Installation/00_guide.md)。

## 思路

两件事先钉住：

1. Autonomy 是 **多进程**：planning / control / task / localization… 分开跑，靠 Autolink 通信。  
2. 日常入口是 **`autolink_launch`**（或 `autolink launch start …`），不是单个 `ros2 launch` 习惯能直接套用。

**不要同时开** `autonomy.launch` 与 `task.launch`，会抢同一套导航职责。

帧与插件 id（`global_frame` / `robot_base_frame`、`planner_id` / `controller_id`）写在各域 conf，改一处要全局搜。离线数据集与 `AUTONOMY_DATASETS` 见 [04 环境变量](04_environment.md)、[Tools · 数据集](../18_Tools/08_datasets.md)。


## 步骤

### 1. 各篇怎么用

| 文档 | 解决什么 |
|------|----------|
| [01 概览](01_overview.md) | 进程角色、典型拓扑 |
| [02 快速运行](02_quickstart.md) | 最短启动命令 |
| [03 多进程](03_autonomy_process.md) | 谁依赖谁、如何只起子集 |
| [04 环境变量](04_environment.md) | `AUTONOMY_*`、数据集挂载 |
| [05 Docker 运行时](05_docker_runtime.md) | `run_autonomy.py`、端口与挂载 |
| [06 ROS 2](06_ros2_integration.md) | 可选桥接 |
| [07 验证](07_verification.md) | 起来后怎么确认健康 |
| [08 排错](08_troubleshooting.md) | 运行期对照表 |
| [09 板端资源](09_board_task_launch_benchmark.md) | task.launch 资源占用参考 |

**本机全栈** → [02](02_quickstart.md)。  
**只起 task** → [03](03_autonomy_process.md) + `task.launch`。  
**容器 + Argus/数据集** → [05](05_docker_runtime.md) + [08 数据集](../18_Tools/08_datasets.md)。  
镜像构建在 Installation；本章只管运行参数。

### 2. 动手前记住

1. **先 `source scripts/setup_environment.bash`** — 找不到 `.so`、launch 找不到 dag，九成是环境没进。  
2. **Docker 只解决「怎么跑容器」** — 构建见 [Installation · Docker](../02_Installation/05_docker.md)。  
3. **ROS 工具看不到话题** — 是否走了 [06](06_ros2_integration.md) 桥，而不是假定原生 ROS。

### 3. 最短成功路径

```bash
source scripts/setup_environment.bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
autolink_launch autonomy.launch
```

验证见 [07 验证](07_verification.md)。勿与 `task.launch` 并行。

### 4. 常见坑

| 现象 | 先查 |
|------|------|
| launch 报缺库 | `setup_environment.bash`；`AUTONOMY_BUILD_DIR` |
| 两个 launch 互相抢 | 是否并行开了 autonomy / task |
| 有 Path 无 `cmd_vel` | control 进程；controller 插件名 |
| Docker 里读不到数据 | `AUTONOMY_DATASETS` 与挂载模式 |
| ROS 工具看不到话题 | [06](06_ros2_integration.md) 桥是否启用 |
