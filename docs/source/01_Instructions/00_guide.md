# 0. 入门指南

## 背景

新人进仓库最常卡在三件事：Autonomy 到底覆盖哪些能力、本机能不能先编过跑起来、代码和文档该从哪下钻。装依赖、编模块的细步骤在 [02 Installation](../02_Installation/00_guide.md)；本章先把预期说清，避免后面按 ROS 习惯或「单进程大库」去猜。

## 思路

把它当成 **多进程整机栈**，而不是一个 `libautonomy.so`：

- 通信走 **Autolink**，消息在 **automsgs**，可不依赖 ROS 2。  
- 导航语义对齐 Navigation2（planner / controller / costmap / BT），进程模型和配置是自己的。  
- 业务域可裁剪（`AUTONOMY_BUILD_*` / `BUILD_*`）；排错先确认「哪个进程在、话题是否一致」。

只想验证「能编、能起」→ [快速上手](02_quickstart.md)。要改算法或裁剪上车 → [架构](03_system_architecture.md) + [仓库](04_repository.md)，再进对应域。


## 步骤

### 1. 按目标打开对应页

| 文档 | 解决什么问题 |
|------|----------------|
| [01 概览](01_overview.md) | 版本、分层、域怎么切 |
| [02 快速上手](02_quickstart.md) | 克隆 → 依赖 → 编译 → `setup` → launch |
| [03 架构与能力](03_system_architecture.md) | 分层职责、导航流水线、和 Nav2 对照 |
| [04 仓库与生态](04_repository.md) | 目录、产物路径、CMake 开关 |

读完后二选一：[Installation](../02_Installation/00_guide.md) 装全环境，或 [Running](../04_Running/00_guide.md) 搞清多进程。

### 2. 先搞清这几条约束

1. **源码根**：带顶层 `CMakeLists.txt` 的目录；板端 `build` 放本地盘，勿 NFS。  
2. **多进程**：规划 / 控制 / 任务 / 定位分进程，靠 Autolink。  
3. **域可裁剪**：上车不必全开；见 [04 · 开关](04_repository.md#45-编译开关)、[Installation · 编译](../02_Installation/06_build.md)。  
4. **帧与 id 一致**：`global_frame` / `robot_base_frame`、`planner_id` / `controller_id` 跨进程统一；链路 Goal → `task` → `planning` → `control`，TF `map → odom → base_link`。  
5. **文档边界**：业务域在 `docs/source/05`–`17`；`autoviz` / `autodriver` / `automanip` 看各自包文档，入口 [Tools · 其它包](../18_Tools/09_ecosystem.md)。

### 3. 按场景最短路径

**本机编过并启动**  
[02](02_quickstart.md) → [Installation 快速安装](../02_Installation/02_quickstart.md) → `source scripts/setup_environment.bash` → `autolink_launch autonomy.launch`。  
勿同时开 `autonomy.launch` 与 `task.launch`。

**改规划 / 控制 / BT**  
[03](03_system_architecture.md) Nav2 对照 → [08](../08_Planning/00_guide.md) / [09](../09_Control/00_guide.md) / [16](../16_Navigator/00_guide.md)。

**云端 / 手机发令** → [15 Bridge](../15_Bridge/00_guide.md)，先 `GetCapabilities`。  
**板端** → [Installation · 嵌入式](../02_Installation/09_embedded_board.md)。  
**只写通信节点** → [03 Communication](../03_Communication/00_guide.md)。

### 4. 常见坑

| 现象 | 先查 |
|------|------|
| 找不到 `.so` / 跑不起来 | 是否 `source scripts/setup_environment.bash`；`AUTONOMY_BUILD_DIR` |
| launch 立刻冲突 | 是否重复开了 `autonomy.launch` 与 `task.launch` |
| 有路径车不动 | control 进程与 `controller_id`；costmap / TF |
| 改一域链不上 | 依赖域是否编进；`AUTONOMY_BUILD_*` |
| 路径对不上 | 是否站在源码根；业务在 `autonomy/autonomy/...` |

排错细表：[Installation §8](../02_Installation/08_troubleshooting.md)、[Running §8](../04_Running/08_troubleshooting.md)、[FAQ](../19_FAQs/00_guide.md)。

第一次接触顺序：本页 → [01](01_overview.md) → [02](02_quickstart.md) + Installation → [03](03_system_architecture.md) → [04](04_repository.md)。之后按任务进 03–20，不必通读编号。
