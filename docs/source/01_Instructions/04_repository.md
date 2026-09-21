# 4. 仓库与生态

本章说明**代码落在哪、编完产出什么、配置与 launch 怎么找、如何裁剪与对接外部工具**。逻辑分层见 [§3](03_system_architecture.md)；安装编译细节见 [02 Installation](../02_Installation/00_guide.md)。

**源码根**：含顶层 `CMakeLists.txt` 的目录（独立克隆即为仓库根；monorepo 中多为 `…/src/autonomy`）。下文路径均相对源码根。当前版本见 `version.json`（**0.2.0**）。

仓库是**超工程**：平台层（通信/消息）几乎始终构建；业务能力在 `autonomy/<域>/`，用 `AUTONOMY_BUILD_*` 裁剪；驱动与可视化等产品子工程用 `BUILD_*` 控制。

## 4.1 顶层怎么分区

| 分区 | 目录 | 作用 |
|------|------|------|
| 平台 | `autolink/`、`automsgs/` | 进程间通信与消息/RPC 定义，导航与其它域共用 |
| 业务域 | `autonomy/*` | SLAM、感知、地图/规划/控制、机械臂、任务 BT、管理面、Bridge… |
| 硬件接入 | `autodriver/` | 相机/雷达/IMU、底盘等 HAL（`BUILD_AUTODRIVER`） |
| 产品工具 | `autoviz/`、`autosim/`、`autonomy/orbisview/` | 3D 可视化、仿真、Web HMI |
| 工程基建 | `cmake/`、`docker/`、`scripts/`、`docs/`、`tools/` | 依赖查找、容器、环境脚本、文档、格式化 |
| 资产 | `config/`、`data/`、`images/` | 部分模块资产、数据、配图 |
| 部署 | `ansible/` | 装机 / 救援类自动化（可选） |

## 4.2 顶层目录树

```text
autonomy/                         # ← 源码根
├── CMakeLists.txt                # 超工程入口：聚合 autolink / automsgs / autonomy / 子工程
├── CMakePresets.json             # 预设：autonomy-minimal、jdr-board 等
├── version.json                  # 版本号 0.2.0
├── package.xml / README.md / LICENSE / CHANGELOG.rst
│
├── autolink/                     # 通信运行时（Node / Channel / Service / Action）
├── automsgs/                     # 消息与 RPC schema（结构对齐 ROS msgs）
│
├── autonomy/                     # 业务域源码（每个子目录可单独 AUTONOMY_BUILD_*）
│   ├── common/                   # 公共工具、数学、日志辅助
│   ├── transform/                # TF 缓冲与变换
│   ├── map/                      # 地图 / Costmap2D
│   ├── planning/                 # 全局规划 → autonomy.planning
│   ├── control/                  # 局部控制 → autonomy.control
│   ├── task/                     # 多域 BT 编排 → autonomy.task（文档章名 Navigator）
│   ├── localization/             # SLAM / 定位（如 Atlas）
│   ├── perception/               # 感知（检测/跟踪等）
│   ├── prediction/               # 预测
│   ├── manipulation/             # 机械臂 / 操作
│   ├── sensor/                   # 传感器同步
│   ├── vehicle/                  # 车体模型
│   ├── audio/                    # 语音等
│   ├── system/                   # 管理面：monitor / 安全 / OTA / 默认 launch
│   ├── bridge/                   # gRPC 对外 API → autonomy.bridge
│   ├── visualization/            # 可视化相关组件
│   ├── orbisview/                # Web HMI（BUILD_ORBISVIEW）
│   └── tools/                    # 域内工具
│
├── autodriver/                   # 传感器与底盘 HAL（独立 CMake 子工程）
│   ├── chassis/  config/  launch/  …
│   └── main.cpp                  # 驱动主进程入口之一
├── autoviz/                      # 原生 3D 可视化（BUILD_AUTOVIZ）
├── autosim/                      # 仿真（BUILD_AUTOSIM）
│
├── config/                       # 仓库级资产（如 perception、localization）
├── data/                         # 数据资产（按需）
├── cmake/                        # Find*.cmake、域选项 autonomy_options.cmake 等
├── docker/                       # Dockerfile、run_autonomy.py、install/*.sh
├── docs/                         # Sphinx 本手册（source/01_Instructions …）
├── scripts/                      # setup_environment.bash、install_dependencies.py、NFS
├── tools/                        # clang_format_sources.py、打包等
├── ansible/                      # 装机剧本（可选）
└── images/                       # README 配图
```

**怎么找代码（经验规则）**：

| 你要改… | 先看 |
|---------|------|
| 通信 API / launch 机制 | `autolink/`、[03 Communication](../03_Communication/00_guide.md) |
| 消息字段 / RPC | `automsgs/` |
| 导航规划/控制/BT | `autonomy/planning|control|task/` |
| SLAM / 感知 | `autonomy/localization|perception/` |
| 机上监控 / OTA / 默认全栈 launch | `autonomy/system/` |
| 远程发令 | `autonomy/bridge/` |
| 相机雷达底盘 | `autodriver/` |
| 依赖安装脚本 | `scripts/`、`docker/install/` |

## 4.3 构建产物

默认在源码根下创建 `build/`（**板端 build 必须在本地盘**，勿放 NFS，见 [Installation §9](../02_Installation/09_embedded_board.md)）。工程已模块化：**没有**单一的 `libautonomy.so` 总库。

| 类型 | 典型路径 | 说明 |
|------|----------|------|
| 域动态库 | `build/lib/libautonomy_*.so` | 如 `libautonomy_common.so`、`libautonomy_planning.so` |
| 通信 / 消息库 | `build/lib/libautolink.so`、`libautomsgs*.so` | 平台层 |
| 可执行文件 | `build/bin/` | `autolink`、`autonomy.planning`、`autonomy.control`、`autonomy.task`、`autonomy.bridge`、`autonomy.monitor`… |
| 插件 | `build/lib/` | `*_planner*.so`、`autonomy_behavior_tree_*.so`、感知 `*_component.so` 等 |

开发期加载环境（把 `build/bin`、`build/lib` 放进 `PATH` / `LD_LIBRARY_PATH`）：

```bash
cd /path/to/源码根
source scripts/setup_environment.bash
which autolink
```

若已 `cmake --install` 到 `/usr/local`：

```bash
sudo cmake --install build --prefix /usr/local
source /usr/local/share/autonomy/setup.bash
# 注意：不要让旧的 build/bin 盖住已安装的 autolink
```

build 不在默认路径时：先 `export AUTONOMY_BUILD_DIR=/你的/build`，再 `source scripts/setup_environment.bash`。

## 4.4 配置与 launch

运行时配置分散在「系统 conf / 任务 BT / 各域 conf / 驱动」几处，不要只盯仓库根 `config/`。

| 用途 | 位置 | 说明 |
|------|------|------|
| 系统 / 监控 / OTA / 日志 | `autonomy/system/conf/*.pb.txt` | 管理面真相源相关 |
| 默认全栈启动 | `autonomy/system/launch/autonomy.launch` | monitor + planning + control + task + bridge… |
| 仅导航三件套 | `autonomy/task/launch/task.launch` | planning + control + task；勿与全栈同时开 |
| 行为树 XML | `autonomy/task/conf/behavior_tree/` | 如 `navigate_to_pose.xml` |
| 任务 / Navigator conf | `autonomy/task/conf/`（含 `navigator.pb.txt`） | 帧名、容差、BT 路径 |
| 各算法域 conf | `autonomy/<域>/conf/` | planning、control、map… |
| 驱动 | `autodriver/config/`、`autodriver/launch/` | 传感器与底盘 |
| 感知等仓库级资产 | `config/perception`、`config/localization` | 与域内 conf 配合使用 |

跨进程必须一致的项：`global_frame` / `robot_base_frame`、默认 `planner_id` / `controller_id`。不一致时常见现象是 TF 失败或选错插件。

启动示例：

```bash
source scripts/setup_environment.bash
autolink_launch autonomy.launch          # 全栈
# 或
autolink launch start task.launch        # 仅导航
autolink launch stop task.launch
```

更多运行说明：[04 Running](../04_Running/00_guide.md)。

## 4.5 编译开关（摘要）

两层开关不要混用：

| 层级 | 前缀 | 例子 | 作用 |
|------|------|------|------|
| 业务域 | `AUTONOMY_BUILD_*` | `AUTONOMY_BUILD_PLANNING` | 是否编译 `autonomy/planning` 等 |
| 产品 / 子工程 | `BUILD_*` | `BUILD_AUTODRIVER`、`BUILD_GRPC` | 驱动、Bridge、可视化等 |

| 常用选项 | 默认（约） | 说明 |
|----------|------------|------|
| `AUTONOMY_BUILD_<域>` | 多数 ON | 关掉后该目录不进构建；依赖图不满足会 FATAL |
| `BUILD_AUTODRIVER` | ON | 传感器 / 底盘 |
| `BUILD_GRPC` | ON | Bridge；关则 bridge 域不可用 |
| `BUILD_AUTOVIZ` / `BUILD_AUTOSIM` / `BUILD_ORBISVIEW` | ON / OFF / ON | 可视 / 仿真 / Web |
| `BUILD_ONNXRUNTIME` | ON | 感知推理（需找到运行时） |
| `BUILD_TEST` / `BUILD_DOCS` | OFF / ON | 单测 / Sphinx |

只要导航、只要感知+驱动等完整 cmake 示例见 [Installation §6.2](../02_Installation/06_build.md)。

## 4.6 生态集成

核心是**独立 C++ 栈**（Autolink 运行时），不强制安装 ROS 2。需要可视化、远程调度或与现有 ROS 工具共存时，再接下列接口。

| 方式 | 组件 | 说明 | 文档 |
|------|------|------|------|
| 多进程入口 | `autolink_launch` / `autolink launch` | 推荐本地联调方式 | [04 Running](../04_Running/00_guide.md) |
| 进程间 IPC | Autolink Channel / Service / Action | 对标 Topic / RPC / Action | [03 Communication](../03_Communication/00_guide.md) |
| 消息类型 | `automsgs` | 字段语义对齐 ROS msgs，便于桥接 | [14 Commsgs](../14_Commsgs/00_guide.md) |
| 远程 API | `autonomy.bridge`（需 `BUILD_GRPC=ON`） | 发令、健康、OTA 等 | [15 Bridge](../15_Bridge/00_guide.md) |
| ROS 2 | 可选包装 / Topic 转发 | 核心编译不依赖 rclcpp | [Running · ROS 2](../04_Running/06_ros2_integration.md) |
| 可视化 | Autoviz、OrbisView、Foxglove、RViz2 | RViz2 一般经 ROS 桥接 | [13 Visualization](../13_Visualization/index.rst) |
| 仿真 | Autosim 等 | | [12 Simulation](../12_Simulation/index.rst) |

## 4.7 日常命令

```bash
# 环境（每个新终端）
source scripts/setup_environment.bash

# 依赖：桌面 full / 板端 board
python3 scripts/install_dependencies.py --skip-installed
python3 scripts/install_dependencies.py --profile board --skip-installed

# 格式化
python3 tools/clang_format_sources.py autonomy/planning
python3 tools/clang_format_sources.py autonomy/task

# 增量编某一目标（已 cmake 配置过）
cmake --build build -j$(nproc) --target autonomy.planning
```

相关：[§1 概览](01_overview.md) · [§2 快速上手](02_quickstart.md) · [§3 架构](03_system_architecture.md) · [02 Installation](../02_Installation/00_guide.md)
