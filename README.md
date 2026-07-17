# autonomy

![openbot framework](./images/openbot_framework.png)

**autonomy** 是一套自主移动机器人软件框架。核心产物为共享库 **`libautonomy.so`**（CMake + C++17），内嵌通信运行时 [autolink](https://github.com/quandy2020/autolink)；可不依赖 ROS/ROS2 独立运行，并通过 `commsgs` 与 ROS 风格消息互操作。

## 功能概览

| 领域 | 能力 |
|------|------|
| **通信** | Autolink（Node / Channel / Service / Action）；可选 gRPC Bridge |
| **定位** | Atlas 视觉 SLAM；Cartographer 激光 |
| **地图** | `costmap_2d` / grid_map；地图服务 |
| **规划** | PlannerServer；NavFn / Dijkstra / Theta\* 等全局规划器 |
| **控制** | ControllerServer；MPPI / Graceful / Pure Pursuit 等控制器 |
| **任务** | TaskServer + BehaviorTree.CPP；导航 / 跟踪 / 探索等应用 BT（节点编入 `libautonomy`） |
| **可视化** | Foxglove Bridge（`autonomy_foxglove_bridge`） |
| **部署** | Docker（x86_64 / aarch64）；Ansible 裸机分发 |

进程入口（`build/bin/`）：`localization` · `planning` · `control` · `task` · `autonomy_bridge`（需 gRPC）· `autonomy_foxglove_bridge` 等。

## 环境要求

- Ubuntu 22.04（推荐，与 Docker 镜像一致）
- GCC 11+ 或 Clang（C++17）
- CMake 3.20+（建议 Ninja）

主要第三方依赖（多由 `docker/install/*.sh` 装到 `/usr/local`）：

- Eigen3、Ceres、glog、gflags、OpenCV、OSQP、yaml-cpp、Lua 5.3、Protobuf、nlohmann_json
- BehaviorTree.CPP 4.x、FBow、g2o、SQLite3、Boost.iostreams、Cairo
- 可选：gRPC、Ipopt、ONNX Runtime、foxglove-sdk、Prometheus

## 快速开始

### 1. 克隆（含 autolink 子模块）

```bash
git clone --recurse-submodules https://github.com/quandy2020/autonomy.git
cd autonomy
```

### 2. 安装依赖

依赖分层：`scripts/install_deps` → `docker/install/*.sh` → `ansible/roles/dependencies/`。详见 [`scripts/README.md`](scripts/README.md)。

```bash
cd scripts
python3 -m install_deps              # APT + 第三方
python3 -m install_deps --apt-only   # 仅 APT
python3 -m install_deps --list-apt
```

### 3. 配置与编译

```bash
mkdir -p build && cd build
cmake -G Ninja ..
ninja
```

### 4. Docker（可选）

```bash
export AUTONOMY_ENV=/path/to/autonomy   # 仓库根目录
python3 docker/run_autonomy.py -p x86_64
```

容器内仓库挂载为 `/workspace/autonomy`：

```bash
cd /workspace/autonomy
mkdir -p build && cd build
cmake -G Ninja ..
ninja
```

更多选项见 [`docs/source/02_Installation/05_docker.md`](docs/source/02_Installation/05_docker.md)。

### 5. Ansible 部署（机器人 / 边缘机）

见 [`ansible/README.md`](ansible/README.md)。

```bash
cd ansible
./deploy.sh build
./deploy.sh deploy robots -e autonomy_artifact_path=../dist/autonomy.tar.gz
```

## CMake 选项

| Option | 默认 | 说明 |
|--------|------|------|
| `BUILD_GRPC` | ON | gRPC Bridge（`autonomy_bridge`） |
| `BUILD_TEST` | ON | 单元测试 |
| `BUILD_DOCS` | ON | Sphinx 文档（若找到 Sphinx） |
| `BUILD_TOOLS` | ON | `autonomy/tools` 源码 |
| `BUILD_PROMETHEUS` | OFF | Prometheus 指标 |
| `BUILD_ONNXRUNTIME` | ON | ONNX 网络封装（若找到运行时） |

## 目录结构

```text
autonomy/
├── autonomy/          # C++ 源码
│   ├── common/        # 公共工具、配置、版本
│   ├── commsgs/       # ROS 兼容消息
│   ├── localization/  # Atlas / Cartographer
│   ├── map/           # costmap / 地图服务
│   ├── planning/      # 全局规划
│   ├── control/       # 局部控制
│   ├── task/          # 任务服务与 BT 应用
│   ├── bridge/        # gRPC 桥
│   ├── visualization/ # Foxglove
│   ├── perception/ · prediction/ · sensor/ · transform/ · system/ · …
├── autolink/          # 通信运行时（git submodule）
├── config/            # Lua / YAML 配置
├── cmake/             # CMake 模块（sources / deps / protobuf / install …）
├── docker/            # 镜像与依赖安装脚本
├── ansible/           # 裸机部署
├── docs/              # Sphinx 文档
├── scripts/           # install_deps 等
└── CMakeLists.txt
```

## 文档

- 在线手册：[autonomy.readthedocs.io](https://autonomy.readthedocs.io/en/latest/index.html)
- 本地构建：`cd build && cmake -DBUILD_DOCS=ON .. && ninja`

## 版本

当前版本见 [`version.json`](version.json)（**0.2.0**）。变更记录：[`CHANGELOG.rst`](CHANGELOG.rst)。

## 许可证

[Apache 2.0](LICENSE)

## 致谢

本项目借鉴并吸收了开源社区的思想与代码，包括：

- [Autoware Universe](https://github.com/autowarefoundation/autoware.universe)
- [Cartographer](https://cartographer-project.org/)
- [Navigation2](https://github.com/ros-navigation/navigation2)
- [ROS 2](https://github.com/ros2)
- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
