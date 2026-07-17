# autonomy

![openbot framework](./images/openbot_framework.png)

**autonomy** 是自主移动机器人软件框架，内嵌 [autolink](https://github.com/quandy2020/autolink)，不依赖 ROS/ROS2。

## 功能概览

| 领域 | 能力 |
|------|------|
| **通信** | Autolink（Node / Channel / Service / Action） |
| **桥接** | 可选 gRPC Bridge（异步） |
| **定位** | Atlas 视觉 SLAM；Cartographer 激光 SLAM |
| **地图** | `costmap_2d` / grid_map |
| **规划** | NavFn / Dijkstra / Theta\* |
| **控制** | MPPI / Graceful / Pure Pursuit |
| **任务** | BT 应用：导航 / 跟踪 / 探索 / 遥操 / 回充 |
| **可视化** | Foxglove Bridge |
| **容器** | x86_64 / aarch64 开发镜像 |
| **部署** | Ansible 裸机分发 |

## 环境要求

- **推荐**：Docker（镜像已含依赖，Ubuntu 22.04）
- **源码编译**：Ubuntu 22.04；GCC 11+ / Clang（C++17）

## 快速开始

### 1. 克隆

```bash
git clone --recurse-submodules https://github.com/quandy2020/autonomy.git
cd autonomy
```

### 2. 编译（二选一）

#### 方式 A：Docker 容器（推荐）

```bash
export AUTONOMY_ENV=/path/to/autonomy
python3 docker/run_autonomy.py -p x86_64
```

进入容器后：

```bash
cd /workspace/autonomy
mkdir -p build && cd build
cmake -G Ninja ..
ninja
```

更多选项见 [Docker 文档](docs/source/02_Installation/05_docker.md)。

#### 方式 B：宿主机源码编译

```bash
cd scripts && python3 -m install_deps
cd ..
mkdir -p build && cd build
cmake -G Ninja ..
ninja
```

### 3. Ansible 部署（可选）

不想在每台机器人上手敲编译时，用 Ansible 远程装软件。两种用法：

1. **本机装一份**：在这台电脑上编译并装好  
2. **多台一起装**：先打成一个包，再推到多台机器人；改配置时也可以只推配置、不重装软件  

更细的说明见 [`ansible/README.md`](ansible/README.md)。

```bash
cd ansible
pip install "ansible>=8,<10"   # 第一次用先装 Ansible

# 在本机编译并安装
./deploy.sh build

# 打成安装包，推到多台机器人；之后若只改配置，用 push 即可
../scripts/package_autonomy_artifact.sh --output ../dist/autonomy.tar.gz
./deploy.sh deploy robots -e autonomy_artifact_path=$PWD/../dist/autonomy.tar.gz
./deploy.sh push robots
```

| 命令 | 描述 |
|------|--------|
| `./deploy.sh build` | 在本机编译安装 |
| `./deploy.sh deploy robots -e …` | 把安装包推到机器人 |
| `./deploy.sh push robots` | 只更新配置并重启服务 |
| `./deploy.sh check robots` | 部署前检查系统是否就绪 |

推多台前：在 `inventory/robots/hosts.yml` 写好机器人 IP，并配好免密 SSH。

## CMake 选项

| Option | 默认 | 说明 |
|--------|------|------|
| `BUILD_GRPC` | ON | gRPC Bridge |
| `BUILD_TEST` | ON | 单元测试 |
| `BUILD_DOCS` | ON | Sphinx 文档 |

## 目录结构

```text
autonomy/
├── autonomy/     # C++ 源码
├── autolink/     # 通信运行时（submodule）
├── config/       # Lua / YAML
├── cmake/        # 构建模块
├── docker/       # 镜像与依赖脚本
├── ansible/      # 裸机部署
├── docs/         # Sphinx
├── scripts/      # install_deps 等
└── CMakeLists.txt
```

## 链接

- 文档：[在线文档](https://autonomy.readthedocs.io/en/latest/index.html)
- 许可证：[Apache 2.0](LICENSE)

## 致谢

- [Autoware Universe](https://github.com/autowarefoundation/autoware.universe)
- [Cartographer](https://cartographer-project.org/)
- [Navigation2](https://github.com/ros-navigation/navigation2)
- [ROS 2](https://github.com/ros2)
- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
