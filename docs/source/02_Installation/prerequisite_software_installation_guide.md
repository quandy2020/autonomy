# 安装指南

本指南将帮助您安装和配置 Autonomy 项目所需的所有先决条件软件。

---

## 1 系统要求

### 1.1 支持的操作系统

- **Linux**: Ubuntu 20.04/22.04 LTS (推荐)
- **架构**: x86-64 或 ARM64 (aarch64)

### 1.2 硬件要求

- **内存**: 至少 8GB RAM (推荐 16GB)
- **存储**: 至少 20GB 可用磁盘空间
- **CPU**: 多核处理器 (推荐 4 核或以上)

---

## 2 Docker 安装与配置

Docker 是推荐的运行环境，提供了跨平台支持和一致的开发环境。

### 2.1. 安装 Docker

如果您的系统尚未安装 Docker，请运行以下命令：

```bash
cd autonomy/docker/scripts
sudo ./install_docker.sh
```

安装完成后，验证 Docker 是否正常工作：

```bash
docker --version
```

### 2.2 构建 Docker 镜像

`run_autonomy.sh` 脚本会根据您的计算机平台（x86-64 或 aarch64）自动选择并构建相应的 Docker 镜像。

**重要提示**: 在运行脚本之前，请设置环境变量 `AUTONOMY_ENV` 指向宿主机上的 autonomy 项目路径（如果需要在宿主机和容器之间共享代码）。

```bash
# 设置环境变量（可选，仅在需要时设置）
# export AUTONOMY_ENV=/path/to/autonomy

# 构建并运行 Docker 容器
cd autonomy/docker
./run_autonomy.sh
```

脚本会自动：
- 检测系统架构（x86-64 或 aarch64）
- 构建相应的 Docker 镜像
- 启动名为 `SpaceHero` 的容器
- 挂载工作目录到容器中

### 2.3 进入 Docker 容器

容器启动后，您可以通过以下命令进入容器：

```bash
docker exec -it SpaceHero /bin/bash
```

---

## 3 获取源码

### 3.1克隆仓库

从 Gitee 或 GitHub 克隆 Autonomy 项目的源码：

```bash
# 从 Gitee 克隆（国内推荐）
git clone https://gitee.com/openbotbeginner/autonomy.git
git clone https://gitee.com/openbotbeginner/autonomy_ros.git

# 或从 GitHub 克隆
git clone https://github.com/quandy2020/autonomy.git
git clone https://github.com/quandy2020/autonomy_ros.git
```

### 3.2 目录结构

克隆完成后，您的目录结构应该如下所示：

```
autonomy/
├── src/
│   ├── autonomy/          # 核心 Autonomy 框架
│———└── autonomy_ros/      # ROS/ROS2 兼容层
```

---

## 4 编译与安装

Autonomy 项目支持两种编译方式：使用 ROS2 (推荐) 或仅使用 C++ (无 ROS 依赖)。

### 4.1 使用 ROS2 编译（推荐）

此方式适合需要在 ROS2 环境中运行或与 ROS2 集成的场景。

1. **进入 Docker 容器**

```bash
docker exec -it SpaceHero /bin/bash
```

2. **设置 ROS2 环境**

```bash
source /opt/ros/humble/setup.bash
```

3. **编译项目**

```bash
cd /workspace/autonomy
colcon build --symlink-install
```

**编译选项说明**:
- `--symlink-install`: 使用符号链接安装，便于开发时修改代码
- 如需仅编译特定包，使用: `colcon build --packages-select <package_name>`

4. **运行测试（可选）**

```bash
colcon test
colcon test-result --verbose
```

### 4.2 C++ 编译（无 ROS 依赖）

此方式适合仅使用 Autonomy 核心功能，不依赖 ROS/ROS2 的场景。

1. **进入 Docker 容器**

```bash
docker exec -it SpaceHero /bin/bash
```

2. **创建构建目录并编译**

```bash
cd /workspace/autonomy/src/autonomy
mkdir -p build
cd build
cmake .. -G Ninja
ninja
```

3. **运行测试（可选）**

```bash
CTEST_OUTPUT_ON_FAILURE=1 ninja test
```

**编译选项说明**:

- `-G Ninja`: 使用 Ninja 构建系统，编译速度更快
- 如需使用 Makefile，可省略 `-G Ninja` 选项

---

## 5 Habitat-Sim 安装（可选）

Habitat-Sim 是 Facebook Research 开发的 3D 仿真平台，用于训练和评估具身智能体。如果项目需要使用 Habitat-Sim 功能，请按照以下步骤从源码编译安装。

### 5.1 前置要求

- **Python 版本**: >= 3.9
- **编译时间**: 约 30-60 分钟（取决于系统性能）
- **存储空间**: 约 2-3 GB（编译后）

### 5.2 安装步骤

1. **获取源码**

```bash
git clone --branch stable https://github.com/facebookresearch/habitat-sim.git
cd habitat-sim
```

2. **安装 Python 依赖**

```bash
pip3 install -r requirements.txt
```

3. **配置 Git 安全设置（Docker 环境）**

```bash
git config --global --add safe.directory '*'
```

4. **编译安装（Headless 模式，适用于 Docker/服务器环境）**

```bash
python3 setup.py install --headless --no-update-submodules
```

**编译参数说明**：
- `--headless`: 用于无显示器系统（Docker 容器、服务器等）
- `--no-update-submodules`: 跳过 Git 子模块更新，避免权限问题

**其他编译选项**：
- `--with-cuda`: 启用 CUDA 支持
- `--bullet`: 启用 Bullet 物理引擎
- `--audio`: 启用音频传感器（仅 Linux）

示例（组合使用）：
```bash
python3 setup.py install --headless --with-cuda --bullet --no-update-submodules
```

### 5.3 验证安装

```bash
# 检查 Python 模块
python3 -c "import habitat_sim; print('habitat-sim installed successfully')"

# 检查安装版本
pip3 list | grep habitat-sim
```

### 5.4 常见问题

**问题 1: Git 子模块权限错误**

如果遇到 `fatal: detected dubious ownership in repository` 错误，运行：
```bash
git config --global --add safe.directory '*'
```

**问题 2: 编译时间较长**

Habitat-Sim 编译需要较长时间，这是正常现象。如果内存不足，可以限制并行编译：
```bash
python3 setup.py build_ext --parallel 1 install --headless --no-update-submodules
```

### 5.5 参考资源

- **GitHub 仓库**: https://github.com/facebookresearch/habitat-sim
- **官方编译文档**: habitat-sim/BUILD_FROM_SOURCE.md
- **稳定版本列表**: https://github.com/facebookresearch/habitat-sim/releases

---

## 6 环境配置

### 6.1 Docker 容器内环境变量配置

在 Docker 容器中，建议将以下环境变量添加到 `~/.bashrc` 文件中：

```bash
# 编辑 .bashrc
vim ~/.bashrc

# 添加以下内容
### Autonomy Environment Variables ###
export GLOG_logtostderr=1           # 日志输出到标准错误
export GLOG_alsologtostderr=0       # 不同时写入日志文件
export GLOG_colorlogtostderr=1      # 启用彩色日志输出
export GLOG_minloglevel=0           # 最小日志级别（0=INFO, 1=WARNING, 2=ERROR, 3=FATAL）
export GLOG_log_dir=${HOME}/.autonomy/log  # 日志文件目录

# 加载 Autonomy 环境
source /usr/local/setup.bash

# 如果使用 ROS2 编译，还需要加载 ROS2 环境
source /opt/ros/humble/setup.bash
```

使配置生效：

```bash
source ~/.bashrc
```

### 5.2 环境变量说明

| 变量名 | 说明 | 推荐值 |
|--------|------|--------|
| `GLOG_logtostderr` | 是否将日志输出到标准错误 | `1` |
| `GLOG_alsologtostderr` | 是否同时写入日志文件 | `0` |
| `GLOG_colorlogtostderr` | 是否启用彩色日志 | `1` |
| `GLOG_minloglevel` | 最小日志级别 | `0` (INFO) |
| `GLOG_log_dir` | 日志文件保存目录 | `${HOME}/.autonomy/log` |

---

## 7 验证安装

### 7.1 检查编译产物

编译完成后，检查是否成功生成了可执行文件和库文件：

```bash
# ROS2 编译方式
ls -la /workspace/autonomy/install/autonomy/bin/

# C++ 编译方式
ls -la /workspace/autonomy/src/autonomy/build/bin/
```

### 7.2 运行测试示例

参考项目中的测试示例验证安装是否正确：

![运行示例](./images/run.png)
