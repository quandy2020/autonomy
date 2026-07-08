# 2. 快速上手

> 完整安装步骤见 [02 Installation](../02_Installation/00_guide.md)；运行与 Docker 见 [04 Running](../04_Running/00_guide.md)。

### 2.1 四步启动

```bash
# 1. 克隆
git clone --recurse-submodules https://github.com/quandy2020/autonomy.git
cd autonomy

# 2. 安装依赖（Ubuntu 22.04 推荐）
python3 -m install_deps

# 3. 编译
mkdir -p build && cd build
cmake -G Ninja ..
ninja

# 4. 验证
ls lib/libautonomy.so
```

### 2.2 系统要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 22.04 LTS（推荐） |
| 架构 | x86-64 或 ARM64 |
| 编译器 | GCC 11+ / Clang，C++17 |
| 内存 | ≥ 8 GB（推荐 16 GB） |
| 构建工具 | CMake ≥ 3.20，Ninja 推荐 |

### 2.3 Docker（可选）

```bash
export AUTONOMY_ENV=/path/to/autonomy
python3 docker/run_autonomy.py -p x86_64
# 容器内
cd /workspace/autonomy/build && cmake -G Ninja .. && ninja
```

### 2.4 配置与运行导航

```bash
# 配置目录默认为 config/
# 编辑 config/autonomy.lua 启用 map / planner / controller / navigator

# 导航离线测试（见 autonomy/system/tools/README.md）
./build/bin/nav_test --config_directory=config
```

### 2.5 构建文档

```bash
cd docs
pip install -r requirements.txt
sphinx-build -b html source build
open build/index.html
```

### 2.6 下一步

| 目标 | 阅读 |
|------|------|
| 理解整体架构 | [§3 系统架构](03_system_architecture.md) |
| 理解导航流水线 | [§4 导航栈全景](04_navigation_stack.md) |
| 通信 API | [03 Communication](../03_Communication/00_guide.md) |
| 安装排错 | [02 Installation](../02_Installation/08_troubleshooting.md) |
