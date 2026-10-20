# 0. 安装与构建指南

## 背景

装 Autonomy 最容易踩的坑：依赖前缀混用、把 `build` 放 NFS、编完不 `source` 环境、板端和开发机用同一套「全开」开关。跑起来之后的 launch / 多进程见 [04 Running](../04_Running/00_guide.md)；目录与开关总表见 [01 · 仓库](../01_Instructions/04_repository.md)。

## 思路

按环境选路径，主线始终是 **依赖同一 prefix → CMake + Ninja → `setup_environment.bash`**：

| 环境 | 入口 |
|------|------|
| Ubuntu 22.04 开发机 | [02 快速安装](02_quickstart.md) |
| 可复现容器 | [05 Docker](05_docker.md) |
| Firefly / RK3588 等 aarch64 | [09 嵌入式](09_embedded_board.md) |
| 只编若干域 | [06 编译](06_build.md) |

卡住先 [08 排错](08_troubleshooting.md)，再对 [03 要求](03_system_requirements.md)、[04 依赖](04_dependencies.md)。


## 步骤

### 1. 各篇怎么用

| 文档 | 解决什么 |
|------|----------|
| [01 概览](01_overview.md) | 产物落哪、`build/` 与 install 前缀 |
| [02 快速安装](02_quickstart.md) | 本机逐步：依赖 → cmake → ninja |
| [03 系统要求](03_system_requirements.md) | OS、内存、工具链底线 |
| [04 依赖](04_dependencies.md) | `install_dependencies.py` 与 profile |
| [05 Docker](05_docker.md) | 镜像内编译；运行期挂载见 Running |
| [06 编译](06_build.md) | Ninja、模块开关、常见 FATAL |
| [07 环境](07_environment.md) | `setup_environment.bash`、路径变量 |
| [08 排错](08_troubleshooting.md) | 依赖 / 链接 / 版本 |
| [09 板端](09_embedded_board.md) | 板端编、NFS |

### 2. 动手前记住

1. **源码根** = 含顶层 `CMakeLists.txt` 的目录。  
2. **依赖前缀单一**：默认 `/usr/local`，勿一半 `~/.local` 一半系统。  
3. **主路径是 CMake + Ninja**，不是 colcon。  
4. **板端 build 在本地盘**；产物是多个 `libautonomy_*.so`。  
5. **裁剪成套**：关 `AUTONOMY_BUILD_*` 时依赖域仍要满足，否则 CMake FATAL。

### 3. 本机最短成功路径

```bash
git clone --recurse-submodules https://github.com/quandy2020/autonomy.git
cd autonomy && git submodule update --init --recursive
python3 scripts/install_dependencies.py --skip-installed
mkdir -p build && cd build
cmake -G Ninja .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/usr/local
ninja -j$(nproc) && cd ..
source scripts/setup_environment.bash
```

验证：`ls build/lib/libautonomy_common.so build/lib/libautolink.so`。然后 [Running · 快速运行](../04_Running/02_quickstart.md)。

### 4. 常见坑

| 现象 | 先查 |
|------|------|
| `find_package` 失败 | 是否同一 prefix；`--skip-installed` 是否跳过关键包 |
| 链接找不到符号 | Debug/Release 或新旧 install 混用 |
| 板端极慢 / 怪错 | build 是否在 NFS |
| 编过跑不起来 | 是否 `source scripts/setup_environment.bash` |
| Docker 缺数据集 | Running 里 `AUTONOMY_DATASETS` / 挂载 |
