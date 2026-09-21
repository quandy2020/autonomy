# 安装与构建指南

按场景选一条路径，跟着编号命令做即可。排错见 [§8](08_troubleshooting.md)。

## 选哪条路径

| 你的环境 | 从这里开始 | 预计时间 |
|----------|------------|----------|
| Ubuntu 22.04 本机开发 | [§2 快速安装](02_quickstart.md) | 30–90 分钟（含第三方库） |
| 想用一致容器环境 | [§5 Docker](05_docker.md) | 镜像已有依赖时约 15 分钟 |
| Firefly / RK3588 等 aarch64 板 | [§9 嵌入式板端](09_embedded_board.md) | NFS 配好后约 1–3 小时 |
| 只编部分模块 | [§6 编译构建 · 模块化](06_build.md) | 在已能全量编译的基础上 |

## 贯穿全文的约定

| 约定 | 含义 |
|------|------|
| **源码根** | 含顶层 `CMakeLists.txt` 的目录（独立克隆即仓库根；monorepo 中多为 `src/autonomy`） |
| **依赖前缀** | 第三方库装到 **`/usr/local`**（可用 `--prefix`）；CMake 用同一 `CMAKE_PREFIX_PATH`，勿混 `~/.local` |
| **依赖入口** | `python3 scripts/install_dependencies.py`（板端加 `--profile board`） |
| **运行环境** | `source scripts/setup_environment.bash`（安装后用 `/usr/local/share/autonomy/setup.bash`） |
| **构建方式** | **CMake + Ninja**；不以 `colcon` 为主路径 |

## 阅读顺序（首次上手）

1. [§3 系统要求](03_system_requirements.md) — 确认 OS / 磁盘  
2. [§2 快速安装](02_quickstart.md) — 克隆 → 依赖 → 编译 → 验证  
3. [§7 环境配置](07_environment.md) — 每个新终端 `source` 一次  
4. [04 Running](../04_Running/00_guide.md) — 启动进程栈  

细节与裁剪：[§4 依赖](04_dependencies.md) · [§6 编译](06_build.md) · [§1 概览](01_overview.md)。
