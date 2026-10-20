# 安装与构建指南

本机 Ubuntu 22.04 从 [快速安装](02_quickstart.md) 走；要一致环境用 [Docker](05_docker.md)；Firefly / RK3588 等板端看 [嵌入式](09_embedded_board.md)；只编部分模块看 [编译](06_build.md)。卡住了翻 [排错](08_troubleshooting.md)。

「源码根」= 带顶层 `CMakeLists.txt` 的目录。依赖默认装到 `/usr/local`（可用 `--prefix`，别和 `~/.local` 混用）。依赖脚本：`python3 scripts/install_dependencies.py`（板端加 `--profile board`）。编完后 `source scripts/setup_environment.bash`。构建主路径是 CMake + Ninja，不是 colcon。

其余页面：需求 [03](03_system_requirements.md)、依赖细节 [04](04_dependencies.md)、环境变量 [07](07_environment.md)、产物说明 [01](01_overview.md)。
