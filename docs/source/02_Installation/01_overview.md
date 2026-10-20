(installation-overview)=
# 1. 安装概览

安装前需要知道：成功后磁盘上会有什么、本机 / Docker / 板端三条路径如何选，以及工程与 ROS 2 的关系。逐步命令不在本页重复。

三条路径最终都落到同一套 CMake 开关与 `setup_environment.bash`；差异在依赖来源（本机脚本、镜像预装、板端 `--profile board`）和 build 目录位置（板端 build 必须在本地盘）。

## 装完你应有什么

| 产物 | 典型位置 | 说明 |
|------|----------|------|
| 域库 | `build/lib/libautonomy_*.so` | 如 `libautonomy_common.so`、`libautonomy_planning.so` |
| 通信库 | `build/lib/libautolink.so`、`libautomsgs.so` | 始终参与构建 |
| 可执行文件 | `build/bin/` | `autolink`、`autonomy.planning`、`autonomy.task` 等 |
| BT 插件 | `build/lib/autonomy_behavior_tree_*.so` | 启用 task 时 |
| 可选 install | `/usr/local` | `cmake --install` 后用系统前缀运行 |

工程已模块化，**没有**单一的 `libautonomy.so` 总库。

## 三条安装路径

```text
本机 Ubuntu ──► install_dependencies.py ──► cmake + ninja ──► setup_environment.bash
Docker 容器 ──►（镜像多已预装依赖）──► 同上 cmake
嵌入式板 ────► NFS 源码 + 板端 --profile board ──► 本地盘 build/
```

交叉编译（x86 容器 + aarch64 sysroot）属进阶，见 `CMakePresets.json` 的 `jdr-board` 与 `run_autonomy.py --help`。

## 依赖与构建关系

```text
APT + docker/install/*.sh → /usr/local
        │
        ▼
CMake（AUTONOMY_BUILD_* / BUILD_*）
        │
        ▼
build/lib + build/bin
```

## 与 ROS 2

主路径纯 CMake，**不依赖 ROS 2 运行时**。ROS 集成见 [04 Running · ROS 2](../04_Running/06_ros2_integration.md)。

下一步：[02 快速安装](02_quickstart.md)。
