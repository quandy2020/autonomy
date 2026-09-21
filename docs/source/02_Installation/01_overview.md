(installation-overview)=
# 1. 安装概览

## 1.1 装完你应有什么

| 产物 | 典型位置 | 说明 |
|------|----------|------|
| 域库 | `build/lib/libautonomy_*.so` | 如 `libautonomy_common.so`、`libautonomy_planning.so` |
| 通信库 | `build/lib/libautolink.so`、`libautomsgs.so` | 始终参与构建 |
| 可执行文件 | `build/bin/` | `autolink`、`autonomy.planning`、`autonomy.task` 等 |
| BT 插件 | `build/lib/autonomy_behavior_tree_*.so` | 启用 task/navigator 时 |
| 可选 install | `/usr/local` | `cmake --install` 后用系统前缀运行 |

> 工程已模块化，**没有**单一的 `libautonomy.so` 总库。

## 1.2 三条安装路径

```text
本机 Ubuntu ──► install_dependencies.py ──► cmake + ninja ──► setup_environment.bash
Docker 容器 ──►（镜像多已预装依赖）──► 同上 cmake
嵌入式板 ────► NFS 源码 + 板端 --profile board ──► 本地盘 build/
```

交叉编译（x86 容器 + aarch64 sysroot）属进阶，见 `CMakePresets.json` 的 `jdr-board` 与 `run_autonomy.py --help`。

## 1.3 依赖与构建关系

```text
APT + docker/install/*.sh  →  /usr/local
         │
         ▼
  CMake（AUTONOMY_BUILD_* / BUILD_*）
         │
         ▼
  build/lib + build/bin
```

## 1.4 与 ROS 2

当前主路径是纯 CMake，**不依赖 ROS 2 运行时**。Docker 镜像可预装 Humble 供可视化或共存；ROS 集成见 [04 Running · ROS 2](../04_Running/06_ros2_integration.md)。

下一步：[§2 快速安装](02_quickstart.md)。
