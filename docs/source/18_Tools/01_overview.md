# 1. 工具概览

### 1.1 工具分类

| 类别 | 位置 | 代表工具 |
|------|------|----------|
| 开发脚本 | `scripts/` | `format.py`、`install_deps`、`package_autonomy_artifact.sh` |
| 系统监控 | `autonomy/system/monitor/` | `MonitorRegistry` / `autonomy.monitor` |
| Autolink CLI | `autolink/autolink/tools/` | `autolink_channel`、`autolink_recorder`、`autolink_launch` |
| Docker | `docker/` | `run_autonomy.py` |
| 模块进程 | `autonomy/*/ *_main.cpp` | planning / control / task / bridge 等 |

> 离线 `autonomy_nav_test` 与进程内聚合入口已移除；端到端见 [04 Running](../04_Running/02_quickstart.md)。

### 1.2 构建产物

| 产物 | 路径 | 条件 |
|------|------|------|
| `libautonomy.so` | `build/lib/` | 默认构建 |
| `autonomy.planning` 等 | `build/bin/` | 对应模块开启 |
| `autolink_launch` | `build/bin/` | 默认构建 autolink |
| BT 插件 `.so` | `build/lib/` | 插件源码存在时 |

### 1.3 文档与实现差异

| 项目 | 文档/注释 | 代码现状 |
|------|-----------|----------|
| `BUILD_TOOLS` | 部分文档要求 `-DBUILD_TOOLS=ON` | 根 CMake 无此 option |
| `autonomy_planning_test` | 多处提及 | 源码目录不存在 |
| `autonomy_controller_test` | 多处提及 | 源码目录不存在 |
| Autolink 工具 | 完整 CLI 集 | 嵌入构建时部分工具可能关闭 |

写文档与排错时以**实际 CMake 与源码**为准。

### 1.4 相关文档

- [§2 快速开始](02_quickstart.md)
- [04 Running](../04_Running/index.rst)
