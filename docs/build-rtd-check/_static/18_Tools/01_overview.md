(tools-overview)=
# 1. 工具概览

### 1.1 工具分类

| 类别 | 位置 | 代表工具 |
|------|------|----------|
| 开发脚本 | `scripts/` | `format.py`、`install_dependency.py` |
| 离线测试 | `autonomy/system/tools/` | `autonomy_nav_test` |
| 系统监控 | `autonomy/system/monitor/` | `MonitorRegistry` |
| Autolink CLI | `autolink/autolink/tools/` | `autolink_channel`、`autolink_recorder` |
| Docker | `docker/` | `run_autonomy.py` |
| 主进程 | `autonomy/system/main.cpp` | Autonomy 常驻进程 |

### 1.2 构建产物

| 产物 | 路径 | 条件 |
|------|------|------|
| `libautonomy.so` | `build/lib/` | 默认构建 |
| `autonomy_nav_test` | `build/bin/` | 需 CMake 接入 `system/tools` |
| `autolink_mainboard` | `build/bin/` | 默认构建 autolink |
| BT 插件 `.so` | `build/lib/` | 插件源码存在时 |

### 1.3 文档与实现差异

| 项目 | 文档/注释 | 代码现状 |
|------|-----------|----------|
| `BUILD_TOOLS` | 部分文档要求 `-DBUILD_TOOLS=ON` | 根 CMake 无此 option |
| `autonomy_planning_test` | 多处提及 | 源码目录不存在 |
| `autonomy_controller_test` | 多处提及 | 源码目录不存在 |
| Autolink 工具 | 完整 CLI 集 | 嵌入构建时 `AUTOLINK_BUILD_TOOLS=OFF` |

写文档与排错时以**实际 CMake 与源码**为准。

### 1.4 相关文档

- [§2 快速开始](02_quickstart.md)
- [04 Running](../04_Running/index.rst)
