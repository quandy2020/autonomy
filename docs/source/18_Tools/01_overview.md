# 1. 工具概览

仓库工具散在 `scripts/`、`tools/python/`、`autonomy/tools/` 与 Autolink/Docker 辅助脚本中；新 clone 时容易不知道 bag 转换、数据集发布或 Bridge CLI 在哪。

按 **开发阶段** 选入口：日常编译 launch 走 [02 快速开始](02_quickstart.md)；离线 bag/数据集看 04/08；联调 Bridge 用 `automsgs/tools/cli/rpc-cli.py`（需 `BUILD_GRPC=ON`）；桌面 3D 看 autoviz（`BUILD_AUTOVIZ`）。

### 类别索引

| 类别 | 位置 | 文档 |
|------|------|------|
| 仓库脚本 | `scripts/`、`tools/python/` | [03](03_dev_scripts.md) |
| bag → record | `autonomy/autonomy/tools/bag_convert/` | [04](04_bag_convert.md) |
| 数据集发布 | `autonomy/tools/datasets/` | [08](08_datasets.md) |
| Autolink CLI | autolink 构建产物 | [06](06_autolink_tools.md) |
| Docker 辅助 | `docker/run_autonomy.py` 等 | [07](07_docker_scripts.md) |
| 监控 | `autonomy/system/monitor/` | [05](05_monitor.md) |
| autoviz 等 | 顶层 `autoviz/` | [09](09_ecosystem.md) |

### 构建产物

- 二进制：`build/bin`  
- BT 插件：`build/lib`（launch 时 `AUTONOMY_BT_PLUGIN_PATH`）

### Bridge CLI

`automsgs/tools/cli/rpc-cli.py` — 见 [15 Bridge · 连接](../15_Bridge/rpcs/01_connection_guide.md)。
