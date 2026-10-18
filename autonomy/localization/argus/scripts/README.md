# Argus scripts

约定：`<动词>_<对象>[_限定].{sh,py}`；Oxford Spires 统一用 `spires` 缩写。

| 前缀 | 用途 |
|------|------|
| `download_spires_*` | 下载数据 |
| `prepare_spires_*` | 数据准备（GT 等） |
| `convert_*` | 格式转换 |
| `run_spires_*` | 评测 / 流水线入口 |
| `plot_*` | Matplotlib 出图 |
| `viz_open3d_*` | Open3D 交互可视化 |
| `_argus_env.sh` | 公共环境变量与 helper（被 source，勿直接执行） |

## 脚本一览

| 脚本 | 说明 |
|------|------|
| `download_spires_sequences.sh` | 下载一个或多个序列 |
| `download_spires_subset.sh` | 下载默认序列（包装 `download_spires_sequences`） |
| `prepare_spires_gt.py` | 生成 / 补齐 `gt.tum` |
| `convert_kalibr_camera.py` | Kalibr → Argus Equidistant YAML |
| `run_spires_eval.sh` | 单模式离线 + ATE 图 |
| `run_spires_modes.sh` | LIVO/LIO/LO 同图对比 |
| `run_spires_batch.sh` | 多序列批量评测 |
| `run_spires_open3d.sh` | 离线 + ATE + Open3D 全流程 |
| `plot_traj_ate.py` | 单轨迹 Umeyama ATE |
| `plot_modes_compare.py` | 多模式对比图 |
| `viz_open3d_traj.py` | 静态轨迹 + 地图 |
| `viz_open3d_mapping.py` | 按时间增量建图（RViz 风格） |

## Shell 风格

```bash
#!/usr/bin/env bash
# 一句话用途
#
# Usage:
#   ./scripts/<name>.sh ...
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_argus_env.sh
source "${SCRIPT_DIR}/_argus_env.sh"
```

常用环境变量见 `_argus_env.sh`（`ARGUS_SPIRES_DEST`、`ARGUS_SPIRES_BIN`、`ARGUS_LIVO_CONFIG` 等）。

## Python 风格

```python
#!/usr/bin/env python3
"""一句话用途.

Usage:
  python3 scripts/<name>.py --foo
"""
from __future__ import annotations
```

共享 Umeyama / TUM 读写：`plot_traj_ate.py`（其它脚本通过 `importlib` 加载）。
