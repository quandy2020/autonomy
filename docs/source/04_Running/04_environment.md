# 4. 环境变量

## 背景

`run_autonomy.py` 挂卷、容器内跑栈时常见的 `AUTONOMY_*` / `AUTOLINK_*`。本机开发机多数只需 `source scripts/setup_environment.bash`。

## 思路

容器、板端与本机 dev 的差异主要在 `AUTONOMY_*` / `AUTOLINK_*` 指向的安装前缀与域裁剪开关。找不到 conf 或 `.so` 时，先对照 `setup_environment.bash` 与步骤表里的变量默认值。

## 步骤

### 1. 宿主机（启动容器）

| 变量 | 作用 | 默认 |
|------|------|------|
| `AUTONOMY_ENV` | 源码根 → `/workspace/autonomy` | 自动检测 |
| `AUTONOMY_CONTAINER_NAME` | 容器名 | `SpaceHero` |
| `AUTONOMY_DATA_VOLUMES` | 通用数据盘（可多条） | 无 |
| `AUTONOMY_DATASETS` | 离线数据集 → `/data:ro` | 无 |
| `AUTONOMY_FORCE_RECREATE` | `1` 重建容器 | 关 |
| `AUTONOMY_NETWORK` | 网络模式 | `host` |
| `AUTONOMY_PORTS` | 端口映射 | 含 8765 等 |
| `AUTONOMY_KEEP_ISAAC_ENTRYPOINT` | 保留 Kit ENTRYPOINT | 关 |
| `AUTONOMY_RUN_AS_HOST_USER` | 宿主机 uid:gid | 关 |

`AUTONOMY_DATASETS`：`HOST` 或 `HOST:CONTAINER[:MODE]`。见 [Tools §8](../18_Tools/08_datasets.md)。

### 2. 容器内

| 变量 | 作用 |
|------|------|
| `PATH` | 含 `build/bin` / `build/autonomy/bin` |
| `AUTOLINK_LAUNCH_PATH` | launch 目录 |
| `AUTOLINK_PATH` | autolink 源/安装（按部署） |
| `LD_LIBRARY_PATH` | 构建产物 `lib` |
| `AUTONOMY_BT_PLUGIN_PATH` | BT 插件 `.so` |
| `PYTHONPATH` | `src/autonomy` + `install/autonomy/python`（先于 `lib/python`） |
| `GLOG_logtostderr` | 日志到 stderr |

### 3. 参见

[§5 Docker 运行时](05_docker_runtime.md) · [02 Installation · Docker](../02_Installation/05_docker.md)
