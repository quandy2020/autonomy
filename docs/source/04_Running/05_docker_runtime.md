# 5. Docker 运行时

入口：`docker/run_autonomy.py`。镜像与安装见 [02 Installation · Docker](../02_Installation/05_docker.md)。

### 5.1 启动

```bash
export AUTONOMY_ENV=/path/to/autonomy
python3 "$AUTONOMY_ENV/docker/run_autonomy.py" -p x86_64
docker exec -it SpaceHero /bin/bash # 名可用 AUTONOMY_CONTAINER_NAME 改
```

### 5.2 参数

| 参数 / 变量 | 作用 |
|-------------|------|
| `-p x86_64` \| `aarch64` | 平台 |
| `-n yes` \| `no` \| `auto` | NVIDIA |
| `--data-volume` / `AUTONOMY_DATA_VOLUMES` | 数据盘 |
| `--datasets` / `AUTONOMY_DATASETS` | 离线数据集 → `/data:ro` |
| `AUTONOMY_FORCE_RECREATE=1` | 改挂载后重建 |
| `--as-host-user` | 宿主机 uid:gid |
| `AUTONOMY_NETWORK` | 默认 `host` |

环境变量全表：[§4](04_environment.md)。数据集：[Tools §8](../18_Tools/08_datasets.md)。

### 5.3 容器内运行

```bash
cd /workspace/autonomy
source scripts/setup_environment.bash
autolink_launch autonomy.launch
```

默认 `--entrypoint /bin/bash`。![命令行运行](./images/command_line_run.png)
