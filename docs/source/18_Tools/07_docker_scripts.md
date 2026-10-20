# 7. Docker 脚本

| 脚本 | 用途 |
|------|------|
| `docker/run_autonomy.py` | 开发容器（SpaceHero 等） |
| `docker/build_docker_*.py` | 构建镜像 |
| `scripts/install_docker.py` | 安装 Docker |
```bash
python3 docker/run_autonomy.py -p x86_64
```
常用：`--data-volume`、`--datasets`、`--as-host-user`。详见 [04 Running · Docker](../04_Running/05_docker_runtime.md)。

**什么时候用：** 在容器里编/跑 autonomy，或挂载数据集盘。

### 与构建开关

容器内 cmake 仍可 `-DBUILD_GRPC=ON/OFF`、`-DBUILD_AUTOVIZ=OFF`（无 GUI 时省依赖）。  
Bridge 联调需映射 **5005**（或你的 grpc port）。

### 常见坑

| 现象 | 说明 |
|------|------|
| 容器内源码空 | 设 `AUTONOMY_ENV` 指向宿主机仓库 |
| 数据集不可见 | `AUTONOMY_DATASETS` + 重建容器 |
| grpcurl 连宿主机 | 用 `host.docker.internal` 或 `--network host` |

数据集：[08](08_datasets.md)。
