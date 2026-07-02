# 5. Docker 环境

Docker 提供一致的 Ubuntu 开发环境，支持 **x86-64 / ARM64** 及可选 **NVIDIA GPU** 镜像。

### 5.1 安装 Docker

```bash
cd docker/scripts
python3 install_docker.py
# 或
sudo ./install_docker.sh   # 若存在 shell 封装

docker --version
```

### 5.2 启动容器

入口脚本：**`docker/run_autonomy.py`**

```bash
# 设置仓库根目录（用于 bind mount）
export AUTONOMY_ENV=/path/to/autonomy

# 自动检测平台，优先 NVIDIA 镜像（若可用）
python3 docker/run_autonomy.py

# 指定平台
python3 docker/run_autonomy.py -p x86_64
python3 docker/run_autonomy.py -p aarch64

# 强制 NVIDIA 镜像
python3 docker/run_autonomy.py -p x86_64 -n yes

# 强制标准镜像（无 GPU）
python3 docker/run_autonomy.py -p x86_64 -n no
```

### 5.3 常用选项

| 选项 / 环境变量 | 说明 |
|-----------------|------|
| `AUTONOMY_ENV` | 宿主机仓库路径，挂载到容器 `/workspace/autonomy` |
| `--as-host-user` / `AUTONOMY_RUN_AS_HOST_USER=1` | 以宿主机 uid:gid 运行 |
| `--data-volume PATH` / `AUTONOMY_DATA_VOLUMES` | 额外挂载数据卷 |
| `--keep-isaac-entrypoint` | 保留 Isaac 衍生镜像 ENTRYPOINT |
| 额外参数 `--` | 透传给 `docker run` |

### 5.4 进入容器

```bash
docker exec -it <container_name> /bin/bash
cd /workspace/autonomy
```

容器名由 `docker_utils.resolve_container_name` 决定，常见为项目相关名称（参见脚本输出）。

### 5.5 容器内编译

```bash
cd /workspace/autonomy
python3 scripts/install_dependency.py   # 若镜像未预装完整依赖
mkdir -p build && cd build
cmake -G Ninja ..
ninja
```

预构建镜像通常已包含大部分 `/usr/local` 第三方库，可能仅需 `ninja`。

### 5.6 镜像与 Dockerfile

| 平台 | Dockerfile 路径 |
|------|-----------------|
| x86-64 | `docker/dockerfile/autonomy.x86_64.dockerfile` |
| x86-64 + NVIDIA | `docker/dockerfile/autonomy.x86_64.nvidia.dockerfile`（若存在） |
| aarch64 | `docker/dockerfile/autonomy.aarch64.dockerfile` |

构建镜像：

```bash
python3 docker/build_docker_x86_64.py    # 参见 docker/ 目录脚本
```

### 5.7 与宿主机开发的差异

| 项目 | 宿主机 | Docker |
|------|--------|--------|
| 依赖隔离 | 污染系统 `/usr/local` | 隔离在镜像内 |
| 文件权限 | 原生 | `--as-host-user` 可避免 root 属主 |
| GPU | 原生驱动 | 需 NVIDIA Container Toolkit |
| 编译速度 | 通常更快 | bind mount I/O 可能略慢 |

### 5.8 相关文档

- [§2 快速安装](02_quickstart.md)
- [04 Running · Docker](../04_Running/05_docker_runtime.md)
