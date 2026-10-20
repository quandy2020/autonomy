# 5. Docker 环境

## 背景

本机直接装依赖会污染 `/usr/local`，且和 CI / 同事环境容易漂移。需要一套与仓库脚本一致的 Ubuntu + 第三方库，又能把源码挂进去编。

运行期挂载、数据集见 [04 Running · Docker](../04_Running/05_docker_runtime.md)；这里只管 **起容器 → 进容器 → 按本机同一套 CMake 编**。

## 思路

宿主机只负责 Docker 与挂载；编译与 `install_dependencies.py` 都在容器内完成，前缀仍用容器里的 `/usr/local`（或你挂进去的同一前缀）。镜像按架构分：`x86_64` / `aarch64`，可选 GPU。


## 步骤

### 1. 宿主机装 Docker

```bash
cd docker/scripts
python3 install_docker.py
docker --version
```

要 GPU 再装 NVIDIA Container Toolkit（见 `docker/scripts/`）。

### 2. 启动容器

```bash
export AUTONOMY_ENV=/path/to/autonomy   # 宿主机源码根 → 容器 /workspace/autonomy

python3 "$AUTONOMY_ENV/docker/run_autonomy.py" -p x86_64
# GPU：  python3 ... -p x86_64 -n yes
# ARM：  python3 ... -p aarch64
```

| 选项 | 作用 |
|------|------|
| `--as-host-user` | 宿主机 uid:gid |
| `--data-volume` / `AUTONOMY_DATA_VOLUMES` | 通用数据盘 |
| `--datasets` / `AUTONOMY_DATASETS` | 离线数据集 → `/data` |
| `--` 后 | 透传 `docker run` |

### 3. 进入容器并确认源码根

```bash
docker exec -it <container_name> bash
cd /workspace/autonomy
# monorepo 时可能是 /workspace/autonomy/src/autonomy
ls CMakeLists.txt scripts docker
```

容器名以 `run_autonomy.py` 输出为准。

### 4. 缺库再补依赖

预构建镜像通常已有 `/usr/local` 库：

```bash
python3 scripts/install_dependencies.py --skip-installed
```

### 5. 容器内编译

与本机相同：[§2](02_quickstart.md)、[§6](06_build.md)。裁剪见 [§6.2](06_build.md)。

### 6. 自建镜像（可选）

| 平台 | Dockerfile |
|------|------------|
| x86-64 | `docker/dockerfile/autonomy.x86_64.dockerfile` |
| x86-64 + NVIDIA | `docker/dockerfile/autonomy.x86_64.nvidia.dockerfile`（若存在） |
| aarch64 | `docker/dockerfile/autonomy.aarch64.dockerfile` |

```bash
python3 docker/build_docker_x86_64.py
```

### 注意

- `sudo cmake --install` 装进 **容器内** `/usr/local`；要持久化请挂卷或宿主机/板上再装。  
- 第三方与 Autonomy 同一前缀，勿混 `~/grpc` 与系统 protobuf。  
- 运行期说明：[04 Running · Docker](../04_Running/05_docker_runtime.md)。
