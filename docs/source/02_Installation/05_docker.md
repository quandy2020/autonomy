# 5. Docker 环境

用容器获得与 CI 一致的 Ubuntu + `/usr/local` 第三方库，再在容器内按本机同一套 CMake 编译。

---

### Step 1 — 安装 Docker（宿主机）

```bash
cd docker/scripts
python3 install_docker.py
docker --version
```

需要 GPU 时再装 NVIDIA Container Toolkit（见仓库 `docker/scripts/`）。

---

### Step 2 — 启动容器

在 **源码根的上一级或本机任意处**，指定挂载路径：

```bash
export AUTONOMY_ENV=/path/to/autonomy   # 宿主机源码根，挂到容器 /workspace/autonomy

python3 "$AUTONOMY_ENV/docker/run_autonomy.py" -p x86_64
# GPU：
# python3 "$AUTONOMY_ENV/docker/run_autonomy.py" -p x86_64 -n yes
# ARM 机 / 镜像：
# python3 "$AUTONOMY_ENV/docker/run_autonomy.py" -p aarch64
```

| 选项 | 作用 |
|------|------|
| `--as-host-user` | 以宿主机 uid:gid 写文件，避免 root 属主 |
| `--data-volume PATH` | 额外挂载数据盘 |
| `--` 后参数 | 透传给 `docker run` |

---

### Step 3 — 进入容器

```bash
docker exec -it <container_name> bash
cd /workspace/autonomy
# 确认顶层 CMakeLists.txt 存在；monorepo 时可能是 /workspace/autonomy/src/autonomy
ls CMakeLists.txt scripts docker
```

容器名以 `run_autonomy.py` 输出为准。

---

### Step 4 — 容器内依赖（按需）

预构建镜像通常已含 `/usr/local` 库。缺什么再补：

```bash
python3 scripts/install_dependencies.py --skip-installed
```

---

### Step 5 — 容器内编译

与 [§2](02_quickstart.md) / [§6.1](06_build.md) 相同：

```bash
mkdir -p build && cd build
cmake -G Ninja .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/usr/local \
  -DCMAKE_INSTALL_PREFIX=/usr/local
ninja -j$(nproc)
cd ..
source scripts/setup_environment.bash
```

模块化裁剪：同一套 `-DAUTONOMY_BUILD_*=` / `-DBUILD_*=`，见 [§6.2](06_build.md)。

---

### 镜像与自建

| 平台 | Dockerfile |
|------|------------|
| x86-64 | `docker/dockerfile/autonomy.x86_64.dockerfile` |
| x86-64 + NVIDIA | `docker/dockerfile/autonomy.x86_64.nvidia.dockerfile`（若存在） |
| aarch64 | `docker/dockerfile/autonomy.aarch64.dockerfile` |

```bash
# 参见 docker/ 下 build 脚本，例如：
python3 docker/build_docker_x86_64.py
```

### 注意

- `sudo cmake --install` 装进 **容器内** `/usr/local`；要持久化请挂卷或在宿主机/板上再装。  
- 第三方与 Autonomy 必须同一前缀，勿混用 `~/grpc` 与系统 protobuf。  
- 运行期 Docker 说明见 [04 Running · Docker](../04_Running/05_docker_runtime.md)。
