# 2. 快速安装

> 约 15–45 分钟（视网络与是否已缓存第三方库而定）。

### 2.1 一键流程（Ubuntu 22.04 宿主机）

```bash
# 1. 克隆
git clone --recurse-submodules https://github.com/quandy2020/autonomy.git
cd autonomy

# 2. 安装依赖（APT + 第三方库，需 sudo）
python3 -m install_deps

# 3. 编译
mkdir -p build && cd build
cmake -G Ninja ..
ninja

# 4. 验证
ls -la lib/libautonomy.so
```

### 2.2 仅安装 APT（跳过第三方编译）

适合已手动安装 `/usr/local` 第三方库的环境：

```bash
python3 -m install_deps --apt-only
```

### 2.3 Docker 快速路径

```bash
export AUTONOMY_ENV=/path/to/autonomy
python3 docker/run_autonomy.py -p x86_64

# 另开终端进入容器
docker exec -it <container_name> /bin/bash
cd /workspace/autonomy/build && cmake -G Ninja .. && ninja
```

详见 [§5 Docker](05_docker.md)。

### 2.4 安装后下一步

| 目标 | 文档 |
|------|------|
| 配置环境变量 | [§7 环境配置](07_environment.md) |
| 运行导航测试 | [04 Running](../04_Running/00_guide.md) |
| 构建文档 | [§6 编译构建](06_build.md) |
