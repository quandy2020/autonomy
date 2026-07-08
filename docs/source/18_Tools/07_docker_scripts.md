# 7. Docker 脚本

`docker/` 目录提供镜像构建、容器运行与依赖安装脚本。

### 7.1 主要入口

| 脚本 | 用途 |
|------|------|
| `docker/run_autonomy.py` | 启动开发容器（x86_64 / aarch64、NVIDIA、数据卷） |
| `docker/build_docker_x86_64.py` | 构建 x86_64 镜像 |
| `docker/build_docker_aarch64.py` | 构建 aarch64 镜像 |
| `docker/scripts/install_docker.py` | Ubuntu 安装 Docker |

### 7.2 快速启动

```bash
python3 docker/run_autonomy.py -p x86_64
docker exec -it SpaceHero /bin/bash
```

### 7.3 Dockerfile

| 平台 | 路径 |
|------|------|
| x86-64 | `docker/dockerfile/autonomy.x86_64.dockerfile` |
| x86-64 + NVIDIA | `docker/dockerfile/autonomy.x86_64.nvidia.dockerfile` |
| aarch64 | `docker/dockerfile/autonomy.aarch64.dockerfile` |

### 7.4 install 脚本

`docker/install/` 含 50+ shell 脚本，由 `scripts/install_deps` (`python3 -m install_deps`) 调用，安装 glog、Protobuf、Ceres、OpenCV、BehaviorTree.CPP、ROS 2 等。

### 7.5 相关文档

- [02 Installation · Docker](../02_Installation/05_docker.md)
- [04 Running · Docker 运行时](../04_Running/05_docker_runtime.md)
