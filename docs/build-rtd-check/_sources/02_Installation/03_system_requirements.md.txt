# 3. 系统要求

### 3.1 操作系统

| 系统 | 版本 | 支持度 |
|------|------|--------|
| Ubuntu | 22.04 LTS | ✅ 推荐（与 Docker 镜像一致） |
| Ubuntu | 20.04 LTS | ⚠️ 可尝试，未作为主 CI 目标 |
| 其他 Linux | — | 需自行解决依赖，不保证 |

`scripts/install_dependency.py` 会检测 `/etc/os-release`，非 Ubuntu 时打印警告并继续。

### 3.2 硬件

| 项目 | 最低 | 推荐 |
|------|------|------|
| CPU | 4 核 x86-64 / ARM64 | 8 核及以上 |
| 内存 | 8 GB | 16 GB（编译 OpenCV/Ceres 时） |
| 磁盘 | 20 GB 可用 | 40 GB+（含 Docker 镜像与 build 缓存） |
| GPU | 非必须 | NVIDIA GPU 可选（仿真 / ONNX / Isaac 衍生镜像） |

### 3.3 编译工具链

| 工具 | 版本要求 |
|------|----------|
| CMake | ≥ 3.20 |
| C++ 编译器 | GCC 11+ 或 Clang，**C++17** |
| Ninja | 推荐（`-G Ninja`） |
| Python | 3.8+（运行 `install_dependency.py`） |
| Git | 支持 submodule |

验证：

```bash
cmake --version
g++ --version
ninja --version
```

### 3.4 架构支持

| 架构 | 宿主机 | Docker |
|------|--------|--------|
| x86-64 | ✅ | `run_autonomy.py -p x86_64` |
| ARM64 (aarch64) | ✅ | `run_autonomy.py -p aarch64` |
| NVIDIA GPU | 可选 | `run_autonomy.py -p x86_64 -n yes` |

### 3.5 网络

- 克隆仓库与 submodule 需访问 GitHub / Gitee
- `docker/install/*.sh` 会从网络下载第三方源码（glog、Ceres、OpenCV 等）
- 国内用户可考虑 Gitee 镜像：`https://gitee.com/quanduyong/autonomy.git`

### 3.6 相关文档

- [§4 依赖安装](04_dependencies.md)
- [§5 Docker 环境](05_docker.md)
