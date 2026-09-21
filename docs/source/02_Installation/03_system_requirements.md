# 3. 系统要求

### 3.1 操作系统

| 系统 | 版本 | 支持 |
|------|------|------|
| Ubuntu | 22.04 LTS | 推荐（与 Docker 镜像一致） |
| Ubuntu | 20.04 LTS | 可尝试，非主目标 |
| 其他 Linux | — | 需自行对齐依赖 |

### 3.2 硬件

| 项目 | 最低 | 推荐 |
|------|------|------|
| CPU | 4 核 x86-64 或 ARM64 | 8 核+ |
| 内存 | 8 GB | 16 GB+（编 OpenCV/Ceres 时） |
| 磁盘 | 20 GB 可用 | 40 GB+（含 Docker / build） |
| GPU | 非必须 | NVIDIA 可选（仿真 / ONNX） |

### 3.3 工具链

| 工具 | 要求 |
|------|------|
| CMake | ≥ 3.20 |
| 编译器 | GCC 11+ 或 Clang，**C++17** |
| Ninja | 推荐 |
| Python | 3.8+ |
| Git | 支持 submodule |

```bash
cmake --version && g++ --version && ninja --version
```

### 3.4 架构

| 架构 | 本机 | Docker |
|------|------|--------|
| x86-64 | ✅ | `run_autonomy.py -p x86_64` |
| aarch64 | ✅ | `run_autonomy.py -p aarch64` |
| NVIDIA GPU | 可选 | `-p x86_64 -n yes` |

### 3.5 网络

- 克隆与 submodule 需访问 GitHub / Gitee  
- `docker/install/*.sh` 会下载第三方源码  
- 板端装依赖需出网（常经开发机 NAT，见 [§9](09_embedded_board.md)）

下一步：[§2 快速安装](02_quickstart.md) 或 [§4 依赖](04_dependencies.md)。
