# 3. 系统要求

在跑 `install_dependencies.py` 或编 OpenCV / Ceres 之前，需要确认 OS、内存与工具链满足底线；否则会在中途 OOM 或 CMake 报找不到库。

以 **Ubuntu 22.04 + CMake ≥ 3.20 + GCC 11 + C++17** 为基准，与 Docker 镜像一致；板端与 GPU 为可选分支，不改变主路径要求。

## 操作系统

| 系统 | 版本 | 支持 |
|------|------|------|
| Ubuntu | 22.04 LTS | 推荐（与 Docker 镜像一致） |
| Ubuntu | 20.04 LTS | 可尝试，非主目标 |
| 其他 Linux | 未验证 | 需自行对齐依赖 |

## 硬件

| 项目 | 最低 | 推荐 |
|------|------|------|
| CPU | 4 核 x86-64 或 ARM64 | 8 核+ |
| 内存 | 8 GB | 16 GB+（编 OpenCV/Ceres 时） |
| 磁盘 | 20 GB 可用 | 40 GB+（含 Docker / build） |
| GPU | 非必须 | NVIDIA 可选（仿真 / ONNX） |

## 工具链

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

## 架构与 Docker 对应

| 架构 | 本机 | Docker |
|------|------|--------|
| x86-64 | 常见开发机 | `run_autonomy.py -p x86_64` |
| aarch64 | Firefly 等板端 | `run_autonomy.py -p aarch64` |
| NVIDIA GPU | 可选 | `-p x86_64 -n yes` |

## 网络

- 克隆与 submodule 需访问 GitHub / Gitee  
- `docker/install/*.sh` 会下载第三方源码  
- 板端装依赖需出网（常经开发机 NAT，见 [§9](09_embedded_board.md)）

装不上时对照 [§8 排错](08_troubleshooting.md)。下一步：[§2 快速安装](02_quickstart.md) 或 [§4 依赖](04_dependencies.md)。
