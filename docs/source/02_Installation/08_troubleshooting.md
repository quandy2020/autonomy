(installation-troubleshooting)=
# 8. 故障排查

按现象对照；板端专项亦见 [§9.8](09_embedded_board.md)。

### 8.1 依赖安装

| 现象 | 处理 |
|------|------|
| `apt-get` 失败 | `sudo apt-get -y --fix-broken install` 后重试 |
| `install_opencv.sh` 中断 | `--resume-from install_opencv.sh --skip-installed` |
| 找不到 Ceres / OSQP / BT | 对应 `bash docker/install/install_*.sh`，确认在 `/usr/local` |
| OSQP 只在 `~/.local` | 用 `--prefix /usr/local`（或同一自定义前缀）重装；**不要**混 `~/.local` 与 `/usr/local` |
| 非 Ubuntu 警告 | 对照脚本 APT 列表自行装等效包 |

### 8.2 CMake 配置

| 现象 | 处理 |
|------|------|
| `Could NOT find Protobuf` | `/usr/local` 装 `install_protobuf.sh`；板端勿混 apt protobuf |
| `Could NOT find Ceres` / `OSQP` / `Lua` | 装对应库；Lua：`sudo apt install liblua5.3-dev` |
| gRPC 报错 | `install_grpc.sh` 或 `-DBUILD_GRPC=OFF` |
| 域依赖 FATAL | 按报错打开缺失的 `AUTONOMY_BUILD_*` |
| Ipopt `stddef` 错误 | 重配 cmake，链接 `Ipopt::Ipopt` |

```bash
rm -rf build && mkdir build && cd build
cmake -G Ninja .. -DCMAKE_PREFIX_PATH=/usr/local && ninja -j$(nproc)
```

### 8.3 编译

| 现象 | 处理 |
|------|------|
| C++17 / 编译器过旧 | GCC 11+ |
| OOM / 被 kill | `ninja -j2`；板端勿在 NFS 上 build |
| protobuf / protoc 版本冲突 | 统一 `/usr/local`；清 build 重配；勿用 `~/grpc` 的 3.14 混 3.19 |
| submodule 缺失 | `git submodule update --init --recursive` |

### 8.4 Docker

| 现象 | 处理 |
|------|------|
| 容器起不来 | `sudo systemctl start docker` |
| 挂载为空 | `export AUTONOMY_ENV=/正确/源码根` |
| 权限 denied | `run_autonomy.py --as-host-user` |
| GPU 不可用 | 装 NVIDIA Container Toolkit |
| Isaac 镜像自动起 Kit | 不要加 `--keep-isaac-entrypoint`（默认已覆盖 entrypoint） |

### 8.5 运行时

| 现象 | 处理 |
|------|------|
| `.so: cannot open` | `source setup`；或 `LD_LIBRARY_PATH` 含 `build/lib` 与 `/usr/local/lib` |
| `task.launch` / launch 找不到 | 用新版 `autolink`；`AUTOLINK_LAUNCH_PATH` 为冒号分隔多路径；确认未用过期 `build/bin/autolink` 盖住 install |
| BT 插件加载失败 | 检查 `AUTONOMY_BT_PLUGIN_PATH` |
| 配置加载失败 | 确认 `config/autonomy.lua` 与 `AUTONOMY_CONFIG_DIR` |
| autoviz GLX | 容器 Mesa / `xhost +local:docker`；见 Running 文档 |

### 8.6 嵌入式板 / NFS

| 现象 | 处理 |
|------|------|
| apt 解析失败 | 板子无默认路由 → 开发机 NAT + `ip route replace default via <HOST_IP>` |
| `ldconfig` Permission denied | 用当前 `docker/install`（带 sudo/`autonomy_ldconfig`） |
| glog 符号 / 版本混乱 | `--profile board`，purge apt glog，只用 `/usr/local` 0.6 |
| NFS 上编译极慢 | `-B` 必须在本地盘（如 `~/autonomy_ws/build`） |

### 8.7 获取帮助

| 渠道 | 链接 |
|------|------|
| Issues | https://github.com/quandy2020/autonomy/issues |
| 文档入口 | [01 Instructions](../01_Instructions/01_overview.md) |
| FAQ | [19 FAQs](../19_FAQs/index.rst) |

相关：[§4](04_dependencies.md) · [§5](05_docker.md) · [§6](06_build.md) · [§9](09_embedded_board.md)
