(installation-troubleshooting)=
# 8. 故障排查

### 8.1 依赖安装

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `apt-get install` 失败 | 包冲突 / 源不可用 | `sudo apt-get -y --fix-broken install` 后重试 |
| `install_opencv.sh` 中断 | 网络 / 内存不足 | `--resume-from install_opencv.sh` |
| `libceres.so not found` | 第三方未装全 | `python3 scripts/install_dependency.py --thirdparty-only` |
| `behaviortree_cpp` 找不到 | BT 库未安装 | 确认 `install_behaviortree_cpp.sh` 成功 |
| 非 Ubuntu 警告 | 脚本面向 Ubuntu | 手动对照 `APT_PACKAGES` 安装等效包 |

### 8.2 CMake 配置

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `Could NOT find Protobuf` | 缺少 dev 包 | `sudo apt install libprotobuf-dev protobuf-compiler` |
| `Could NOT find Ceres` | `/usr/local` 无 Ceres | 运行 `install_ceres_solver.sh` |
| `Could NOT find Lua` | 缺少 lua5.3 | `sudo apt install liblua5.3-dev` |
| gRPC 相关错误 | `BUILD_GRPC=ON` 但缺 gRPC | 安装 gRPC 或 `-DBUILD_GRPC=OFF` |

清理后重配：

```bash
cd build && rm -rf * && cmake -G Ninja .. && ninja
```

### 8.3 编译错误

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| C++17 相关错误 | 编译器过旧 | 升级 GCC 至 11+ |
| OOM / 编译器被 kill | 内存不足 | `ninja -j2` 限制并行 |
| 链接错误 undefined reference | 依赖顺序 / 缺库 | 检查 `/usr/local/lib` 是否在 `LD_LIBRARY_PATH` |
| submodule 缺失 | 未初始化 submodule | `git submodule update --init --recursive` |

### 8.4 Docker

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 无法启动容器 | Docker 未运行 | `sudo systemctl start docker` |
| GPU 不可用 | 未装 NVIDIA Toolkit | `install_nvidia_container_toolkit.sh` |
| 挂载目录为空 | `AUTONOMY_ENV` 未设置 | `export AUTONOMY_ENV=/正确/路径` |
| 权限 denied | root 创建的文件 | `run_autonomy.py --as-host-user` |
| Isaac 镜像自动启动 Kit | ENTRYPOINT 行为 | 默认已覆盖为 bash；勿加 `--keep-isaac-entrypoint` 除非需要 |

### 8.5 运行时

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `libautonomy.so: cannot open` | 未设置 rpath | 从 `build/bin` 运行或 `export LD_LIBRARY_PATH=build/lib` |
| `Autonomy not ready` | BT 插件路径错误 | 设置 `AUTONOMY_BT_PLUGIN_PATH` |
| 配置加载失败 | `config_directory` 错误 | 确认 `config/autonomy.lua` 存在 |
| TF / 帧名错误 | `common.lua` 不一致 | 统一 `global_frame` / `robot_base_frame` |

### 8.6 Habitat-Sim（可选）

仅在使用仿真相关功能时需要：

```bash
git clone --branch stable https://github.com/facebookresearch/habitat-sim.git
cd habitat-sim
pip3 install -r requirements.txt
git config --global --add safe.directory '*'
python3 setup.py install --headless --no-update-submodules
```

验证：

```bash
python3 -c "import habitat_sim; print('OK')"
```

内存不足时限制并行：`python3 setup.py build_ext --parallel 1 install --headless --no-update-submodules`

### 8.7 获取帮助

| 渠道 | 说明 |
|------|------|
| GitHub Issues | https://github.com/quandy2020/autonomy/issues |
| 文档 | [01 Instructions](../01_Instructions/00_guide.md) |
| FAQ | [19 FAQs](../19_FAQs/index.rst) |

### 8.8 相关文档

- [§4 依赖安装](04_dependencies.md)
- [§5 Docker 环境](05_docker.md)
- [§6 编译构建](06_build.md)
