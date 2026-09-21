# 4. 依赖安装

Autonomy 依赖通过 **`scripts/install_deps` (`python3 -m install_deps`)** 统一安装，与 `docker/dockerfile/autonomy.x86_64.dockerfile` 保持同步。

### 4.1 脚本用法

```bash
# 完整安装（APT + 第三方）
python3 -m install_deps

# 仅 APT
python3 -m install_deps --apt-only

# 仅第三方（需已装 APT）
python3 -m install_deps --thirdparty-only

# 列出 APT 包名
python3 -m install_deps --list-apt

# 从某脚本续装（中断后恢复）
python3 -m install_deps --resume-from install_opencv.sh

# 跳过已检测到的库
python3 -m install_deps --skip-installed

# 预览命令不执行
python3 -m install_deps --dry-run
```

### 4.2 APT 包分类

| 类别 | 示例包 | 用途 |
|------|--------|------|
| 构建工具 | `cmake`, `ninja-build`, `git`, `pkg-config` | 编译基础设施 |
| Python | `python3-pip`, `python3-dev`, `sphinx` | 脚本与文档 |
| CMake 库 | `libeigen3-dev`, `liblua5.3-dev`, `libprotobuf-dev`, `libyaml-cpp-dev` | `find_package` |
| 数值/稀疏 | `libsuitesparse-dev`, `libblas-dev`, `liblapack-dev` | Ceres 等 |
| 测试 | `libgtest-dev`, `libgmock-dev` | 单元测试 |
| 多媒体/GUI | `libopencv` 相关 dev 包（部分由脚本编译） | 视觉 / 仿真 |

完整列表以脚本内 `APT_PACKAGES` 为准（约 50+ 项）。

### 4.3 第三方安装脚本

按顺序执行 `docker/install/` 下脚本，默认安装到 **`/usr/local`**：

| 顺序 | 脚本 | 库 |
|------|------|-----|
| 1 | `install_gtest.sh` | Google Test |
| 2 | `install_glog.sh` | glog |
| 3 | `install_gflags.sh` | gflags |
| 4 | `install_grpc.sh` | gRPC |
| 5 | `install_gperftools.sh` | tcmalloc |
| 6 | `install_opencv.sh` | OpenCV |
| 7 | `install_ceres_solver.sh` | Ceres |
| 8 | `install_nlohmann.sh` | nlohmann/json |
| 9 | `install_osqp.sh` | OSQP |
| 10 | `install_behaviortree_cpp.sh` | **BehaviorTree.CPP 4.x**（Navigator 必需） |
| 11+ | `install_python_modules.sh`, `install_assimp.sh`, `install_ogre.sh`, `install_adolc.sh`, `install_ipopt.sh` | 可选组件依赖 |

> **注意**：BehaviorTree.CPP 为行为树导航所必需；若跳过，Navigator BT 模式将无法加载插件。

### 4.4 安装路径约定

```
/usr/local/
├── include/     # 头文件
├── lib/         # .so / .a
└── bin/         # 可执行工具
```

CMake 通过 `CMAKE_PREFIX_PATH` 或默认搜索路径找到上述库。

### 4.5 可选依赖

| 组件 | 安装方式 | CMake 选项 |
|------|----------|------------|
| gRPC Bridge | `install_grpc.sh` | `BUILD_GRPC=ON`（默认） |
| ONNX Runtime | `install_onnixruntime.sh` | `BUILD_ONNXRUNTIME=ON` |
| Habitat 仿真 | `install_habitat.sh` | 仿真模块 |
| ROS 2 Humble | `install_ros2.sh` | Docker 镜像内可选 |

### 4.6 板端依赖（`--profile board`）

aarch64 板（Firefly 等）请使用：

```bash
python3 scripts/install_dependencies.py --profile board --skip-installed
```

对齐 `docker/dockerfile/autonomy.aarch64.dockerfile`：不装 GUI/Sphinx；glog/protobuf/ceres/grpc 等强制 `docker/install` → `/usr/local`。  
NFS 同步源码 + 板端编译完整流程见 [§9 嵌入式板端](09_embedded_board.md)。

### 4.7 验证依赖

```bash
# Ceres
ls /usr/local/lib/libceres.so 2>/dev/null || ls /usr/lib/x86_64-linux-gnu/libceres.so

# OSQP（common MPC 默认依赖；缺则 cmake 报 Could NOT find OSQP）
ls /usr/local/lib/libosqp.so /usr/local/include/osqp/osqp.h
# 若装到用户前缀：$HOME/.local/lib/libosqp.so

# BehaviorTree.CPP
ls /usr/local/lib/libbehaviortree_cpp.so

# Protobuf（板端应为 3.19.x）
/usr/local/bin/protoc --version 2>/dev/null || protoc --version
```

> **OSQP 未找到**：先执行 `bash docker/install/install_osqp.sh`（或
> `python3 -m install_deps --resume-from install_osqp.sh`），再保证
> `CMAKE_PREFIX_PATH` 覆盖安装前缀（`/usr/local` 或 `$HOME/.local`）。
> `FindOSQP.cmake` 会搜索这两处；仅 `-DCMAKE_PREFIX_PATH=/usr/local` 时
> 若库只在 `~/.local` 仍可能失败，请重装到 `/usr/local` 或把
> `$HOME/.local` 一并加入 `CMAKE_PREFIX_PATH`。

### 4.8 相关文档

- [§6 编译构建](06_build.md)
- [§8 故障排查 · 依赖](08_troubleshooting.md)
- [§9 嵌入式板端](09_embedded_board.md)
