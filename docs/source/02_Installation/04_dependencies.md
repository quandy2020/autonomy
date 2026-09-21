# 4. 依赖安装

**唯一推荐入口**（与 Dockerfile / CMake 预期对齐）：

```bash
# 在源码根执行
python3 scripts/install_dependencies.py --skip-installed
```

| Profile | 适用 | 说明 |
|---------|------|------|
| `full`（默认） | 桌面 / CI | APT + Dockerfile 第三方 + Assimp/Ogre 等 |
| `board` | aarch64 车端 | 无 GUI/Sphinx；强制关键库进 `/usr/local` |

实现目录 `scripts/install_deps/` 也可：`cd scripts && python3 -m install_deps …`（无 `--profile`）。日常请用上面的 `install_dependencies.py`。

---

### 4.1 常用命令

```bash
# 完整（跳过已检测到的库）
python3 scripts/install_dependencies.py --skip-installed

# 仅 APT / 仅第三方
python3 scripts/install_dependencies.py --apt-only
python3 scripts/install_dependencies.py --thirdparty-only --skip-installed

# 查看本 profile 将装什么
python3 scripts/install_dependencies.py --list --profile full
python3 scripts/install_dependencies.py --list --profile board

# 从某脚本续装
python3 scripts/install_dependencies.py --resume-from install_ceres_solver.sh --skip-installed

# 板端
python3 scripts/install_dependencies.py --profile board --skip-installed

# 预览
python3 scripts/install_dependencies.py --dry-run --profile board
```

单库也可直接跑脚本（最快）：

```bash
bash docker/install/install_osqp.sh
bash docker/install/install_glog.sh
AUTONOMY_MAKE_JOBS=2 bash docker/install/install_ceres_solver.sh   # 内存紧
```

---

### 4.2 第三方脚本顺序（`full` / `board` 共用核心）

默认安装到 **`/usr/local`**：

| 顺序 | 脚本 | 库 |
|------|------|-----|
| 1 | `install_gtest.sh` | Google Test |
| 2 | `install_glog.sh` | glog 0.6 |
| 3 | `install_gflags.sh` | gflags |
| 4 | `install_protobuf.sh` | **Protobuf 3.19** |
| 5 | `install_grpc.sh` | gRPC |
| 6 | `install_gperftools.sh` | tcmalloc |
| 7 | `install_opencv.sh` | OpenCV |
| 8 | `install_ceres_solver.sh` | Ceres |
| 9 | `install_g2o.sh` / `install_fbow.sh` | 定位相关 |
| 10 | `install_nlohmann.sh` | nlohmann/json |
| 11 | `install_osqp.sh` | OSQP（common MPC） |
| 12 | `install_behaviortree_cpp.sh` | BehaviorTree.CPP 4.x |
| … | `install_adolc.sh` / `install_ipopt.sh` 等 | 可选 |
| full 额外 | `install_assimp.sh` / `install_ogre.sh` 等 | 可视化 |

> BehaviorTree.CPP 为 Navigator BT 模式所需。板端会 purge 冲突的 apt 版 glog/protobuf/grpc，避免 ABI 混用。

---

### 4.3 安装路径

```text
/usr/local/include/
/usr/local/lib/
/usr/local/bin/   # 如 protoc
```

CMake：`-DCMAKE_PREFIX_PATH=/usr/local`。不要把 gRPC/protobuf 装在 `~/grpc` 却让 Autonomy 链 `/usr/local` 的另一套 protobuf。

---

### 4.4 可选能力

| 组件 | 安装 | CMake |
|------|------|-------|
| gRPC Bridge | `install_grpc.sh`（依赖脚本已含） | `BUILD_GRPC=ON`（默认） |
| Fast DDS | `install_fastdds.sh`（钉 v3.6.2） | Autolink `AUTOLINK_ENABLE_FASTDDS` |
| ONNX | 见 docker/install 中 ONNX 脚本 | `BUILD_ONNXRUNTIME=ON` |
| ROS 2 Humble | Docker 镜像内可选 | 非构建硬依赖 |

---

### 4.5 验证

```bash
ls /usr/local/lib/libceres.so
ls /usr/local/lib/libosqp.so /usr/local/include/osqp/osqp.h
ls /usr/local/lib/libbehaviortree_cpp.so
ls /usr/local/lib/libglog.so
/usr/local/bin/protoc --version    # 板端期望 3.19.x
```

OSQP 找不到：先 `bash docker/install/install_osqp.sh`，保证前缀是 `/usr/local`（不要只在 `~/.local`）。

板端完整流程：[§9](09_embedded_board.md)。排错：[§8](08_troubleshooting.md)。
