# 6. 编译构建

依赖就绪后，在本机 / Docker / 板端用 **同一套 CMake 开关** 编译。路径差异见文末；板端 NFS 细节见 [§9](09_embedded_board.md)。

| 目标 | 章节 |
|------|------|
| 全量编译 | [§6.1](#61-全量编译) |
| 只编部分域 / 产品 | [§6.2](#62-模块化编译) |
| 已配置工程只编某 target | [§6.3](#63-按-target-增量编译) |
| 测试 / 文档 / 清理 | [§6.4](#64-测试文档与清理) |

---

## 6.1 全量编译

在 **源码根**（含顶层 `CMakeLists.txt`）：

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

无 Ninja：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/usr/local
cmake --build build -j$(nproc)
```

默认多数域与 `BUILD_AUTODRIVER` / `BUILD_AUTOVIZ` 等为 ON。验证：

```bash
ls build/lib/libautonomy_common.so build/lib/libautolink.so
ls build/bin/autolink build/bin/autonomy.* 2>/dev/null | head
```

可选：`sudo cmake --install build --prefix /usr/local`。

---

## 6.2 模块化编译

Autonomy 是超工程：

1. **始终构建**：`autolink`、`automsgs`  
2. **业务域**：`autonomy/<域>/`，由 `AUTONOMY_BUILD_<域>` 控制（默认全 ON）  
3. **产品子工程**：`BUILD_AUTODRIVER` / `BUILD_AUTOVIZ` / `BUILD_GRPC` 等，与域开关正交  

定义见 [`cmake/autonomy_options.cmake`](../../../cmake/autonomy_options.cmake)、顶层 [`CMakeLists.txt`](../../../CMakeLists.txt)。

### 6.2.1 两层开关（勿混用）

| 层级 | 前缀 | 例子 | 作用 |
|------|------|------|------|
| 产品 / 子工程 | `BUILD_*` | `BUILD_GRPC`、`BUILD_AUTODRIVER` | 可选能力与独立目录 |
| 业务域 | `AUTONOMY_BUILD_*` | `AUTONOMY_BUILD_PLANNING` | 是否编译对应域 |

关掉某域后，若仍开启依赖它的域，**configure 阶段 FATAL**。

### 6.2.2 域依赖与主要目标

| 域开关 | 硬依赖 | 主要目标 |
|--------|--------|----------|
| `AUTONOMY_BUILD_COMMON` | — | `autonomy_common` |
| `AUTONOMY_BUILD_TRANSFORM` | COMMON | `autonomy_transform` |
| `AUTONOMY_BUILD_MAP` | COMMON, TRANSFORM | `autonomy_map` |
| `AUTONOMY_BUILD_VEHICLE` | COMMON | `autonomy_vehicle` |
| `AUTONOMY_BUILD_MANIPULATION` | COMMON, TRANSFORM | `autonomy_manipulation` |
| `AUTONOMY_BUILD_PREDICTION` | COMMON | `autonomy_prediction` |
| `AUTONOMY_BUILD_CONTROL` | COMMON, TRANSFORM, MAP | `autonomy_control`、`autonomy.control` |
| `AUTONOMY_BUILD_PLANNING` | COMMON, TRANSFORM, MAP | `autonomy_planning`、`autonomy.planning` |
| `AUTONOMY_BUILD_PERCEPTION` | COMMON, TRANSFORM, MAP | `autonomy_perception`、`autonomy.perception`、`follow_component` |
| `AUTONOMY_BUILD_LOCALIZATION` | COMMON, TRANSFORM | `autonomy_localization`、`autonomy.localization` |
| `AUTONOMY_BUILD_SENSOR` | COMMON, CONTROL | `autonomy_sensor` |
| `AUTONOMY_BUILD_TASK` | COMMON, TRANSFORM, MAP, CONTROL | `autonomy_task`、`autonomy.task` |
| `AUTONOMY_BUILD_SYSTEM` | COMMON, TASK | `autonomy_system`、`autonomy.monitor`、`autonomy.ota` |
| `AUTONOMY_BUILD_AUDIO` | COMMON | `autonomy_audio` |
| `AUTONOMY_BUILD_BRIDGE` | COMMON, SYSTEM, TASK + `BUILD_GRPC=ON` | `autonomy_bridge`、`autonomy.bridge` |
| `AUTONOMY_BUILD_VISUALIZATION` | MAP + Foxglove | （可视化） |

| 产品开关 | 目录 / 说明 |
|----------|-------------|
| `BUILD_AUTODRIVER` | `autodriver/` → `autodriver`、`autodriver_main` |
| `BUILD_AUTOVIZ` / `BUILD_AUTOSIM` / `BUILD_ORBISVIEW` | 可视化 / 仿真 / Web HMI |
| `BUILD_ONNXRUNTIME` | 感知 `base_component`（与 PERCEPTION 域独立） |

### 6.2.3 场景示例

未列出的 `AUTONOMY_BUILD_*` 建议显式 OFF，避免默认 ON 把多余域编进来。

#### A. 导航（planning + control + task）

```bash
cd build
cmake -G Ninja .. \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/usr/local \
  -DBUILD_TEST=OFF -DBUILD_DOCS=OFF \
  -DBUILD_AUTOVIZ=OFF -DBUILD_AUTOSIM=OFF -DBUILD_ORBISVIEW=OFF \
  -DBUILD_AUTODRIVER=OFF -DBUILD_GRPC=OFF \
  -DAUTONOMY_BUILD_COMMON=ON -DAUTONOMY_BUILD_TRANSFORM=ON \
  -DAUTONOMY_BUILD_MAP=ON -DAUTONOMY_BUILD_CONTROL=ON \
  -DAUTONOMY_BUILD_PLANNING=ON -DAUTONOMY_BUILD_TASK=ON \
  -DAUTONOMY_BUILD_VEHICLE=OFF -DAUTONOMY_BUILD_MANIPULATION=OFF \
  -DAUTONOMY_BUILD_PREDICTION=OFF -DAUTONOMY_BUILD_PERCEPTION=OFF \
  -DAUTONOMY_BUILD_LOCALIZATION=OFF -DAUTONOMY_BUILD_SENSOR=OFF \
  -DAUTONOMY_BUILD_SYSTEM=OFF -DAUTONOMY_BUILD_AUDIO=OFF \
  -DAUTONOMY_BUILD_BRIDGE=OFF -DAUTONOMY_BUILD_VISUALIZATION=OFF
ninja -j$(nproc)
```

#### B. 感知 + Autodriver

```bash
cmake -G Ninja .. \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/usr/local \
  -DBUILD_TEST=OFF -DBUILD_DOCS=OFF \
  -DBUILD_AUTOVIZ=OFF -DBUILD_AUTOSIM=OFF -DBUILD_ORBISVIEW=OFF \
  -DBUILD_GRPC=OFF -DBUILD_AUTODRIVER=ON -DBUILD_ONNXRUNTIME=ON \
  -DAUTONOMY_BUILD_COMMON=ON -DAUTONOMY_BUILD_TRANSFORM=ON \
  -DAUTONOMY_BUILD_MAP=ON -DAUTONOMY_BUILD_PERCEPTION=ON \
  -DAUTONOMY_BUILD_VEHICLE=OFF -DAUTONOMY_BUILD_MANIPULATION=OFF \
  -DAUTONOMY_BUILD_PREDICTION=OFF -DAUTONOMY_BUILD_CONTROL=OFF \
  -DAUTONOMY_BUILD_PLANNING=OFF -DAUTONOMY_BUILD_LOCALIZATION=OFF \
  -DAUTONOMY_BUILD_SENSOR=OFF -DAUTONOMY_BUILD_TASK=OFF \
  -DAUTONOMY_BUILD_SYSTEM=OFF -DAUTONOMY_BUILD_AUDIO=OFF \
  -DAUTONOMY_BUILD_BRIDGE=OFF -DAUTONOMY_BUILD_VISUALIZATION=OFF
ninja -j$(nproc)
```

#### C. 管理面（system + bridge）

打开：COMMON、TRANSFORM、MAP、CONTROL、TASK、SYSTEM、BRIDGE，以及 `BUILD_GRPC=ON`（task 依赖 control/map，不能只开 system）。

#### D. 最小骨架（调试 CMake）

```bash
cmake --preset autonomy-minimal
cmake --build --preset autonomy-minimal
```

见 [`CMakePresets.json`](../../../CMakePresets.json)。

### 6.2.4 产品级开关速查

| 选项 | 默认 | 说明 |
|------|------|------|
| `BUILD_GRPC` | ON | Bridge；关则 BRIDGE 域不可用 |
| `BUILD_TEST` | OFF | 单测 |
| `BUILD_DOCS` | ON | Sphinx |
| `BUILD_AUTODRIVER` | ON | 传感器 / 底盘 |
| `BUILD_AUTOVIZ` | ON | 3D 可视化 |
| `BUILD_AUTOSIM` | OFF | 仿真 |
| `BUILD_ORBISVIEW` | ON | Web HMI |
| `BUILD_ONNXRUNTIME` | ON | 感知 Base |
| `BUILD_TENSORRT` | OFF | TensorRT |

---

## 6.3 按 target 增量编译

改过 `AUTONOMY_BUILD_*` / `BUILD_*` 后须重新 `cmake`。开关未变时：

```bash
cd build
cmake --build . -j$(nproc) --target autonomy.planning
cmake --build . -j$(nproc) --target autonomy.control autonomy.task
cmake --build . -j$(nproc) --target autonomy.perception follow_component
cmake --build . -j$(nproc) --target autodriver autodriver_main
cmake --build . -j$(nproc) --target autonomy.monitor autonomy.bridge
```

---

## 6.4 测试、文档与清理

```bash
# 测试
cmake -S . -B build -DBUILD_TEST=ON -DCMAKE_PREFIX_PATH=/usr/local
cmake --build build -j$(nproc)
cd build && ctest --output-on-failure

# 文档
cd docs && pip install -r requirements.txt && sphinx-build -b html source build

# 清理产物（保留 cache 时可只 ninja clean）
cd build && ninja clean
# 彻底重配：rm -rf build && 重新 cmake
```

---

## 6.5 本机 / Docker / 板端路径差异

| 环境 | 源码 | build 目录 | 依赖 |
|------|------|------------|------|
| 本机 | 源码根 | `源码根/build` | `install_dependencies.py` |
| Docker | 常为 `/workspace/autonomy` | 容器内 `build/` | 镜像预装或同脚本 |
| 板端 | NFS 如 `~/autonomy` | **必须本地盘**，如 `~/autonomy_ws/build` | `--profile board` |

板端示例：

```bash
cmake -S ~/autonomy -B ~/autonomy_ws/build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/usr/local \
  -DBUILD_AUTOVIZ=OFF -DBUILD_ORBISVIEW=OFF -DBUILD_DOCS=OFF -DBUILD_TEST=OFF
cmake --build ~/autonomy_ws/build -j4
```

交叉：`cmake --preset jdr-board`（需 sysroot / toolchain）。

---

相关：[§4 依赖](04_dependencies.md) · [§5 Docker](05_docker.md) · [§7 环境](07_environment.md) · [§9 板端](09_embedded_board.md) · [`cmake/README.md`](../../../cmake/README.md)
