# 2　安装

`libbolza` 为**独立 CMake 包**：不依赖 `autonomy_*`、autocmake、autolink、ROS。  
可选地由 `automanip` 以 `add_subdirectory(bolza)` 一并编入，那是工程组织，不是库依赖。

## 2.1　依赖

必选：

- Eigen3
- OpenMP、Threads
- CppAD/Cg（`find_package(CppAD)`）
- Pinocchio（`find_package(Pinocchio)`）
- GLPK（`find_package(GLPK)`；Frank–Wolfe / bilevel）
- BLASFEO（配置期 `FetchContent`）+ HPIPM（优先 `thirdparty/hpipm` submodule，否则 `FetchContent`）
- C++17（`std::filesystem`；自研 INFO `property_tree` / iostream 日志 / `RungeKuttaDormandPrince5`；**无 Boost**）

可选：

- coal — 机械臂自碰撞（`find_package(Coal)`）
- ONNX Runtime — MPC-Net 推理

> 可视化 / 话题 demo（`automanip/examples`）是**另一工程**，可另链 autolink；与 `libbolza` 解耦。

CppAD：`find_package(CppAD)`（`cmake/modules/FindCppAD.cmake`，查找 `cppad/cg.hpp`）。缺依赖时日志：`bolza: skipped`。

```bash
# 在 bolza 仓库根拉取第三方（hpipm / coal；BLASFEO 由 CMake FetchContent）
git submodule update --init --recursive
```

说明见 [`thirdparty/README.md`](../thirdparty/README.md)。

## 2.2　工程组织（对齐 cartographer）

| 路径 | 作用 |
|:---|:---|
| `CMakeLists.txt` | 源码 glob、`add_library(bolza)`、`bolza_test`、install |
| `cmake/functions.cmake` | `bolza_initialize_project` / `bolza_test` / `bolza_binary` |
| `cmake/modules/` | `FindCppAD` · `FindPinocchio` · `FindCoal` · `FindONNXRuntime` · `FindGLPK` · `FindGMock` |
| `bolza-config.cmake.in` | `find_package(bolza)` |

## 2.3　CMake 选项

| 选项 | 默认 | 作用 |
|:---|:---|:---|
| `BOLZA_WITH_ONNX` | ON | 找到 SDK 则编入 ONNX |
| `BOLZA_BUILD_TESTS` | OFF | 注册 `bolza/**/*_test.cpp` GTest 目标 |
| `BOLZA_ROBOTIC_ASSETS_ROOT` | `bolza/robotic_assets` | 单测 URDF 根（含 `resources/`） |

- 可选宏：`BOLZA_HAS_COAL` · `BOLZA_HAS_ONNX`

## 2.4　构建（推荐：仅库）

```bash
cmake -S automanip/bolza -B build/bolza \
  -DCMAKE_PREFIX_PATH=/path/to/cppad/prefix
cmake --build build/bolza -j"$(nproc)" --target bolza
```

安装后可由其它工程 `find_package(bolza)` / `target_link_libraries(… bolza)`。

## 2.5　与 automanip 一并构建（可选）

若需要 `bolza_tutorial_di` / `*_demo`（在 `automanip/examples`），在上层 umbrella 中配置后：

```bash
cmake --build <umbrella-build> -j"$(nproc)" --target bolza bolza_tutorial_di cartpole_demo
```

- 独立构建只产出 `libbolza`；demo / 教程可执行文件属 `automanip/examples`，不是 `libbolza` 的依赖。

## 2.6　自检

- 产物：`libbolza`（及可选的 `include/bolza/` install）
- 首次 CppAD codegen 较慢（如 `/tmp/bolza/...`）属正常
- install：`include/bolza/` · `lib/` · `share/bolza/solver_license.txt`
