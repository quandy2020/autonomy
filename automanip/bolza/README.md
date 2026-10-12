# Bolza

Bolza 形式最优控制库（`libbolza`）。**独立 CMake 包**，不依赖 `autonomy_*` / autocmake / autolink。  
可对本目录 `cmake -S`，也可被 [`automanip`](../) `add_subdirectory`（仅为工程组织）。

> 连续时间 Bolza 问题 + 多重射击 / DDP 族求解器；算法源自 OCS2（BSD），见 `solver_license.txt`。

**文档**：[docs/README.md](docs/README.md)（`01`–`13` 扁平编号）

## 布局

```
automanip/bolza/
├── bolza/                 # 源码 + 并列 *_test（cartographer 式）
│   ├── core/ oc/ mpc/ …
│   └── robotic_assets/
├── cmake/
│   ├── functions.cmake    # bolza_initialize_project / bolza_test / …
│   ├── modules/           # FindCppAD · Pinocchio · Coal · ONNXRuntime · GLPK · GMock
│   ├── bolza_hpipm.cmake
│   └── bolza_version.cmake
├── bolza-config.cmake.in  # find_package(bolza)
├── config/conf.hpp.in
├── docs/
├── package.xml
└── CMakeLists.txt

```

Include：`#include <bolza/core/types.hpp>` · 命名空间 `bolza` · 版本头 `#include <bolza/conf/conf.hpp>`。

`libbolza` **不依赖** autonomy / autolink / ROS；`*_demo` 在 `automanip/examples`（独立工程）中构建。

## 模块

| 目录 | 内容 |
|------|------|
| `core` | Types、cost/constraint、dynamics、AD、penalties、loopshaping |
| `oc` | OptimalControlProblem、rollout、multiple shooting、KKT |
| `mpc` | MpcBase、MRT、settings |
| `ddp` | GaussNewtonDDP、SLQ、iLQR、Riccati |
| `sqp` / `ipm` / `slp` | 多重射击非线性求解；`ipm` 含 `HpipmInterface`（BLASFEO+HPIPM） |
| `frank_wolfe` / `bilevel` | Frank–Wolfe NLP + GDDP/OCS2 双层（需 GLPK） |
| `switched_model` | 切换系统接口（无 ROS；自 `ocs2_switched_model_interface`） |
| `qp` | 离散转录与 QP |
| `pinocchio` | FK/动力学/自碰撞；`GeometryInterfaceVisualization` 为 ROS-free 距离段 |
| `mpcnet` | 数据生成 / ONNX 推理（无 Python 封装） |

感知足式示例（无 ROS）：`automanip/examples/perceptive_anymal/`。

## 依赖

| 级别 | 包 |
|------|-----|
| 必选 | Eigen3、OpenMP、Threads、CppAD/Cg、Pinocchio、GLPK、BLASFEO（FetchContent）+ HPIPM（submodule / FetchContent） |
| 可选 | coal、ONNX Runtime |

CppAD：`find_package(CppAD)`（`cmake/modules/FindCppAD.cmake`，需 `<cppad/cg.hpp>`）。

## 选项

| CMake 选项 | 默认 | 说明 |
|------------|------|------|
| `BOLZA_WITH_ONNX` | ON | 找到 onnxruntime 时编入 ONNX 控制器 |
| `BOLZA_BUILD_TESTS` | OFF | 注册 `bolza/**/*_test.cpp`（`bolza_test` / CTest） |

## 构建

```bash
cmake -S automanip/bolza -B build/bolza \
  -DCMAKE_PREFIX_PATH=/path/to/cppad/prefix
cmake --build build/bolza -j"$(nproc)" --target bolza
```

需要 demo / 教程可执行文件时，在上层 `automanip` umbrella 中另编 `bolza_tutorial_di` / `*_demo`（不链入 `libbolza`）。

## 许可证

- 包元数据 / 本仓库新增文件：Apache-2.0
- `bolza/` 算法源码：大量保留上游 **BSD-3-Clause**（Farshidian / OCS2），全文见 `solver_license.txt`
- `bolza/ipm/hpipm_interface*`：`HpipmInterface`；BLASFEO/HPIPM 为上游 BSD-2-Clause
