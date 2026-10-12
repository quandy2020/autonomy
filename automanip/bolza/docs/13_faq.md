# 13　常见问题

## 13.1　构建

**`libbolza skipped`**

- 检查：Eigen3 · OpenMP · Threads · CppAD · Pinocchio · GLPK
- 重配：

```bash
cmake -S automanip/bolza -B build/bolza
# 若找不到 CppAD：安装 CppAD + CppADCodeGen，或设 CMAKE_PREFIX_PATH / CppAD_DIR
```

- 机械臂自碰撞未编入 → coal（`find_package(Coal)`）
- **不依赖** autonomy / autocmake / autolink 即可编 `libbolza`；demo 见 `automanip/examples`（独立工程）
- HPIPM 失败 → 在 bolza 仓库根执行 `git submodule update --init --recursive`；无 submodule 时配置期需网络 `FetchContent`；确认 `BOLZA_WITH_HPIPM=ON`
- SQP/IPM QP 失败 → 开 `printSolverStatus` 看 HPIPM 返回码
- 不引入 ROS / autolink：`libbolza` 无中间件依赖；自碰撞距离段见 `GeometryInterfaceVisualization::ComputeDistances`
- 单测：`-DBOLZA_BUILD_TESTS=ON`（需 GTest）；不含 RaiSim / ROS
- pinocchio URDF 测：默认用树内 `examples/assets`（如 `anymal_c/urdf/anymal.urdf`）；可覆盖 `-DBOLZA_ROBOTIC_ASSETS_ROOT` / `BOLZA_ROBOTIC_ASSETS`
- CppAD 缓存目录默认 `/tmp/bolza`（对齐命名，非 `/tmp/ocs2`）

## 13.2　运行

- 首次很慢 → CppAD 在 `libraryFolder` 编译；删目录可重生
- 线程优先级警告 → 可忽略
- 退出码 2 → 求解器名非法，见 [07_solvers.md](07_solvers.md)
- 无画面 → 查示例工程的可视化配置与 `LD_LIBRARY_PATH`；demo 可空跑
- 跟踪发散 → 增迭代 · 核参考维数 · 缩时域 / 降频 · 开 `displayShortSummary`
- 多 demo 干扰 → 全局话题，一次只跑一个

## 13.3　接口

- 找不到 `bolza/...` → 链 `bolza`；include 根为 `automanip/bolza/`
- 改 OCP 不生效 → 用 ReferenceManager / 同步模块，见 [05_optimal_control_modules.md](05_optimal_control_modules.md)
- 状态–输入等式失败 → 对 $u$ 的 Jacobian 须行满秩，否则改软约束
