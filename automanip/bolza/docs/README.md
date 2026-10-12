# Bolza 文档

- 库：`libbolza`（Bolza 最优控制 / MPC）
- 源码：`automanip/bolza/` · 示例：`automanip/examples/`
- 命名空间：`bolza` · 纯 CMake · **不依赖 autonomy / autocmake / autolink**
- 算法源自 [OCS2](https://leggedrobotics.github.io/ocs2/overview.html)；SQP·IPM 二次子问题对齐 HPIPM
- C++ 注释：Doxygen（规范 `.cursor/rules/bolza/doxygen.mdc`；生成 `doxygen Doxyfile`）
- 公式：`$...$` / `$$...$$`
- 包说明：[../README.md](../README.md) · 示例话题：[../../examples/README.md](../../examples/README.md)

## 目录

1. [01_overview.md](01_overview.md) — 总览
2. [02_installation.md](02_installation.md) — 安装
3. [03_getting_started.md](03_getting_started.md) — 入门
4. [04_robotic_examples.md](04_robotic_examples.md) — **教程（§4.1 libbolza step-by-step）** + 机器人示例
5. [05_optimal_control_modules.md](05_optimal_control_modules.md) — 最优控制模块
6. [06_configuration.md](06_configuration.md) — 配置
7. [07_solvers.md](07_solvers.md) — 求解器
8. [08_from_urdf_to_ocp.md](08_from_urdf_to_ocp.md) — 由 URDF 建模
9. [09_profiling.md](09_profiling.md) — 性能剖析
10. [10_mpcnet.md](10_mpcnet.md) — MPC-Net
11. [11_architecture.md](11_architecture.md) — 软件结构
12. [12_glossary.md](12_glossary.md) — 术语表
13. [13_faq.md](13_faq.md) — 常见问题

- 建议顺序：02 → 03 → **04 §4.1 教程** → 04 各机器人 → 05–07

## 验证安装

```bash
# 仅 libbolza（推荐）
cmake -S automanip/bolza -B build/bolza
cmake --build build/bolza -j"$(nproc)" --target bolza

# 教程 / demo 可执行文件在 automanip/examples（可选，属上层工程）
cmake --build <umbrella-build> -j"$(nproc)" --target bolza_tutorial_di cartpole_demo
```

- 异常：[13_faq.md](13_faq.md)
