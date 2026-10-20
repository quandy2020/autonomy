# 9. 其它仓库包

这份 Sphinx 主要写 `autonomy/` 业务域。旁边几个顶层包有自己的文档树：

- `autoviz/` — 可视化，见 [Autoviz](../13_Visualization/09_autoviz.md) 和 `autoviz/docs/`
- `autodriver/` — 驱动 HAL，`autodriver/docs/`
- `automanip/` — 机械臂 / bolza，`automanip/docs/`
- `autosim/` — 仿真，见 [AutoSim](../12_Simulation/06_autosim.md)
- `benchmark/` — 规划等 Python 基准，仓库根目录

开关在根 `CMakeLists.txt`（`BUILD_AUTOVIZ`、`BUILD_AUTODRIVER` 等）。目录怎么切见 [仓库说明](../01_Instructions/04_repository.md)。
