# 9. 其它仓库包

本树主要写 `autonomy/` 域。旁边顶层包自带文档：
| 包 | 用途 | 文档 |
|----|------|------|
| `autoviz/` | Autolink 原生 3D 可视化 | [13 · Autoviz](../13_Visualization/09_autoviz.md)、`autoviz/docs/` |
| `autodriver/` | 驱动 HAL | `autodriver/docs/` |
| `automanip/` | 机械臂 / bolza | `automanip/docs/` |
| `autosim/` | 仿真 | [12 · AutoSim](../12_Simulation/06_autosim.md) |
| `benchmark/` | Python 基准 | 仓库根 |

**什么时候看：** 主 Sphinx 树没写到的 autoviz、驱动、仿真包。

### CMake 开关（根 `CMakeLists.txt`）

| 开关 | 典型用途 |
|------|----------|
| `BUILD_AUTOVIZ` | 桌面 3D；板端常 OFF |
| `BUILD_AUTODRIVER` | 驱动栈 |
| `BUILD_GRPC` | Bridge（与可视化无关） |
| `BUILD_AUTOSIM` / `BUILD_ORBISVIEW` | 仿真 / Web HMI |

目录与域关系：[仓库说明](../01_Instructions/04_repository.md)。

### 常见坑

| 现象 | 说明 |
|------|------|
| 找不到 autoviz 二进制 | `BUILD_AUTOVIZ=OFF` 或未装 Qt6 |
| 以为 autoviz 走 Bridge | 可视化走 Autolink topic，不是 gRPC 命令面 |
