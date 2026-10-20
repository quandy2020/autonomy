# 0. Visualization 指南

## 背景

栈在跑却「看不见」——仓库里 **`autonomy/visualization` 的 VisualizationServer 还没落地**，不要找一个域内 GUI 进程。实际有三条成熟路径：本机 Autoviz、远程 Foxglove、已有 ROS 时的 RViz2。

## 思路

建议优先级：

1. **[Autoviz](09_autoviz.md)** — Autolink 原生桌面（Qt + Ogre），不依赖 ROS  
2. **[Foxglove](06_foxglove.md)** — 浏览器 / WS，适合远程  
3. **[RViz2](07_rviz2_ros2.md)** — 已有 ROS 2 工具链时  

- 消息仍是 **commsgs**；显示端再桥到 ROS 或 Autoviz。  
- **Autoviz** 是顶层包（`BUILD_AUTOVIZ`），详细文档在 `autoviz/docs/`。  
- Docker 映射 **8765** 给 Foxglove 时，对照 [Running · Docker](../04_Running/05_docker_runtime.md)。


## 步骤

### 1. 按问题选页

| 文档 | 解决什么 |
|------|----------|
| [01 概览](01_overview.md) | 三条路径怎么选 |
| [02 快速开始](02_quickstart.md) | 最短看到画面 |
| [03 数据类型](03_data_types.md) | 地图/路径/标记等 |
| [04 配置](04_configuration.md) | 可视化相关 conf |
| [05 架构](05_architecture.md) | 数据到显示端 |
| [06 Foxglove](06_foxglove.md) | 8765、bridge |
| [07 RViz2](07_rviz2_ros2.md) | ROS 侧显示 |
| [08 综述](08_survey.md) | 对照 |
| [09 Autoviz](09_autoviz.md) | 构建与使用 |

**本机桌面、无 ROS** → [09 Autoviz](09_autoviz.md)。  
**远程网页** → [06 Foxglove](06_foxglove.md)。  
**RViz 习惯** → [07](07_rviz2_ros2.md) + [Running · ROS 2](../04_Running/06_ros2_integration.md)。

### 2. 常见坑

| 现象 | 先查 |
|------|------|
| 没有可视化进程 | 是否在找未实现的 VisualizationServer |
| Foxglove 连不上 | 端口映射；bridge 是否起 |
| Autoviz 编不过 | Qt6 / Ogre；见包内 README |
