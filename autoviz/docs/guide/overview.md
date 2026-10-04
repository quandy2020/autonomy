# 概述

Autoviz（Aviz）是 Autonomy 栈的原生桌面 3D 可视化工具：经 **Autolink** 订阅 channel，在本地 Qt 窗口中渲染传感器、TF、地图与机器人模型。

## 定位

| 维度 | Autoviz | Foxglove Bridge | RViz2 |
|------|---------|-----------------|-------|
| 形态 | 原生桌面（Qt + **Ogre 1.x**） | WebSocket 转发 | ROS 2 桌面 |
| 通信 | Autolink 直连 | Autolink → WebSocket | rclcpp |
| 产物 | `libautoviz` + `bin/autoviz` | 服务进程 | 完整 ROS 2 栈 |
| 平台 | Linux · macOS · Windows | 浏览器 / 服务 | 主要为 Linux |

## 目标

| 目标 | 说明 |
|------|------|
| Autolink 原生 | 不链接、不运行 ROS / rclcpp / rviz |
| 概念对齐 RViz2 | Display / Tool / Panel / ViewController / Fixed Frame |
| 跨平台 | Qt 6；视口固定 **Ogre 1.12**（可 vendor） |
| 可扩展 | `AUTOVIZ_PLUGIN_PATH` 动态加载插件 |

## 非目标

- 替代 Foxglove（远程协作、MCAP Studio）
- 二进制兼容 RViz2 插件（API 不同，仅概念对齐）
- 内置 SLAM / 导航编辑（仅可视化与交互工具）

## 与 ROS 的关系

| 项 | 状态 |
|----|------|
| 编译 / 运行时 ROS | 无 |
| 消息类型字符串 | 兼容 `sensor_msgs/...` 等别名，映射到 automsgs |
| `.rviz` 导入 | 离线 YAML 解析，不依赖 ROS 运行时 |
| TF | 内嵌 `autoviz/transform/`（vendored tf2 BufferCore） |
| `package://` mesh | `AUTOVIZ_RESOURCE_PATH`，不读 ament |

与 ROS 2 互通时使用独立的 `autonomy_ros` 桥接进程；Autoviz 仍只连接 Autolink。

## 相关文档

- [构建](build.md) · [使用](usage.md) · [部署](deployment.md)
- [架构](../architecture/overview.md)
