(running-overview)=
# 1. 运行概览

### 1.1 运行方式一览

| 方式 | 入口 | 依赖 ROS 2 | 典型用途 |
|------|------|------------|----------|
| **离线导航测试** | `autonomy_nav_test` | 否 | 端到端验证规划+控制+BT |
| **Autonomy 常驻进程** | `system/main.cpp` 对应二进制 | 否 | 加载配置、等待上层发令 |
| **Docker 容器** | `docker/run_autonomy.py` | 否 | 隔离开发环境 |
| **ROS 2 集成** | `autonomy_ros` launch（可选） | 是 | RViz2、Gazebo、ros2 CLI |
| **模块单测工具** | `planning_test` / `controller_test` | 否 | 单模块调试 |

### 1.2 推荐路径

当前 **`libautonomy` 主路径不依赖 ROS 2**。新用户建议：

```
02 Installation 编译
    → autonomy_nav_test（§4）
    → 确认 Navigation succeeded
    → 再按需接入 ROS 2 / Bridge
```

### 1.3 运行时组件

`system::Autonomy` 启动后持有：

| 组件 | 职责 |
|------|------|
| `MapServer` | 加载静态地图 → costmap |
| `PlannerServer` | 全局路径规划 |
| `ControllerServer` | 局部跟踪、里程计 |
| `TransformServer` / `tf_buffer` | 坐标变换 |
| Navigator（BT） | 任务编排（若 `Configure` 启用） |

### 1.4 配置入口

```bash
--configuration_directory=config
--configuration_basename=autonomy.lua
```

主配置 `config/autonomy.lua` 聚合 map / planning / controller / navigator 等子配置。

### 1.5 相关文档

- [§2 快速运行](02_quickstart.md)
- [02 Installation](../02_Installation/00_guide.md)
