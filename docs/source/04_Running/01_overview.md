(running-overview)=
# 1. 运行概览

### 1.1 运行方式一览

| 方式 | 入口 | 依赖 ROS 2 | 典型用途 |
|------|------|------------|----------|
| **多进程栈** | `autolink_launch autonomy.launch` | 否 | 规划 / 控制 / 任务 / Bridge 等独立进程 |
| **探索闭环** | `autolink_launch exploration.launch` | 否 | perception + planning/control/task |
| **Docker 容器** | `docker/run_autonomy.py` | 否 | 隔离开发环境 |
| **ROS 2 集成** | `autonomy_ros` launch（可选） | 是 | RViz2、Gazebo、ros2 CLI |
| **模块单测工具** | `planning_test` / `controller_test` | 否 | 单模块调试 |

### 1.2 推荐路径

当前主路径为 **autolink 多进程**，各模块通过 topic / service / action 通信。新用户建议：

```
02 Installation 编译
    → autolink_launch autonomy.launch（§2 / §3）
    → Bridge 或 Action Client 发令
    → 再按需接入 ROS 2 / 可视化
```

### 1.3 运行时组件（多进程）

| 进程 | 职责 |
|------|------|
| `autonomy.planning` | 全局路径规划 + costmap |
| `autonomy.control` | 局部跟踪、cmd_vel |
| `autonomy.task` | BT / 导航任务编排 |
| `autonomy.perception` | 感知（默认可关闭） |
| `autonomy.bridge` | 外部 gRPC / 桥接 |
| `autonomy.monitor` | 系统健康监控 |
| `autonomy.foxglove_bridge` | Foxglove 可视化桥 |

### 1.4 配置入口

```bash
--conf=autonomy.pb.txt   # planning / control / perception 共享 AutonomyOptions 快照
# 模块本地：autonomy/<mod>/conf/*.pb.txt
# 搜索：AUTONOMY_CONF_PATH → AUTONOMY_PATH → install/源码 conf/
```

详见各模块 `conf/` 与 [§3 多进程栈](03_autonomy_process.md)。
