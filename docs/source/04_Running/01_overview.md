(running-overview)=
# 1. 运行概览

## 1.1 运行方式

| 方式 | 入口 | 依赖 ROS 2 | 用途 |
|------|------|------------|------|
| 多进程栈 | `autolink_launch autonomy.launch` | 否 | 规划 / 控制 / 任务 / Bridge |
| 探索闭环 | `autolink_launch exploration.launch` | 否 | perception + 导航三件套 |
| Docker | `docker/run_autonomy.py` | 否 | 隔离环境 |
| ROS 2 | `autonomy_ros` launch（可选） | 是 | RViz2、Gazebo、ros2 CLI |
| 单测工具 | `planning_test` / `controller_test` | 否 | 单模块调试 |

主路径：**autolink 多进程** → Bridge / Action 发令。

## 1.2 进程

| 进程 | 职责 |
|------|------|
| `autonomy.planning` | 全局规划 + costmap |
| `autonomy.control` | 局部跟踪、`cmd_vel` |
| `autonomy.task` | BT / 任务编排 |
| `autonomy.perception` | 感知（可关） |
| `autonomy.bridge` | gRPC / 桥接 |
| `autonomy.monitor` | 健康监控 |
| `autonomy.foxglove_bridge` | Foxglove |

## 1.3 配置

```bash
--conf=autonomy.pb.txt # planning / control / perception 共享 AutonomyOptions
# 搜索：AUTONOMY_CONF_PATH → AUTONOMY_PATH → install/源码 conf/
```
