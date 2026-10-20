(running-overview)=
# 1. 运行概览

编过之后怎么起栈、有哪些进程、配置从哪读。最短命令在 [02 快速运行](02_quickstart.md)。

用一张职责表建立索引，细节跳到专题页，不在概览里展开命令序列。

主路径是 **Autolink 多进程**：planning / control / task / bridge… 分开跑，用 launch 拉齐。发令走 Bridge 或 Action，不依赖 ROS 2。

## 怎么起

| 方式 | 入口 | 什么时候用 |
|------|------|------------|
| 全栈 | `autolink_launch autonomy.launch` | 日常联调 |
| 探索栈 | `autolink_launch exploration.launch` | 感知+导航闭环实验（与全栈勿并行） |
| 仅导航三件套 | `autolink launch start task.launch` | 冒烟、Autoviz 点目标 |
| Docker | `docker/run_autonomy.py` | 隔离环境、挂数据集 |
| ROS 2（可选） | `autonomy_ros` launch | 必须接 RViz2 / Gazebo / ros2 CLI |
| 单模块工具 | `planning_test` / `controller_test` 等 | 只盯一个域 |

**不要**同时开 `autonomy.launch`、`exploration.launch` 与 `task.launch`（职责重叠）。

## 常见进程

| 进程 | 职责 |
|------|------|
| `autonomy.planning` | 全局规划 + costmap |
| `autonomy.control` | 局部跟踪、`cmd_vel` |
| `autonomy.task` | BT / 任务 |
| `autonomy.perception` | 感知（可裁剪） |
| `autonomy.bridge` | gRPC |
| `autonomy.monitor` | 健康监控 |
| `autonomy.foxglove_bridge` | Foxglove（可选） |

依赖关系与只起子集：见 [03](03_autonomy_process.md)。

## 配置从哪找

```bash
# 典型：planning / control / perception 共享 AutonomyOptions
--conf=autonomy.pb.txt
# 搜索顺序大致：AUTONOMY_CONF_PATH → AUTONOMY_PATH → install/源码 conf/
```

环境变量全表：[04](04_environment.md)。起来后怎么验：[07](07_verification.md)。
