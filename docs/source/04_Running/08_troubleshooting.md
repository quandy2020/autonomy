(running-troubleshooting)=
# 8. 故障排查

## 背景

运行期：缺库、BT 加载失败、有 Path 无 `cmd_vel`、Docker 挂载等。安装阶段问题见 [Installation §8](../02_Installation/08_troubleshooting.md)。板端资源对照 [§9](09_board_task_launch_benchmark.md)。

## 思路

先判断阶段（配置/编译/运行/网络），再查表；根因多为路径、前缀或开关不一致。


## 步骤

![运行示意](./images/running.png)

### 1. 可执行文件

| 现象 | 处理 |
|------|------|
| 找不到 `autonomy.planning` 等 | `ninja -C build`；检查模块 CMake |
| 找不到 `autolink_launch` | `PATH` 含 `build/bin`；或 `source setup` |
| `.so: cannot open` | `LD_LIBRARY_PATH` 含 `build/lib` 与 `/usr/local/lib` |

### 2. 导航 / BT

| 现象 | 处理 |
|------|------|
| BT 插件 load 失败 | `export AUTONOMY_BT_PLUGIN_PATH=build/lib` |
| `GetPlan failed` | 起终点在障碍上；调坐标 |
| `TransformAvailable` 失败 | 统一 `global_frame` / `base_frame` |
| `no robot pose` | 仿真 / Bridge 注入 odom+TF |

### 3. Docker

| 现象 | 处理 |
|------|------|
| 挂载为空 | `AUTONOMY_ENV` 指向源码根 |
| 数据盘不可写 | `--data-volume` |
| `docker exec` 失败 | `docker ps`；检查容器名 |
| GPU 不可用 | NVIDIA Container Toolkit |

### 4. ROS 2

| 现象 | 处理 |
|------|------|
| 节点无法启动 | `source /opt/ros/humble/setup.bash` |
| 无话题 | 查 `ros2 launch` 日志 |

### 5. 板端资源

| 现象 | 对照 | 处理 |
|------|------|------|
| 空闲 CPU 远高于 8% 单核 | [§9.1](09_board_task_launch_benchmark.md) | 查残留 `autonomy.*` |
| RSS 远高于 400 MiB | 同左约 360–370 MiB | 查 costmap / 泄漏 |
| 启动很慢 | [§9.2](09_board_task_launch_benchmark.md) | 查磁盘 / setup |
