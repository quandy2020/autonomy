# 运行指南

本指南介绍如何运行 Autonomy 项目，包括 C++ 直接运行、ROS2 集成运行和 Docker 容器运行等方式。

---

## 1 C++ 直接运行

### 1.1 运行格式

C++ 可执行文件命名格式：`工程名.模块名.组件名.可执行文件名`

```bash
# 示例
./autonomy.map.grid_map.grid_map_core.grid_map_test
```

![运行示例](./images/running.png)

### 1.2 命令行运行

在编译完成后，进入可执行文件所在目录运行：

```bash
cd /workspace/autonomy/build/autonomy/bin
./autonomy.map.grid_map.grid_map_core.grid_map_test
```

![命令行运行](./images/command_line_run.png)

---

## 2 ROS2 集成运行

### 2.1 环境变量配置

在运行 ROS2 节点之前，需要设置必要的环境变量。建议将其添加到 `~/.bashrc` 或 `~/.zshrc` 文件中：

```bash
### Autonomy 环境变量 ###
export GLOG_logtostderr=1           # 日志输出到标准错误
export GLOG_alsologtostderr=0       # 不同时写入日志文件
export GLOG_colorlogtostderr=1      # 启用彩色日志
export GLOG_minloglevel=0           # 最小日志级别 (0=INFO)
export AUTOLINK_PATH=/workspace/autonomy/src/autonomy/autolink/autolink

### Autonomy_ROS 环境变量 ###
export AUTONOMY_MODEL=waffle        # 机器人模型（可选）

### ROS2 环境 ###
source /opt/ros/humble/setup.bash
source /workspace/autonomy/install/setup.bash
```

使配置生效：

```bash
source ~/.bashrc
```

### 2.2 启动 Autonomy_ROS

使用 launch 文件启动 Autonomy_ROS 系统：

```bash
ros2 launch autonomy_ros autonomy.launch.py
```

**Launch 参数说明**:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_sim_time` | `true` | 是否使用仿真时间 |
| `use_gazebo` | `false` | 是否使用 Gazebo 仿真 |
| `world_type` | `house` | 仿真世界类型 (`house` 或 `world`) |
| `x_pose` | `-2.0` | 机器人初始 X 坐标 |
| `y_pose` | `-0.5` | 机器人初始 Y 坐标 |

**示例**:

```bash
# 使用 Gazebo 仿真
ros2 launch autonomy_ros autonomy.launch.py use_gazebo:=true world_type:=house

# 使用假节点（无仿真）
ros2 launch autonomy_ros autonomy.launch.py use_gazebo:=false
```

![ROS2 运行效果](./images/openbot_ros_simulator.png)

### 2.3 Gazebo 仿真（可选）

如果需要使用 Gazebo 进行仿真，可以使用以下命令：

```bash
# 空世界仿真
ros2 launch autonomy_gazebo empty_world.launch.py gui:=true spawn_robot:=true

# 房屋环境仿真
ros2 launch autonomy_gazebo autonomy_house.launch.py gui:=true spawn_robot:=true
```

### 2.4 Docker 容器运行

推荐使用 Docker 作为开发环境。启动脚本为 `autonomy/docker/run_autonomy.py`，会自动检测平台、选择镜像、挂载代码目录并启动名为 `SpaceHero` 的容器。

#### 2.4.1 快速启动

```bash
cd autonomy/docker
python3 run_autonomy.py
```

常用启动示例：

```bash
# 指定 x86_64 平台，使用 NVIDIA 镜像（Isaac-Sim / Isaac-Lab）
python3 run_autonomy.py -p x86_64 -n yes

# 指定标准镜像（无 NVIDIA）
python3 run_autonomy.py -p x86_64 -n no

# ARM64 平台
python3 run_autonomy.py -p aarch64

# 将额外参数传给 docker run（写在 -- 之后）
python3 run_autonomy.py -- --rm
```

进入已运行的容器：

```bash
docker exec -it SpaceHero /bin/bash
```

容器内启动 ROS2（需先 source 环境）：

```bash
source /opt/ros/humble/setup.bash
source /workspace/autonomy/install/setup.bash
ros2 launch autonomy_ros autonomy.launch.py
```

#### 2.4.2 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `-p`, `--platform` | 自动检测 | 平台架构：`x86_64`、`aarch64`、`arm64` |
| `-n`, `--nvidia` | `auto` | 镜像选择：`auto`（自动）、`yes`（NVIDIA）、`no`（标准） |
| `--keep-isaac-entrypoint` | 关闭 | NVIDIA 镜像下保留镜像自带 ENTRYPOINT（可能自动启动 Kit/流式服务） |
| `--data-volume` | 无 | 宿主机数据卷，可重复指定；格式见下文 |
| `--` 之后的参数 | 无 | 原样传递给 `docker run` |

**`--data-volume` 格式**：

| 格式 | 说明 |
|------|------|
| `HOST` | 宿主机与容器路径相同，读写 |
| `HOST:CONTAINER` | 自定义容器内路径 |
| `HOST:CONTAINER:MODE` | 自定义权限，如 `ro` |

示例：

```bash
# 挂载 4TB 数据盘
python3 run_autonomy.py --data-volume /mnt/data4t

# 指定容器内路径与权限
python3 run_autonomy.py --data-volume /mnt/data4t:/mnt/data4t:rw

# 挂载多个数据卷
python3 run_autonomy.py --data-volume /mnt/data4t --data-volume /data/shared:/data/shared
```

#### 2.4.3 环境变量配置

建议将常用配置写入 `~/.bashrc` 或 `~/.zshrc`，一次设置、长期生效：

```bash
### Autonomy Docker 运行配置 ###

# 宿主机代码目录（挂载到容器 /workspace/autonomy）
export AUTONOMY_ENV=/path/to/autonomy

# 宿主机数据卷（逗号或空格分隔多个路径）
export AUTONOMY_DATA_VOLUMES=/mnt/data4t

# 容器名称、端口映射、网络模式
export AUTONOMY_CONTAINER_NAME=SpaceHero
export AUTONOMY_PORTS=8765:8765
export AUTONOMY_NETWORK=host

# NVIDIA 镜像：保留 Isaac-Lab 默认 ENTRYPOINT（等价于 --keep-isaac-entrypoint）
# export AUTONOMY_KEEP_ISAAC_ENTRYPOINT=1
```

使配置生效：

```bash
source ~/.bashrc   # 或 source ~/.zshrc
```

**环境变量说明**：

| 环境变量 | 说明 | 优先级 |
|----------|------|--------|
| `AUTONOMY_ENV` | 宿主机项目路径，挂载为容器内 `/workspace/autonomy`；未设置时自动检测仓库根目录 | 高于自动检测 |
| `AUTONOMY_DATA_VOLUMES` | 数据卷列表，如 `/mnt/data4t` 或 `/mnt/data4t,/other/path` | 低于 `--data-volume` |
| `AUTONOMY_CONTAINER_NAME` | Docker 容器名称 | 默认 `SpaceHero` |
| `AUTONOMY_PORTS` | 端口映射，如 `8765:8765` 或 `8765:8765,8080:8080` | 默认 `8765:8765` |
| `AUTONOMY_NETWORK` | 网络模式（Linux），如 `host`、`bridge` | 默认 `host` |
| `AUTONOMY_KEEP_ISAAC_ENTRYPOINT` | 设为 `1` 时保留 NVIDIA 镜像默认 ENTRYPOINT | 低于 `--keep-isaac-entrypoint` |
| `DISPLAY` | X11 显示（GUI 转发），默认 `:0` | 继承宿主机 |

配置好环境变量后，直接运行即可，无需每次传参：

```bash
cd autonomy/docker
python3 run_autonomy.py -p x86_64 -n yes
```

#### 2.4.4 默认容器行为

以下为可通过环境变量配置的项（未设置时使用默认值）：

| 项目 | 环境变量 | 默认值 |
|------|----------|--------|
| 容器名称 | `AUTONOMY_CONTAINER_NAME` | `SpaceHero` |
| 工作目录 | — | `/workspace/autonomy` |
| 端口映射 | `AUTONOMY_PORTS` | `8765:8765` |
| 网络模式 | `AUTONOMY_NETWORK` | `host`（Linux） |
| 特权模式 | — | `--privileged` |
| 设备挂载 | — | `/dev:/dev` |
| 时区 | — | `/etc/localtime`（只读） |

如需临时覆盖端口、网络等，可通过 `--` 传递 `docker run` 参数，或联系维护者扩展可配置项。

#### 2.4.5 NVIDIA 镜像说明

使用 `-n yes` 或自动选中 NVIDIA 镜像时，脚本默认以 `--entrypoint /bin/bash` 启动，避免容器一启动就运行 Omniverse Kit / 流式服务。

若需恢复镜像自带启动逻辑：

```bash
python3 run_autonomy.py -p x86_64 -n yes --keep-isaac-entrypoint
# 或
AUTONOMY_KEEP_ISAAC_ENTRYPOINT=1 python3 run_autonomy.py -p x86_64 -n yes
```

---

## 3 验证运行状态

### 3.1 检查节点

```bash
# 查看所有运行中的节点
ros2 node list

# 查看特定节点信息
ros2 node info /<node_name>
```

### 3.2 查看话题

```bash
# 列出所有话题
ros2 topic list

# 查看话题内容
ros2 topic echo /<topic_name>

# 查看话题信息
ros2 topic info /<topic_name>
```

### 3.3 使用可视化工具

```bash
# 启动 RViz2
rviz2

# 启动 rqt
rqt
```

---

## 4 常见问题

### Q: 提示找不到可执行文件？

**A**: 确保已经完成编译，并且可执行文件路径正确。

```bash
# 检查编译是否成功
ls /workspace/autonomy/install/autonomy/bin/
```

### Q: ROS2 节点无法启动？

**A**: 检查环境变量是否已正确设置：

```bash
echo $GLOG_logtostderr
echo $AUTOLINK_PATH
```

### Q: Docker 容器内找不到数据盘或无法写入？

**A**: 在宿主机配置数据卷并确认路径存在：

```bash
# 宿主机检查挂载
ls /mnt/data4t

# 方式一：环境变量（推荐）
export AUTONOMY_DATA_VOLUMES=/mnt/data4t
python3 run_autonomy.py

# 方式二：命令行
python3 run_autonomy.py --data-volume /mnt/data4t
```

容器内验证：

```bash
docker exec -it SpaceHero ls /mnt/data4t
```

### Q: Gazebo 仿真无法启动？

**A**: 确保已安装 Gazebo 和相关依赖：

```bash
# 检查 Gazebo
which gazebo
gazebo --version
```
