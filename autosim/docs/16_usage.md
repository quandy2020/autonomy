# 16 · 详细使用指南

面向日常运行：选择场景、机器人、传感器，启动/遥控/停止，并接入 Autoviz。
字段级说明见 [05 配置](05_configuration.md)，通道表见 [06 话题](06_topics.md)。

> 路径约定：以下命令默认在 `/workspace/autonomy` 执行；`scripts/run.sh` 是
> `src/autonomy/autosim/scripts/run.sh` 的简写。若已进入 `autosim/`，直接用
> `scripts/run.sh` 即可。

最短路径：

```bash
cd /workspace/autonomy
src/autonomy/autosim/scripts/run.sh sim --list
SCENE=17DRP5sb8fy ROBOT=unitree_go2 SENSORS=basic \
  src/autonomy/autosim/scripts/run.sh sim
src/autonomy/autosim/scripts/run.sh teleop   # 另开终端
src/autonomy/autosim/scripts/run.sh stop
```

| 相关 | 链接 |
|------|------|
| 配置字段 | [05](05_configuration.md) |
| 话题 | [06](06_topics.md) |
| 机器人 / 关节 | [15](15_robot_articulation.md) |
| 地图 / Autoviz | [08](08_map_and_viz.md) |
| 排错 | [10](10_troubleshooting.md) |

---

## 1. 运行位置与前置条件

默认在 SpaceHero 容器内、工作区根目录执行：

```bash
docker exec -it SpaceHero bash
cd /workspace/autonomy
```

`scripts/run.sh` 会准备 `PYTHONPATH`、`LD_LIBRARY_PATH`、Autolink 路径和 Python 解释器。推荐始终通过它启动；直接 `python -m autosim` 需要自行配置同样的环境。

先确认可用项：

```bash
src/autonomy/autosim/scripts/run.sh sim --list
```

输出包含场景、机器人、传感器和预设。若提示 autolink bindings 缺失，先构建/安装 autonomy，或设置 `AUTOLINK_PYTHON_DIR`。

AutoSim 默认只允许一个实例，锁文件是 `/tmp/autosim.lock`。切换配置前先停止旧实例：

```bash
src/autonomy/autosim/scripts/run.sh stop
```

---

## 2. 启动模式

| 模式 | 命令 | 用途 |
|------|------|------|
| `sim` / `bridge` | `scripts/run.sh sim` | 只启动仿真桥 |
| `teleop` / `telop` | `scripts/run.sh teleop` | 键盘遥控，需要交互式 TTY |
| `both` | `scripts/run.sh both` | 同一终端启动 sim + teleop |
| `stop` | `scripts/run.sh stop` | 停止当前实例 |

`both` 会在后台启动 sim，前台进入键盘控制。退出 teleop 后如果 sim 仍在运行，执行 `scripts/run.sh stop`。

查看 CLI 参数：

```bash
src/autonomy/autosim/scripts/run.sh sim --help
```

---

## 3. 三种配置入口

### 3.1 选择器（推荐）

```bash
SCENE=17DRP5sb8fy ROBOT=unitree_go2 SENSORS=basic \
  src/autonomy/autosim/scripts/run.sh sim
```

也可以直接传 CLI：

```bash
src/autonomy/autosim/scripts/run.sh sim \
  --scene 17DRP5sb8fy \
  --robot unitree_go2 \
  --sensors camera,lidar_2d,imu,odom
```

选择器会以 `CONFIG`（默认 `config/default.yaml`）为基础，依次叠加场景、机器人档案和传感器选择。

### 3.2 完整 YAML

```bash
CONFIG=src/autonomy/autosim/config/atlas.yaml \
  src/autonomy/autosim/scripts/run.sh sim
```

适合需要自定义噪声、话题、地图、行人、数据集或策略的场景。完整字段见 [05](05_configuration.md)。

### 3.3 优先级

`CONFIG` 基础配置 → `SCENE` → `ROBOT` → `SENSORS`。机器人档案会覆盖基础配置中的 URDF、轮距、初始关节姿态和速度上限；显式 `SENSORS` 只保留列出的传感器。

---

## 4. 场景

### 4.1 查看和切换

```bash
src/autonomy/autosim/scripts/run.sh sim --list
SCENE=jtcxE69GiFV ROBOT=unitree_go2 SENSORS=basic \
  src/autonomy/autosim/scripts/run.sh sim
```

当前工作区已登记 10 个 MP3D GLB 场景：`17DRP5sb8fy`、`29hnd4uzFmX`、`2azQ1b91cZZ`、`S9hNv5qa7GM`、`jtcxE69GiFV`、`kEZ7cmS4wCh`、`mJXqzFtmKg4`、`q9vSo1VnCiC`、`vyrNrziPKCB`、`zsNo4HB9uLZ`；以 `--list` 输出为准。`empty` 是空场景 studio，不需要 YAML。

### 4.2 临时场景

直接给 GLB/USD 路径，不需要新增 YAML：

```bash
SCENE=/workspace/autonomy/src/MyScene/MyScene.glb \
ROBOT=unitree_go2 SENSORS=basic \
src/autonomy/autosim/scripts/run.sh sim
```

相对路径按工作区根目录解析。

### 4.3 可复用场景

新增 `config/scenes/<scene>.yaml`，只写覆盖项：

```yaml
habitat:
  path: ${AUTOSIM_SCENE_ROOT:-/workspace/autonomy/src}/MyScene/MyScene.glb
  # 可选；不写则沿用基础配置并自动吸附 navmesh
  # spawn: [0.0, 0.0, 0.0]
```

然后就可以用短名称启动：

```bash
SCENE=MyScene ROBOT=unitree_go2 SENSORS=basic \
  src/autonomy/autosim/scripts/run.sh sim
```

场景根目录默认 `/workspace/autonomy/src`，可覆盖：

```bash
AUTOSIM_SCENE_ROOT=/data/mp3d SCENE=17DRP5sb8fy \
ROBOT=unitree_go2 SENSORS=basic \
src/autonomy/autosim/scripts/run.sh sim
```

### 4.4 空场景

```bash
SCENE=empty ROBOT=google_robot SENSORS=basic \
  src/autonomy/autosim/scripts/run.sh sim
```

空场景使用 `config/robots/_base.yaml` 的 studio 地面和中性背景，适合查看机器人外观、关节和传感器。

---

## 5. 机器人

| profile | 自由度 | 默认传感器 | 速度上限 |
|---------|--------|------------|----------|
| `google_robot` | 15 | camera、stereo、range、imu、odom、joint_states、chase | 0.5 m/s / 1.0 rad/s |
| `unitree_g1` | 29 | camera、lidar_2d、imu、odom、joint_states、chase | 1.2 / 1.5 |
| `unitree_go2` | 12 | camera、lidar_2d、imu、odom、joint_states、chase | 2.0 / 2.0 |
| `turtlebot3_burger` | 2 | camera、lidar_2d、imu、odom、joint_states、chase | 0.5 / 1.0 |
| `husky` | 4 | camera、lidar_2d、imu、odom、joint_states、chase | 1.0 / 1.5 |

选择机器人：

```bash
SCENE=empty ROBOT=unitree_g1 SENSORS=basic \
  src/autonomy/autosim/scripts/run.sh sim
```

`ROBOT` 也可以写机器人 YAML 路径。省略 `SENSORS` 时使用该档案默认传感器；Google 默认不含 2D 激光，而包含双目、ToF 和悬崖测距。

五个模型都订阅 `/cmd_vel`，也都支持 `/joint_commands`。机器人资源、URDF、mesh 和关节列表见 [15](15_robot_articulation.md)。这五个档案面向 Habitat；Isaac 使用自己的 USD 配置。

---

## 6. 传感器

### 6.1 名称

| 名称 | 内容 |
|------|------|
| `camera` | RGB + depth + CameraInfo |
| `lidar_2d` | 2D LaserScan |
| `lidar_3d` | 3D PointCloud2 |
| `stereo` | 左右目 RGB + CameraInfo |
| `range` | ToF / 悬崖等单光束 Range |
| `imu` | Imu |
| `odom` | Odometry |
| `chase` | 第三人称跟随 RGB |
| `topdown` | 俯视 RGB |
| `joint_states` | JointState |
| `cameras` | 环视相机组（配置提供时） |
| `panorama` | 全景相机（配置提供时） |

### 6.2 预设

| 预设 | 等价内容 |
|------|----------|
| `basic` | `camera,lidar_2d,imu,odom` |
| `rgbd` | `camera,imu,odom` |
| `lidar` | `lidar_2d,imu,odom` |
| `all` | 当前配置里存在的全部传感器块 |
| `none` | 关闭全部传感器块 |

### 6.3 显式选择

```bash
# 只开 RGB-D、IMU、里程计
SENSORS=camera,imu,odom \
  src/autonomy/autosim/scripts/run.sh sim

# 只开 2D 激光和里程计
SENSORS=lidar_2d,odom \
  src/autonomy/autosim/scripts/run.sh sim

# 全部可用传感器
SENSORS=all \
  src/autonomy/autosim/scripts/run.sh sim

# 关闭全部传感器块；TF/clock 等机器人级输出仍按配置
SENSORS=none \
  src/autonomy/autosim/scripts/run.sh sim
```

规则：

- 省略 `SENSORS`：使用机器人档案默认值。
- 显式写 `SENSORS`：只打开列出的块，其余关闭。
- `all`：打开当前配置存在的块，不存在的跳过。
- 显式写一个当前配置没有的传感器：启动时报 `sensor(s) not defined`。
- 对 Google 使用 `SENSORS=basic` 会把传感器切换为 `camera,lidar_2d,imu,odom`，并关闭 `stereo` / `range`；想保留 Google 原装组合时省略 `SENSORS`。

别名：`scan`/`laser` → `lidar_2d`，`lidar3d`/`points` → `lidar_3d`，`rgb`/`depth` → `camera`，`tof` → `range`，`joints` → `joint_states`，`surround` → `cameras`，`pano` → `panorama`。

---

## 7. 控制

### 7.1 `/cmd_vel`

所有机器人订阅 Autolink `TwistStamped`：

- `twist.linear.x`：前进/后退，m/s
- `twist.angular.z`：原地转向/弧线，rad/s
- 0.5 秒没有新指令会减速停止

键盘遥控：

```bash
SCENE=empty ROBOT=unitree_go2 SENSORS=basic \
  src/autonomy/autosim/scripts/run.sh both
```

按键：`w`/`x` 增加/减少线速度，`a`/`d` 增加/减少角速度，空格或 `s` 停车。需要终端焦点和 TTY。

### 7.2 `/joint_commands`

`/joint_commands` 接收 Autolink `JointState` protobuf，不是 ROS middleware。`name` 和 `position` 数组等长，可只更新部分关节：

- 旋转关节：弧度
- 移动关节：米
- 未知名称、重复名称、非有限值、超限位置会拒绝整条命令
- Mimic 关节跟随源关节，控制源关节即可

状态从 `/joint_states` 发布；活动关节走 `/tf`，固定关节走 `/tf_static`，`/robot_description` 发布 URDF。浏览器查看器和滑块：

```bash
docker exec -it -w /workspace/autonomy/src/autonomy/autosim \
  SpaceHero scripts/run_robot_viewer.sh \
  --profile google_robot --host 0.0.0.0 --port 8765
```

浏览器打开 `http://<主机IP>:8765`；只想列出关节名和范围时加 `--list`。

---

## 8. 常用配方

### 8.1 空场景查看 Google Robot

```bash
SCENE=empty ROBOT=google_robot \
  src/autonomy/autosim/scripts/run.sh sim
```

### 8.2 Go2 在 MP3D 场景中导航

```bash
SCENE=17DRP5sb8fy ROBOT=unitree_go2 SENSORS=basic \
  src/autonomy/autosim/scripts/run.sh sim
```

### 8.3 G1 人形机器人

```bash
SCENE=empty ROBOT=unitree_g1 SENSORS=basic \
  src/autonomy/autosim/scripts/run.sh both
```

### 8.4 Husky 和 TurtleBot3

```bash
SCENE=jtcxE69GiFV ROBOT=husky SENSORS=basic \
  src/autonomy/autosim/scripts/run.sh sim

SCENE=jtcxE69GiFV ROBOT=turtlebot3_burger SENSORS=basic \
  src/autonomy/autosim/scripts/run.sh sim
```

### 8.5 Google 原装传感器

```bash
SCENE=empty ROBOT=google_robot \
  src/autonomy/autosim/scripts/run.sh sim
```

### 8.6 Fake 后端

```bash
CONFIG=src/autonomy/autosim/config/fake.yaml \
  src/autonomy/autosim/scripts/run.sh sim
```

### 8.7 Atlas RGB-D + IMU

```bash
CONFIG=src/autonomy/autosim/config/atlas.yaml \
  src/autonomy/autosim/scripts/run.sh sim
autoviz --config src/autonomy/autoviz/config/atlas.autoviz
```

完整四终端流程见 [13](13_atlas_validation.md)。

---

## 9. 话题与 Autoviz

常用通道：

| 方向 | 通道 |
|------|------|
| 入 | `/cmd_vel`、`/joint_commands` |
| 出 | `/odom`、`/scan`、`/points`、`/imu/data` |
| 出 | `/camera/rgb/image_raw`、`/camera/depth/image_raw`、`/camera/camera_info` |
| 出 | `/camera/chase/image_raw`、`/camera/topdown/image_raw` |
| 出 | `/joint_states`、`/tf`、`/tf_static`、`/clock` |
| 出 | `/map`、`/overall/map`、`/robot_description` |

完整通道表见 [06](06_topics.md)。Autoviz：

1. 启动 sim，确保 `map.publish: true`（Atlas 用 `atlas.yaml`）。
2. 启动 Autoviz，Fixed Frame 设为 `map`。
3. 添加 Image、LaserScan、Map、PointCloud2、RobotModel、TF 等显示。

---

## 10. 配置扩展

- 场景：`config/scenes/<scene>.yaml`，只写 `habitat.path` 和需要覆盖的 `spawn` 等。
- 机器人：`config/robots/<profile>.yaml`，通过 `extends: _base.yaml` 复用公共块。
- 传感器：直接在选择器里开关，或在 YAML 的 `habitat.sensors` 下改 `enabled`、`rate_hz`、`noise`、`channel`。
- 继承：`extends` 支持相对路径和列表，映射深合并，列表/标量替换。
- 环境变量：`AUTOSIM_SCENE_ROOT` 场景根；`ISAACLAB_PATH` Isaac Lab；`AUTOSIM_ISAAC_HEADLESS=0` Isaac GUI；`AUTOLINK_PYTHON_DIR` Autolink Python 目录；`AUTOSIM_INSTANCE_LOCK_PATH` 可改实例锁文件，仅用于隔离测试，并行实例仍需避免同名话题。

---

## 11. 停止与排错

```bash
src/autonomy/autosim/scripts/run.sh stop
```

常见问题：

| 现象 | 处理 |
|------|------|
| `another AutoSim instance is already running` | 先 `run.sh stop`；不要同时启动两个实例 |
| `unknown scene` | 用 `--list` 的名称，或给存在的 GLB/USD/YAML 路径 |
| `sensor(s) not defined` | 该配置没有这个传感器块；换配置或用 `all` |
| `autolink Python bindings ... not found` | 构建/安装 autonomy，或设置 `AUTOLINK_PYTHON_DIR` |
| teleop 无响应 | 在交互式 TTY 中运行，保持终端焦点 |
| 没有图像 | 检查 `camera.enabled`、场景灯光、Autoviz 订阅和 Fixed Frame |

更多见 [10](10_troubleshooting.md)。

---

## 12. 命令速查

| 目的 | 命令 |
|------|------|
| 列出场景/机器人/传感器 | `scripts/run.sh sim --list` |
| 选择器启动 | `SCENE=... ROBOT=... SENSORS=... scripts/run.sh sim` |
| 完整 YAML | `CONFIG=... scripts/run.sh sim` |
| 仿真 + 键盘遥控 | `... scripts/run.sh both` |
| 单独键盘遥控 | `scripts/run.sh teleop` |
| 停止 | `scripts/run.sh stop` |
| 查看机器人关节 | `scripts/run_robot_viewer.sh --profile ... --list` |
| 浏览器查看器 | `scripts/run_robot_viewer.sh --profile ...` |
