# 05 · 配置

配置真相：`config/*.yaml` + `autosim/runtime/settings.py` 校验。本节只列稳定约定。

| 相关 | 链接 |
|------|------|
| 话题名 | [06](06_topics.md) |
| Isaac 块 | [07](07_isaac.md) |

---

## 1. 选用哪份 YAML

| 文件 | 用途 |
|------|------|
| `default.yaml` | Habitat 主配置 |
| `atlas.yaml` | Habitat Atlas 验证：RGB-D + IMU + chase + GT `/overall/map` |
| `habitat_people.yaml` | Habitat + CPU 行人 / SMPL-X |
| `fake.yaml` | 无引擎联调 |
| `isaac.yaml` | Isaac + USD |
| `isaac_nav.yaml` | Isaac 导航 / SLAM 传感 |
| `slam.yaml` | `mode: slam`，默认不发 GT OccupancyGrid |
| `lio.yaml` / `lightning.yaml` | 开 3D 点云 + IMU，LIO 验证 |
| `bev.yaml` | BEV 相关实验配置 |
| `nav.yaml` | Habitat 导航联调（与 `default.yaml` 同栈，显式 nav + GT 地图） |
| `fake_nav.yaml` | fake 后端的导航联调 |
| `coverage.yaml` | fake 后端的全覆盖验证场景 |
| `robots/<profile>.yaml` | 五个机器人档案的展示与遥操作配置，见第 4 节；公共块在 `robots/_base.yaml` |
| `scenes/<scene>.yaml` | 场景路径片段，供 `extends` 引用；根目录可用 `AUTOSIM_SCENE_ROOT` 覆盖 |

启动：`CONFIG=.../xxx.yaml scripts/run.sh sim`。

### 简洁启动

日常运行不需要改 YAML，直接选择场景、机器人和传感器：

```bash
SCENE=17DRP5sb8fy ROBOT=unitree_go2 SENSORS=basic \
  src/autonomy/autosim/scripts/run.sh sim

SCENE=empty ROBOT=google_robot \
  src/autonomy/autosim/scripts/run.sh sim
```

| 选择器 | 取值 |
|--------|------|
| `SCENE` / `--scene` | `config/scenes/` 名称、场景 YAML、GLB/USD 路径、`empty` |
| `ROBOT` / `--robot` | `config/robots/` 名称或机器人 YAML 路径 |
| `SENSORS` / `--sensors` | 逗号分隔的传感器名；`basic`、`rgbd`、`lidar`、`all`、`none` |

`basic` 为 `camera,lidar_2d,imu,odom`。省略 `SENSORS` 时使用所选机器人档案的默认传感器；
显式指定时只打开列出的传感器。`--scene`、`--robot`、`--sensors` 也可以直接跟在
`scripts/run.sh sim` 后面。`python3 -m autosim --list` 列出可用名称。

---

## 2. 根键结构（示意）

```yaml
habitat:
  backend: isaac          # habitat | fake | isaac
  mode: nav               # nav | slam
  path: usd/worlds/...    # 场景路径（相对 autosim/ 或绝对）
  spawn: [x, y, yaw]
  robot: { ... }
  sensors: { lidar_2d, lidar_3d, camera, stereo, range, topdown, chase, imu, odom, joint_states }
  map: { enabled, publish, ply, grid }
  scenario: { enabled, ... }   # Fake / Habitat 过程场景
  isaac: { headless, physics } # 仅 Isaac
  dataset: { enabled, root, fps, ... }  # LeRobot 录制，见 [11](11_dataset.md)
  policy: { enabled, model_path, ... }  # 可选 ONNX 运控
```

路径相对 **autosim 工程根**（含 `config/`、`usd/` 的那一层）。

### 场景路径

`config/scenes/<scene>.yaml` 只保存 `habitat.path`（需要时也保存 `map.ply.source`）。默认场景根目录是 `/workspace/autonomy/src`，可用环境变量覆盖：

```bash
AUTOSIM_SCENE_ROOT=/data/mp3d CONFIG=config/default.yaml scripts/run.sh sim
```

`Config.load` 会展开所有字符串里的 `${VAR}` 和 `${VAR:-default}`；变量未设置且没有默认值时抛 `ValueError`。

---

## 3. 常用字段

| 字段 | 含义 |
|------|------|
| `robot.profile` | `google_robot` / `unitree_g1` / `unitree_go2` / `turtlebot3_burger` / `husky` 均有 `config/robots/<profile>.yaml`，自动填 URDF、`instance`、`fixed_base` 和 `root_height`；轮式两个 profile 另带 `wheel_radius` / `wheel_separation`，轮子随 `/cmd_vel` 滚动 |
| `robot.control_hz` | 控制与积分频率 |
| `robot.tf.publish_map_odom` | 可被 `mode` 覆盖 |
| `robot.truth.enabled` | 同时启用真值 PoseStamped 与可选 Odometry 输出 |
| `robot.truth.channel` | 兼容真值位姿通道，默认 `/gt/pose` |
| `robot.truth.odometry_channel` | 可选真值里程计通道；非空时发布 `map -> base_link` Odometry，Atlas 使用 `/gt/odometry` |
| `sensors.*.rate_hz` | 发布频率门控 |
| `sensors.*.noise` | 高斯 σ；odom 为积分噪声 |
| `map.enabled` | 是否构建真值地图 |
| `map.publish` | 是否发布；`slam` 下仍可发 `/overall/map`，但 OccupancyGrid `/map` 与 `map→odom` 关闭 |
| `map.grid.resolution` | OccupancyGrid 分辨率 |
| `map.grid.z_min` / `z_max` | 高度带；加载 PLY 时也过滤 `/overall/map` |
| `map.ply.channel` | 点云通道，如 `/overall/map` |
| `map.ply.source` | 可选外部 PLY（MP3D `*_semantic.ply`）；优先于环视 ray-cast |
| `map.ply.stride` | 发布端降采样步长（大 PLY 需 ≥5 以免超过 Autolink SHM 32 MiB） |
| `sensors.topdown.*` | 全局正交俯视 RGB；见 [08](08_map_and_viz.md) |
| `sensors.chase.*` | 第三人称跟随 RGB；开启后自动 instance URDF（仅 chase 帧可见） |
| `robot.instance` | 强制在 Habitat 场景挂 URDF AO（默认 false；chase 会自动打开） |
| `dataset.enabled` | 是否录制 LeRobot 数据集 |
| `dataset.root` | 输出目录（含 `meta/` `data/` `videos/`） |
| `dataset.fps` | 录制频率（可低于 `control_hz`） |
| `dataset.episode_frames` | `>0` 时按帧数切 episode；`0` 则退出时 flush |
| `scenario.pedestrian` | CPU 行人（agents YAML / SFM）；激光融合始终可用 |
| `humanoids.*` | Habitat SMPL-X 视觉（TrackVLA 布局）；见 [14](14_habitat_humanoids.md) |

---

## 4. 机器人档案

`autosim/robot/profiles.py`：`profile` 非空时合并默认 `urdf`、`usd_robot`、`wheel_separation`、`base_link_height`。  
YAML 显式字段优先于档案默认值。

`config/robots/<profile>.yaml` 是每个模型的展示/遥操作入口，`shipped_robot_defaults()` 也读这份文件，因此查看器、`Runner` 和 smoke 脚本共用同一套初始姿态与速度上限。

| profile | 自由度 | URDF 根 | 说明 |
|---------|--------|---------|------|
| `google_robot` | 15 | `link_base` | 轮式底盘 + 单臂 + 夹爪 + 头；双目、8 路 ToF、2 路悬崖 |
| `unitree_g1` | 29 | `pelvis` | 人形；RGB-D + 2D 激光 + IMU |
| `unitree_go2` | 12 | `base` | 四足；RGB-D + 2D 激光 + IMU |
| `turtlebot3_burger` | 2 | `base_footprint` | 两轮差速；`wheel_radius: 0.033` |
| `husky` | 4 | `base_footprint` | 四轮差速；`wheel_radius: 0.1651` |

`sensors.odom.child_frame` 应写成该模型的 URDF 根节点。运行时的 `odom_child_frame()` 会回退到根节点，写错不会报错，但 `/odom` 的 `child_frame_id` 会和 TF 对不上。

### 继承：`extends`

`Config.load` 支持根级 `extends`，机器人档案用它把公共块收进 `config/robots/_base.yaml`：

```yaml
extends: _base.yaml
habitat:
  robot:
    profile: unitree_go2
```

- 相对路径按声明它的 YAML 所在目录解析；也可写列表，按从左到右深合并。
- 映射递归合并，列表和标量整体替换；子文件字段优先。
- `_base.yaml` 是片段，文件名以 `_` 开头的 YAML 不当作可直接启动的配置。

---

## 5. 校验失败时

`Config.load` 抛 `ValueError`（空通道、重复通道、非法 backend 等）。  
先对照本表与目标 YAML，再查 `autosim/runtime/settings.py` 中 `validate_*`。

真值双路输出示例：

```yaml
robot:
  truth:
    enabled: true
    channel: /gt/pose
    odometry_channel: /gt/odometry
```

将 `odometry_channel` 省略或设为空字符串可只保留原有 PoseStamped；这不会改变旧配置行为。
