# autosim

多后端**传感器–执行器桥**：经 Autolink 发布传感、订阅 `/cmd_vel`，并对接 TF / 仿真时钟 / 可选真值地图。  
**不实现** MapService / NavigationService / LocalizationService RPC（建图算法由 Cartographer 等承担；`habitat.map` 仅提供场景真值栅格 / 点云）。

![Isaac Sim + Autoviz](images/isaac.gif)

> Isaac 后端（酷家乐 InteriorAgent）与 Autoviz：RGB-D、`/map`、`/overall/map`、PhysX 碰撞。动图见 [`images/`](images/)。

## 后端一览

| `habitat.backend` | 配置 | 说明 |
|-------------------|------|------|
| `habitat` | `config/default.yaml` | Habitat-Sim + GLB / MP3D |
| `fake` | `config/fake.yaml` | 无引擎：差速 + RGBD raycast + scenario 地图 |
| `isaac` | `config/isaac.yaml` | Isaac Sim + USD（`usd/`）；PhysX、相机、真值地图 |

## 模块（每文件一个类）

| 文件 | 类 | 职责 |
|------|-----|------|
| `config.py` | `Config` | YAML 加载与校验 |
| `clock.py` | `Clock` | 仿真时钟 |
| `robot/plant.py` | `Robot` | 差速、积分里程计、带偏置 IMU、真值 |
| `robot/profiles.py` | `RobotProfile` | turtlebot3 / husky 档案（URDF + USD） |
| `backend/` | `WorldBackend` | 引擎无关传感 / 位姿协议 + registry |
| `backends/habitat` | — | Habitat-Sim（包装 `simulator.py`） |
| `backends/fake` | `FakeWorld` | 过程 scenario + RGBD raycast |
| `backends/isaac` | `IsaacSimulator` | Isaac Sim + USD + PhysX |
| `backends/isaac/physics.py` | `IsaacPhysics` | PhysX World、胶囊碰撞、穿墙阻挡 |
| `backends/isaac/floorplan_map.py` | `FloorplanMap` | InteriorAgent `rooms.json` → `/map` |
| `sensors.py` | `Sensors` | 2D/3D lidar and camera sampling |
| `simulator.py` | `Simulator` | Habitat scene + cast_ray |
| `map.py` | `Map` | PLY 环视点云 + 2D OccupancyGrid |
| `scenario/map/scenario.py` | `Scenario` | 按类型生成占据体并写出 Habitat 网格 |
| `scenario/map/volume.py` | `Volume` | 体素占据、出生点挖空、二维栅格 |
| `scenario/map/stage.py` | `Stage` | 占据体 → Habitat Y-up OBJ |
| `scenario/arena.py` | `Arena` | 障碍与行人共用的平面活动范围 |
| `scenario/field.py` | `Field` | 二维占据：反弹、滑动、出生重采样 |
| `scenario/cast.py` | `Cast` | 射线与圆柱 / 胶囊的最近命中 |
| `scenario/traffic.py` | `Traffic` | 每拍推进动态体，并供激光取更近命中 |
| `scenario/obstacle/obstacle.py` | `Obstacle` | 边界与墙面反弹的动态圆柱障碍 |
| `scenario/pedestrian/pedestrian.py` | `Pedestrian` | 朝目标行走、沿墙滑动并相互避让的行人 |
| `urdf.py` | `UrdfModel` | URDF path + sensor mounts |
| `messages.py` | `Messages` | automsgs encode/decode |
| `bridge.py` | `Bridge` | autolink channels |
| `runner.py` | `Runner` | main loop |
| `teleop.py` | `Teleop` | keyboard → `/cmd_vel` |

## 资源目录

| 路径 | 内容 |
|------|------|
| [`images/`](images/) | 演示动图（如 `isaac.gif`） |
| [`usd/`](usd/) | Isaac USD：`worlds/`、`robots/`（大场景见 `usd/.gitignore`） |
| [`config/`](config/) | `default` / `fake` / `isaac` / `slam` / `lio` 等 |
| [`urdf/`](urdf/) | 机器人 URDF |

## 依赖

- 必需：`numpy`、`PyYAML`、`protobuf`
- 运行时：`autolink` Python、`automsgs` Python 绑定
- 可选：`habitat-sim`（`pip install -e ".[habitat]"`）
- 可选：Isaac Sim / Omniverse Python（`pip install -e ".[isaac]"` 文档化 extras；需在 Isaac 环境中运行）

## 安装与运行

**colcon / CMake**（`BUILD_AUTOSIM=ON`，默认开启）：构建 `autonomy` 时会执行 `pip install -e autosim`。

```bash
colcon build --packages-select autonomy
# 关闭：colcon build --packages-select autonomy --cmake-args -DBUILD_AUTOSIM=OFF
```

**手动安装**（或 Habitat 可选依赖）：

```bash
cd src/autonomy/autosim
pip install -e ".[dev]"          # 测试
pip install -e ".[habitat,dev]"  # 含 habitat-sim
pip install -e ".[isaac,dev]"    # 文档化 Isaac extras（需 Isaac Sim 环境）
```

**运行**（工作区根目录）：

```bash
cd /workspace/autonomy
src/autonomy/autosim/scripts/run.sh sim
./src/autonomy/autosim/scripts/run.sh teleop
```

```bash
CONFIG=src/autonomy/autosim/config/fake.yaml src/autonomy/autosim/scripts/run.sh sim

# Isaac：run.sh 自动改用 ISAACLAB_PATH（默认 /workspace/isaaclab）下的 python.sh
ISAACLAB_PATH=/workspace/isaaclab CONFIG=src/autonomy/autosim/config/isaac.yaml \
  src/autonomy/autosim/scripts/run.sh sim
# 无显示时默认 HEADLESS；需要 GUI 时可 AUTOSIM_ISAAC_HEADLESS=0
```

### Isaac 要点

- `sensors.camera.enabled: true`：机器人 prim 下挂 Camera（Replicator RGB + depth）；headless 同样 offscreen 渲染。
- `habitat.isaac.physics.enabled: true`（默认）：PhysX `World`、kinematic 胶囊碰撞；`block_motion` 时拒绝穿墙位姿。
- 真值地图：`map.enabled/publish: true` → `/map`（OccupancyGrid，InteriorAgent `rooms.json`）与 `/overall/map`（USD 网格点云）。
- 场景资源：默认 `usd/worlds/InteriorAgent/kujiale_*`（需自行下载，已被 `usd/.gitignore` 忽略）；占位烟测用 `usd/worlds/simple_room/`。
- Isaac Sim 自带 **Python 3.11**，autolink 需有对应 `_core.cpython-311-*.so`。若 `run.sh` 报缺少绑定，用 Isaac 解释器重编 pybind 后拷到 `install/autonomy/python/autolink/`。
- 机器人档案：`habitat.robot.profile` 可为 `turtlebot3_burger` 或 `husky`。

**Lightning LIO 验证**（开 3D 点云 `/points` + `/imu`）：

```bash
CONFIG=src/autonomy/autosim/config/lio.yaml src/autonomy/autosim/scripts/run.sh sim
# 另终端：autolink launch start localization_lio.launch
```

或进入 autosim 目录：

```bash
cd src/autonomy/autosim
./scripts/run.sh sim
./scripts/run.sh teleop
```

键盘遥控与 `autonomy_teleop` 相同：`w`/`x` 增减线速度，`a`/`d` 增减角速度，`space`/`s` 停车；速度会保持直到下一次按键。需在交互式 TTY 中运行（`docker exec -it SpaceHero /bin/bash`），并点击该终端窗口取得焦点。

`habitat.path` 为空且 `habitat.scenario.enabled: true` 时，按 [SensorSimulator](https://github.com/TJU-Aerial-Robotics/SensorSimulator) RandomScene 生成占据点云、二维栅格和 Habitat OBJ：`perlin` / `perlin2d`、`posts`、`maze`、`maze3d`、`random_forest`、`random_room`，以及需要 `scenario.ply` 的实景 `forest` / `building`。`obstacle` / `pedestrian` 不写入静态网格：控制周期调用 `step`，激光取圆柱或胶囊与静态命中的更近距离。默认 `habitat.mode: nav`：漂移修正的 `map→odom→base_link` + GT `/map`。`slam` 仅发 `odom→base_link`（Cartographer 负责 `map→odom`/`/map`）。

测试需要已安装的 `automsgs` Python 绑定（仓库内不再附带 `stubs/`）。

## 配置要点

- 噪声字段为高斯 σ；里程计为**积分噪声**（非每拍贴 GT）。
- `habitat.mode`：`nav`（默认）发漂移修正的 `map→odom→base_link`；`slam` 仅发 `odom→base_link`。
- `robot.tf` → `/tf` + `/tf_static`；`robot.clock` → `/clock`。
- `map.ply`：`source`/`file` 与/或 `channel`；`map.publish: true` 时 `map.grid` → `/map`。

| 方向 | 通道（默认） | Proto |
|------|------|-------|
| 入 | `/cmd_vel` | `TwistStamped` |
| 出 | `/scan` `/points` `/camera/*` `/imu` `/odom` | 传感 |
| 出 | `/tf` `/tf_static` | `TFMessage` |
| 出 | `/clock` | `Time` |
| 出 | `/map`、`/overall/map` | `OccupancyGrid` / `PointCloud2` |
| 出 | `/gt/pose` | `PoseStamped`（默认关） |

## Habitat 场景

```yaml
habitat:
  path: /data/hm3d/scene.basis.glb
  map:
    enabled: true
```

## 测试

```bash
cd autosim && pytest tests -v
```

## Autoviz 联调

1. 配置中设 `habitat.mode: nav`、`map.enabled: true`、`map.publish: true`（Isaac 默认已开）。
2. 启动仿真后启动 Autoviz，Fixed Frame 设为 `map`。
3. 常用话题：`/camera/rgb/image_raw`、`/camera/depth/image_raw`、`/map`、`/overall/map`、`/scan`、`/tf`。
4. PointCloud2（`/overall/map`）：**Color Transformer** → `Intensity` 或 `AxisColor`；大点云可设 **Decimate**；`map.ply.stride` 在发布端降采样。
