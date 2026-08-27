# autosim

Habitat-Sim **传感器–执行器桥**：经 autolink 发布传感、订阅 `/cmd_vel`，并对接 TF / 仿真时钟 / 可选真值地图。  
**不实现** MapService / NavigationService / LocalizationService RPC（建图算法由 Cartographer 等承担；`habitat.map` 仅提供场景真值栅格/PLY）。

## 模块（每文件一个类）

| 文件 | 类 | 职责 |
|------|-----|------|
| `config.py` | `Config` | YAML 加载与校验 |
| `clock.py` | `Clock` | 仿真时钟 |
| `robot.py` | `Robot` | 差速、积分里程计、带偏置 IMU、真值 |
| `sensors.py` | `Sensors` | 2D/3D lidar and camera sampling |
| `simulator.py` | `Simulator` | Habitat scene + cast_ray（无 Mock） |
| `map.py` | `Map` | PLY 环视点云 + 2D OccupancyGrid（自由空间雕刻） |
| `urdf.py` | `UrdfModel` | URDF path + sensor mounts |
| `messages.py` | `Messages` | automsgs encode/decode |
| `bridge.py` | `Bridge` | autolink channels |
| `runner.py` | `Runner` | main loop |
| `teleop.py` | `Teleop` | keyboard → `/cmd_vel` |

## 依赖

- 必需：`numpy`、`PyYAML`、`protobuf`
- 运行时：`autolink` Python、`automsgs` Python 绑定
- 可选：`habitat-sim`（`pip install -e ".[habitat]"`）

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
```

**运行**（工作区根目录）：

```bash
cd /workspace/autonomy
src/autonomy/autosim/scripts/run.sh sim
./src/autonomy/autosim/scripts/run.sh teleop
```

或进入 autosim 目录：

```bash
cd src/autonomy/autosim
./scripts/run.sh sim
./scripts/run.sh teleop
```

键盘遥控与 `autonomy_teleop` 相同：`w`/`x` 增减线速度，`a`/`d` 增减角速度，`space`/`s` 停车；速度会保持直到下一次按键。需在交互式 TTY 中运行（`docker exec -it SpaceHero /bin/bash`），并点击该终端窗口取得焦点。

`habitat.path` 为空时使用 Habitat 空舞台（仍需 `habitat-sim`）。默认 `habitat.mode: slam`：仅发 `odom→base_link`（并令轮式里程计贴合真值，避免 `/scan` 相对地图旋转）；Cartographer 负责 `map→odom`/`/map`。导航演示用 `mode: nav`（漂移修正的 `map→odom→base_link` + GT `/map`）。
测试需要已安装的 `automsgs` Python 绑定（仓库内不再附带 `stubs/`）。

## 配置要点

- 噪声字段为高斯 σ；里程计为**积分噪声**（非每拍贴 GT）。
- `habitat.mode`：`nav`（默认）发漂移修正的 `map→odom→base_link`（LaserScan 与 GT 地图对齐）；`slam` 仅发 `odom→base_link`（Cartographer 负责 `map→odom`/`/map`）。
- `robot.tf` → `/tf` + `/tf_static`（主链见上；另附 URDF 传感器外参）。
- `robot.clock` → `/clock`（`builtin_interfaces/Time`）。
- `map.ply`：`source`/`file` 与/或 `channel`；`map.publish: true` 时 `map.grid` → `/map` OccupancyGrid。

| 方向 | 通道（默认） | Proto |
|------|------|-------|
| 入 | `/cmd_vel` | `TwistStamped` |
| 出 | `/scan` `/points` `/camera/*` `/imu` `/odom` | 传感 |
| 出 | `/tf` `/tf_static` | `TFMessage` |
| 出 | `/clock` | `Time` |
| 出 | `/map`（可选 `/map/points` 或 `/map_points`） | `OccupancyGrid` / `PointCloud2` |
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

## Autoviz 点云联调

1. 在 `config/default.yaml`（或副本）中设 `habitat.mode: nav`、`map.enabled: true`、`map.publish: true`，例如 `map.ply.channel: /map_points`。
2. 启动仿真（需已安装 `habitat-sim`）：

```bash
cd /workspace/autonomy
src/autonomy/autosim/scripts/run.sh sim
```

3. 启动 Autoviz，Fixed Frame 设为 `map`。
4. Add Display → **PointCloud2**，Topic 与 `map.ply.channel` 一致（如 **`/map_points`**）。
5. 推荐属性：**Color Transformer** → `Intensity`（channel 填 `intensity`）或 `AxisColor`；大点云可再设 **Decimate**。
6. 地图点云带 `intensity` 字段（到原点距离）；可用 **Size (Pixels)** 调整点大小。

`map.ply.stride` 在发布端降采样（默认 1）；Autoviz 侧 **Decimate** 可叠加使用。
