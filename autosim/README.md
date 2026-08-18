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

```bash
cd autosim
pip install -e ".[dev]"
python -m autosim --config config/default.yaml
./scripts/run.sh sim
./scripts/run.sh teleop
```

`habitat.path` 为空时使用 Habitat 空舞台（仍需 `habitat-sim`）。真值地图默认关闭；加载真实场景后再设 `habitat.map.enabled: true`。  
测试需要已安装的 `automsgs` Python 绑定（仓库内不再附带 `stubs/`）。

## 配置要点

- 噪声字段为高斯 σ；里程计为**积分噪声**（非每拍贴 GT）。
- `robot.tf` → `/tf` + `/tf_static`（`map→odom→base_link` + URDF 传感器外参）。
- `robot.clock` → `/clock`（`rosgraph_msgs/Clock`）。
- `map.ply`：`file` 与/或 `channel` 至少其一；`map.grid` → `/map` OccupancyGrid。

| 方向 | 通道（默认） | Proto |
|------|------|-------|
| 入 | `/cmd_vel` | `TwistStamped` |
| 出 | `/scan` `/points` `/camera/*` `/imu` `/odom` | 传感 |
| 出 | `/tf` `/tf_static` | `TFMessage` |
| 出 | `/clock` | `Clock` |
| 出 | `/map`（可选 `/map/points`） | `OccupancyGrid` / `PointCloud2` |
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
