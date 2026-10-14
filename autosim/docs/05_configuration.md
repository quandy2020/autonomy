# 05 · 配置

配置真相：`config/*.yaml` + `autosim/config.py` 校验。本节只列稳定约定。

| 相关 | 链接 |
|------|------|
| 话题名 | [06](06_topics.md) |
| Isaac 块 | [07](07_isaac.md) |

---

## 1. 选用哪份 YAML

| 文件 | 用途 |
|------|------|
| `default.yaml` | Habitat 主配置 |
| `fake.yaml` | 无引擎联调 |
| `isaac.yaml` | Isaac + USD |
| `slam.yaml` | `mode: slam`，GT 地图不发 |
| `lio.yaml` / `lightning.yaml` | 开 3D 点云 + IMU，LIO 验证 |
| `bev.yaml` | BEV 相关实验配置 |

启动：`CONFIG=.../xxx.yaml scripts/run.sh sim`。

---

## 2. 根键结构（示意）

```yaml
habitat:
  backend: isaac          # habitat | fake | isaac
  mode: nav               # nav | slam
  path: usd/worlds/...    # 场景路径（相对 autosim/ 或绝对）
  spawn: [x, y, yaw]
  robot: { ... }
  sensors: { lidar_2d, lidar_3d, camera, imu, odom }
  map: { enabled, publish, ply, grid }
  scenario: { enabled, ... }   # Fake / Habitat 过程场景
  isaac: { headless, physics } # 仅 Isaac
```

路径相对 **autosim 工程根**（含 `config/`、`usd/` 的那一层）。

---

## 3. 常用字段

| 字段 | 含义 |
|------|------|
| `robot.profile` | `turtlebot3_burger` / `husky`（填 URDF、USD、轮距） |
| `robot.control_hz` | 控制与积分频率 |
| `robot.tf.publish_map_odom` | 可被 `mode` 覆盖 |
| `sensors.*.rate_hz` | 发布频率门控 |
| `sensors.*.noise` | 高斯 σ；odom 为积分噪声 |
| `map.enabled` | 是否构建真值地图 |
| `map.publish` | 是否发布（slam 下强制 false） |
| `map.grid.resolution` | OccupancyGrid 分辨率 |
| `map.ply.channel` | 点云通道，如 `/overall/map` |
| `map.ply.stride` | 发布端降采样步长 |

---

## 4. 机器人档案

`autosim/robot/profiles.py`：`profile` 非空时合并默认 `urdf`、`usd_robot`、`wheel_separation`、`base_link_height`。  
YAML 显式字段优先于档案默认值。

---

## 5. 校验失败时

`Config.load` 抛 `ValueError`（空通道、重复通道、非法 backend 等）。  
先对照本表与目标 YAML，再查 `autosim/config.py` 中 `validate_*`。
