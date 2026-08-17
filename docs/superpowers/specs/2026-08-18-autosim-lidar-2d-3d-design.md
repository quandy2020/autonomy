# autosim：并行 2D / 3D LiDAR（cast_ray）设计

**日期：** 2026-08-18  
**状态：** 待用户审阅  
**范围：** 将 `habitat.sensors.lidar` 拆为可并行的 `lidar_2d` / `lidar_3d`；以 Habitat `cast_ray` 采样；经 automsgs 发布 `LaserScan` 与 `PointCloud2`。

## 1. 目标与边界

### 1.1 目标

- 2D 与 3D 雷达可同时启用，各自通道、帧、频率、几何参数独立
- 2D → `sensor_msgs.LaserScan`（默认 `/scan`）
- 3D → `sensor_msgs.PointCloud2`（默认 `/points`，字段仅 `x,y,z` float32）
- 采样语义为场景射线查询（`Simulator.cast_ray`），非深度图近似
- Mock 模式提供确定性假数据，便于无 `habitat-sim` 的 CI

### 1.2 非目标（本轮不做）

- Habitat C++ Lidar 插件或 `LidarSensorSpec`（上游无内置）
- 点云 intensity / ring / per-point timestamp 字段
- GPU 批量射线或异步采样
- 将现有 RGB-D 深度中心行逻辑保留为可选 `source`（一律改用 raycast）

### 1.3 已确认决策

| 项 | 选择 |
|----|------|
| 并行性 | 可并行：`lidar_2d` / `lidar_3d` 各自 `enabled` |
| 数据源意图 | Habitat 原生射线（实现为 `cast_ray`） |
| 2D 源 | 与 3D 一致：水平单环射线 |
| 方案 | 方案 1：`cast_ray` 网格 |

## 2. 配置

替换 `habitat.sensors.lidar`：

```yaml
sensors:
  lidar_2d:
    enabled: false
    channel: /scan
    frame: laser_link
    rate_hz: 10.0
    angle_min: -3.14159
    angle_max: 3.14159
    range_min: 0.1
    range_max: 30.0
    num_beams: 360
    noise: 0.0

  lidar_3d:
    enabled: false
    channel: /points
    frame: lidar_link
    rate_hz: 10.0
    horizontal:
      angle_min: -3.14159
      angle_max: 3.14159
      num_beams: 360
    vertical:
      angle_min: -0.26
      angle_max: 0.26
      num_rings: 16
    range_min: 0.1
    range_max: 30.0
    noise: 0.0
```

### 2.1 校验规则

- `REQUIRED_SENSOR_KEYS`：`lidar_2d`、`lidar_3d`、`camera`、`imu`、`odom`
- `enabled` 必填（bool）
- 通道名非空；与 `cmd_vel` / 相机 / imu / odom / truth 及彼此不重复
- `lidar_2d.num_beams >= 1`；`lidar_3d.horizontal.num_beams >= 1` 且 `vertical.num_rings >= 1`
- `angle_max >= angle_min`；`range_max > range_min`

### 2.2 channel_map

| 逻辑键 | 配置路径 |
|--------|----------|
| `scan` | `lidar_2d.channel` |
| `points` | `lidar_3d.channel` |

## 3. 采样几何

传感器原点取当前 Agent 平面位姿 `(x, y, yaw)`，高度默认 `0`（与现有平面模型一致；后续若加 URDF 再挂载高度）。

坐标系约定（与现有平面桥一致）：

- 桥平面：`x` 前、`y` 左、`yaw` 绕竖直轴
- Habitat：Y-up，平面 `y` 映射世界 `z`

### 3.1 2D

对方位角 \(\theta_i \in [\texttt{angle\_min}, \texttt{angle\_max}]\) 均匀取 `num_beams` 束，在水平面发射射线：

- 命中：距离 \(r = \lVert p_{\mathrm{hit}} - p_{\mathrm{origin}}\rVert\)，再 clip 到 `[range_min, range_max]`
- 未命中：填 `range_max`（与常见 LaserScan 无效读数习惯一致）
- 可选 `noise`：高斯加噪后再 clip（`noise == 0` 跳过）

输出：`float32[num_beams]` → `LaserScan`

### 3.2 3D

水平角 \(\phi_j\)（`horizontal.num_beams`）× 仰角 \(\psi_k\)（`vertical.num_rings`）：

- 方向：由 \((\phi_j + \mathrm{yaw}, \psi_k)\) 构成单位方向（Habitat 世界系）
- 命中：将命中点变换到传感器系（`lidar_link`），写入 `(x,y,z)`
- 未命中：跳过（首版 `is_dense=false`，不填哨兵点）
- 可选 `noise`：对命中点沿射线方向扰动距离后再写坐标

输出：`Nx3 float32` → `PointCloud2`（`height=1`, `width=N`, `point_step=12`, fields `x,y,z`）

### 3.3 Mock

- 2D：全束 `0.5 * range_max`
- 3D：`num_rings` 层、每层 `num_beams` 点的单位半径圆环（缩放可固定），保证 shape 可测

## 4. 模块改动

| 模块 | 改动 |
|------|------|
| `config/default.yaml` | 拆分 `lidar_2d` / `lidar_3d` |
| `config.py` | 校验、`channel_map` |
| `simulator.py` | `laser_ranges(...)`、`lidar_points(...)`；移除对外 `depth_ring`（或改为私有兼容后删除） |
| `sensors.py` | `sample_laser` / `sample_points` 委托 simulator |
| `messages.py` | `encode_point_cloud2` |
| `bridge.py` | writers 含 `points` |
| `runner.py` | 双雷达独立 elapsed / `enabled` 门控 |
| 测试 / README | 配置样例、通道表、Mock 断言 |

`Runner` 行为：

- `enabled: false`：不创建对应 writer 计时逻辑（或创建但不发布；推荐不发布且不采样，省 CPU）
- 两者均可每 tick 独立触发

## 5. 测试计划

- `test_config`：加载默认 YAML；缺键 / 空通道 / 重复通道失败
- `test_simulator`：Mock 下 2D shape、3D `Nx3` 与 `N <= beams*rings`
- `test_messages`：`PointCloud2` 字段与 `point_step`
- `test_runner_smoke`：分别 / 同时 enabled 时发布对应通道

## 6. 风险与缓解

| 风险 | 缓解 |
|------|------|
| `cast_ray` 高分辨率 CPU 贵 | 默认 `enabled: false`；文档提示降低 beams/rings |
| `.deps` 可能缺 `PointCloud2` Python stub | 测试用本地 stub 补齐，与现有 LaserScan 策略一致 |
| 平面高度为 0 与真机挂载不符 | 文档标明；后续 URDF/外参再扩展 `position` |

## 7. 与主桥设计关系

补充 `2026-08-18-autosim-habitat-bridge-design.md` 中「2D LiDAR」条目：首期套件升级为 **可并行 2D LaserScan + 3D PointCloud2**，采样由深度中心行改为 `cast_ray`。本文件为该增量的权威说明。
