# autosim

Habitat-Sim **传感器–执行器桥**：经 autolink 发布传感、订阅 `/cmd_vel`，对接现有 SLAM / 导航栈。  
**不实现** Map / Navigation / Localization RPC。

## 模块（每文件一个类）

| 文件 | 类 | 职责 |
|------|-----|------|
| `config.py` | `Config` | YAML 加载与校验 |
| `clock.py` | `Clock` | 仿真时钟 |
| `robot.py` | `Robot` | 差速、里程计、惯性、真值 |
| `sensors.py` | `Sensors` | 2D/3D lidar and camera sampling |
| `simulator.py` | `Simulator` | Habitat / Mock scene + cast_ray |
| `messages.py` | `Messages` | automsgs encode/decode |
| `bridge.py` | `Bridge` | autolink channels |
| `runner.py` | `Runner` | main loop |

## 依赖

- 必需：`numpy`、`PyYAML`、`protobuf`
- 运行时：`autolink` Python、`automsgs` Python 绑定
- 可选：`habitat-sim`（`pip install -e ".[habitat]"`）

## 安装与运行

```bash
cd autosim
pip install -e ".[dev]"
python -m autosim --config config/default.yaml
# 或
python examples/run_bridge.py
```

`habitat.path` 为空且无 `habitat-sim` 时回退 Mock。本地 codec 测试可用 `.deps/python`（见 `tests/conftest.py`），无需完整编译。

## 配置要点

传感器与底盘均在 `habitat` 下：`habitat.robot.*`、`habitat.sensors.*`。  
2D/3D 雷达可并行，各自 `enabled`；采样为 `cast_ray`（Mock 下为确定性假数据）。

| 方向 | 通道（默认） | Proto | 配置位置 |
|------|------|-------|------|
| 入 | `/cmd_vel` | `TwistStamped` | `habitat.robot.cmd_vel` |
| 出 | `/scan` | `LaserScan` | `habitat.sensors.lidar_2d` |
| 出 | `/points` | `PointCloud2` | `habitat.sensors.lidar_3d` |
| 出 | `/camera/rgb|depth|info` | `Image` / `CameraInfo` | `habitat.sensors.camera` |
| 出 | `/imu` | `Imu` | `habitat.sensors.imu` |
| 出 | `/odom` | `Odometry` | `habitat.sensors.odom` |
| 出 | `/gt/pose` | `PoseStamped` | `habitat.robot.truth`（默认关） |

## Habitat 场景

```yaml
habitat:
  path: /data/hm3d/scene.basis.glb
```

## 测试

```bash
cd autosim && pytest tests -v
```

## 联调

先起 autosim，再起 Cartographer / Atlas / navigator。
