# autosim

Habitat-Sim 传感器–执行器桥：经 autolink + automsgs 为 autonomy 栈（SLAM / 导航）提供仿真传感与控制接口。

## 职责边界

autosim **仅做传感器–执行器桥接**，不包含 SLAM、导航或地图服务：

- **World / IO**：场景加载、Agent 运动、传感器采样
- **Codec**：NumPy / Habitat 观测 → automsgs proto
- **Bridge**：autolink 通道读写、配置、主循环

**非目标（首期不做）：** MapService / NavigationService / LocalizationService RPC、行人 / 多机器人、全局 `/clock`、完整 URDF TF 树。

## 依赖

| 类别 | 包 |
|------|-----|
| 运行时 | `numpy`, `PyYAML`, `protobuf` |
| 可选 | `habitat-sim`（`pip install -e ".[habitat]"`） |
| 集成 | autolink Python API、automsgs 生成的 Python proto（仓库内） |

## 安装

```bash
cd autosim
pip install -e ".[dev]"
```

开发环境含 `pytest`；需要 Habitat 仿真时再安装 `[habitat]` 额外依赖。

## 运行

```bash
python -m autosim --config config/default.yaml
```

或使用示例脚本（等价入口）：

```bash
python examples/run_bridge.py
```

## 默认通道

通道名可在 YAML 中覆盖，默认与 `config/driver`、`config/system/monitor` 及 design spec §4 对齐：

| 方向 | 通道 | Proto | 默认频率 |
|------|------|-------|----------|
| 入 | `/cmd_vel` | `geometry_msgs.TwistStamped`（亦接受 `Twist`） | 事件驱动 |
| 出 | `/scan` | `sensor_msgs.LaserScan` | 10–20 Hz |
| 出 | `/camera/rgb/image_raw` | `sensor_msgs.Image` | 15–30 Hz |
| 出 | `/camera/depth/image_raw` | `sensor_msgs.Image` | 同 RGB |
| 出 | `/camera/camera_info` | `sensor_msgs.CameraInfo` | 与图像同步 |
| 出 | `/imu` | `sensor_msgs.Imu` | 最高 100 Hz |
| 出 | `/odom` | `nav_msgs.Odometry` | 与控制环同频 |
| 出 | `/gt/pose` | `geometry_msgs.PoseStamped` | 默认关闭 |

设计细节见 `docs/superpowers/specs/2026-08-18-autosim-habitat-bridge-design.md`。

## 真值开关

`truth.enabled` 控制是否发布 `/gt/pose`（世界/`map` 下 `base_link` 位姿）。默认 `false`；设为 `true` 时启用，**不**替代 `/odom`，避免污染 SLAM。

```yaml
truth:
  enabled: true
```

## Habitat 场景

`scene.backend: habitat` 时加载外部官方数据集（HM3D / Replica / MP3D 等）：

```yaml
scene:
  backend: habitat
  path: /path/to/scene.glb          # 或 .ply / 数据集场景目录
  spawn: [0.0, 0.0, 0.0]            # x, y, yaw
```

未安装 `habitat-sim` 时，`backend: minimal` 回退至 `MockWorld`（内存仿真，无真实渲染），单元测试与烟雾测试仍可跑通。

## 测试

```bash
cd autosim && pytest tests -v
```

本地无需完整编译 autonomy 栈：`tests/conftest.py` 将 `.deps/python` 加入 `sys.path`，可直接跑 codec / config / drive 等单元测试。

## 与 autonomy 栈联调

1. **先**启动 autosim（发布传感器、订阅 `/cmd_vel`）
2. **再**启动 Cartographer / Atlas / navigator 等定位与导航节点

autosim 不实现 Map / Navigation / Localization RPC；仅通过 autolink 通道对接现有 C++ 栈。
