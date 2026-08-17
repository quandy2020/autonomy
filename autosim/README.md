# autosim

Habitat-Sim 传感器–执行器桥：经 autolink + automsgs 为 autonomy 栈（SLAM / 导航）提供仿真传感与控制接口。

## 职责边界

autosim **仅做桥接**，不包含 SLAM、导航或地图服务：

- **World / IO**：Habitat 场景、Agent 运动、传感器采样
- **Codec**：NumPy / Habitat 观测 → automsgs proto
- **Bridge**：autolink 通道读写、配置、主循环

**非目标（首期不做）：** MapService / NavigationService / LocalizationService、行人 / 多机器人、全局 `/clock`、完整 URDF TF 树。

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

或使用 CLI 入口（后续任务实现 `runner` 后可用）：

```bash
autosim --config config/default.yaml
```

## 默认通道

通道名可在 YAML 中覆盖，默认与 `config/driver`、`config/system/monitor` 对齐：

| 方向 | 通道 | Proto |
|------|------|-------|
| 入 | `/cmd_vel` | `TwistStamped` |
| 出 | `/scan` | `LaserScan` |
| 出 | `/camera/rgb/image_raw` | `Image` |
| 出 | `/camera/depth/image_raw` | `Image` |
| 出 | `/camera/camera_info` | `CameraInfo` |
| 出 | `/imu` | `Imu` |
| 出 | `/odom` | `Odometry` |
| 出 | `/gt/pose` | `PoseStamped`（默认关） |

设计细节见 `docs/superpowers/specs/2026-08-18-autosim-habitat-bridge-design.md`。
