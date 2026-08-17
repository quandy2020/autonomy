# autosim：Habitat 传感器–执行器桥设计

**日期：** 2026-08-18  
**状态：** 已评审（对话确认）  
**范围：** `autosim/` 新建 Python 包，基于 Habitat-Sim，经 autolink + automsgs 为移动机器人 SLAM / 导航提供仿真传感与控制接口。

## 1. 目标与边界

### 1.1 目标

为现有 `autonomy` 栈（Cartographer / Atlas、navigator 等）提供可替换真机驱动的仿真前端：

- Habitat-Sim 负责场景、Agent 运动与传感器观测
- autolink Python API 负责通道读写
- automsgs proto 作为消息契约
- 模块化、文件名 1–3 词、职责单一

### 1.2 非目标（首期不做）

- 不实现 MapService / NavigationService / LocalizationService
- 不实现行人、多机器人、轮胎动力学
- 不广播全局 `/clock`、不发布完整 URDF TF 树
- 不做端到端 mock SLAM / 导航

### 1.3 已确认决策

| 项 | 选择 |
|----|------|
| 职责 | 传感器–执行器桥（方案 A） |
| 套件 | 差速 + 2D LiDAR + RGB-D + IMU + Odometry |
| 场景 | 默认轻量 `minimal`，可配置官方 Habitat 数据集路径 |
| 真值 | 可选 `/gt/pose`，默认关闭 |
| 架构 | 分层桥（World → IO → Codec → Bridge） |

## 2. 架构

```
cmd_vel ──► drive ──► world.step(agent)
                            │
         ┌──────────────────┼──────────────────┐
         ▼                  ▼                  ▼
      lidar/rgbd           imu/odom         truth?
         │                  │                  │
         └────── codec ─────┴──────► bridge.publish
```

### 2.1 层职责

| 层 | 模块 | 职责 | 禁止依赖 |
|----|------|------|----------|
| World | `world`, `agent` | Habitat 场景加载、重置、步进、Agent 位姿 | proto、autolink |
| IO | `lidar`, `rgbd`, `imu`, `odometry`, `drive`, `truth` | 采样 / 控制 / 可选真值 | autolink |
| Codec | `codec` | NumPy / Habitat 观测 → automsgs proto | Habitat（可选仅类型无关数组） |
| Bridge | `bridge`, `runner`, `clock`, `config` | 配置、时钟、autolink I/O、主循环 | Habitat 内部 API |

## 3. 包结构

```
autosim/
├── README.md
├── pyproject.toml
├── autosim/
│   ├── __init__.py
│   ├── __main__.py     # python -m autosim → runner
│   ├── runner.py       # 入口：配置、Node、主循环
│   ├── world.py        # Habitat 场景
│   ├── agent.py        # 差速 Agent 与传感器挂载
│   ├── drive.py        # Twist → 运动
│   ├── lidar.py        # 2D 扫描
│   ├── rgbd.py         # RGB + Depth
│   ├── imu.py          # 位姿差分 IMU
│   ├── odometry.py     # 轮式里程计（可噪声）
│   ├── truth.py        # 可选真值位姿
│   ├── bridge.py       # autolink Writer/Reader
│   ├── codec.py        # proto 编解码
│   ├── clock.py        # 仿真时间戳
│   └── config.py       # YAML 加载与校验
├── config/
│   └── default.yaml
├── assets/scenes/minimal/
└── examples/
    └── run_bridge.py
```

命名约定：模块文件 1–3 个英文单词，反映专业角色，避免 `utils` / `helpers` 大杂烩。

## 4. 通道与消息

默认通道可与现有 `config/driver`、`config/system/monitor` 对齐，均可 YAML 覆盖。

| 方向 | 通道 | Proto | 默认 |
|------|------|-------|------|
| 入 | `/cmd_vel` | `geometry_msgs.TwistStamped`（亦接受无 header 的 `Twist`） | 事件驱动 |
| 出 | `/scan` | `sensor_msgs.LaserScan` | 10–20 Hz |
| 出 | `/camera/rgb/image_raw` | `sensor_msgs.Image` | 15–30 Hz |
| 出 | `/camera/depth/image_raw` | `sensor_msgs.Image` | 同 RGB |
| 出 | `/camera/camera_info` | `sensor_msgs.CameraInfo` | 与图像同步 |
| 出 | `/imu` | `sensor_msgs.Imu` | 最高 100 Hz |
| 出 | `/odom` | `nav_msgs.Odometry` | 与控制环同频 |
| 出 | `/gt/pose` | `geometry_msgs.PoseStamped` | 默认关闭 |

### 4.1 坐标系

- 机体系：`base_link`；传感器：`laser_link`、`camera_link`、`imu_link`
- 里程计：`odom → base_link`（积分，可含噪声）
- 真值：世界/`map` 下 `base_link` 位姿；**不**替代 `/odom`，避免污染 SLAM

### 4.2 时间

- `clock` 维护仿真时间；出站 `header.stamp` 统一使用
- 首期不广播 `/clock`

## 5. 配置

`config/default.yaml` 主要键：

- `scene`：`backend`（`minimal` | `habitat`）、`path`、`spawn`  
  - `minimal`：仓库内自带、Habitat 可加载的轻量资产（仍走 Habitat-Sim，非第二套物理引擎）  
  - `habitat`：外部官方数据集路径（HM3D / Replica / MP3D 等）
- `robot`：轮距、最大线/角速度、传感器外参
- `rates`：各传感器与控制环频率
- `channels`：通道名映射
- `noise`：odom / imu / lidar 噪声
- `truth.enabled`：默认 `false`
- `habitat`：GPU、分辨率、headless

## 6. 错误处理

| 情况 | 行为 |
|------|------|
| 场景路径无效 | 启动失败，报错含期望路径 |
| Habitat 未安装 | 启动失败，提示依赖 |
| `cmd_vel` 超时 | 速度置零（可配置 watchdog） |
| 单帧传感器失败 | 跳过该帧并日志，主循环继续 |
| 通道名空/重复 | 启动校验失败 |

## 7. 测试

- **单元：** `codec` 字段完整性；`drive` 积分 / 限速 / watchdog；`config` 校验
- **烟雾：** mock `world`（无 Habitat）跑通 bridge 调度
- **手动：** 安装 Habitat 后 `examples/run_bridge.py` 联调真栈

## 8. 与仓库集成

- 新顶层包 `autosim/`（目录已存在，当前为空）
- 依赖：`habitat-sim`（可选运行时）、`autolink` Python、`automsgs` 生成的 Python proto、PyYAML、NumPy
- 不修改现有 C++ SLAM / 导航逻辑；仅通过通道对接
- 可选后续：在 `config/simulation/` 增加指向 autosim 的配置片段（非首期阻塞）

## 9. 成功标准

1. `python -m autosim` 或 `examples/run_bridge.py` 能在 `minimal` 场景启动
2. 订阅 `/cmd_vel` 后机器人在仿真中运动
3. 发布 `/scan`、`/camera/*`、`/imu`、`/odom` 且 proto 可被现有栈解析
4. `truth.enabled=true` 时出现 `/gt/pose`；默认关闭
5. 无 Habitat 时 codec / drive / config 测试仍可通过
