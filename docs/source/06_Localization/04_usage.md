(localization-usage)=
# 4. 使用指南

本章按 **Cartographer / Lightning / Atlas** 与 **AMCL（待集成）** 分节说明。

---

## 4.1 Cartographer

> 完整文档：[cartographer/guide.md](cartographer/guide.md)

| 项 | 值 |
|----|-----|
| 配置目录 | `autonomy/localization/conf/cartographer/` |
| 二进制 | `autonomy.localization --localization_mode=cartographer` |
| Launch | `localization.launch` / `localization_cartographer.launch` |

```bash
autonomy.localization \
  --localization_mode=cartographer \
  --configuration_directory=autonomy/localization/conf/cartographer \
  --configuration_basename=autosim_2d.lua \
  --save_state_filename=data/maps/cartographer.pbstream
```

---

## 4.2 Lightning LIO

| 项 | 值 |
|----|-----|
| 配置 | `autonomy/localization/conf/lightning/` |
| 二进制 | `--localization_mode=lightning` |
| Launch | `localization_lightning.launch` |

```bash
autonomy.localization \
  --localization_mode=lightning \
  --lightning_config=autonomy/localization/conf/lightning/autosim.yaml \
  --lightning_imu_topic=/imu \
  --lightning_lidar_topic=/points
```

---

## 4.3 AMCL 配置（待集成）

### 4.3.1 配置入口

| 项 | 值 |
|----|-----|
| 配置文件 | `config/localization/localization.lua` |
| 子配置 | `config/localization/amcl/amcl.lua` |
| Proto | `autonomy.localization.proto.AmclOptions`（规划中） |

### 4.3.2 依赖话题

| 话题 | 类型 | 方向 |
|------|------|------|
| `/map` | OccupancyGrid | 订阅 |
| `/scan` | LaserScan | 订阅 |
| `/odom` | Odometry | 订阅（可选） |
| TF `map→odom` | Transform | 发布 |

---

## 4.4 与导航栈集成

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   Driver    │────►│ Localization │────►│  Transform  │
│ camera/scan │     │ Carto/Light  │     │  map↔base   │
└─────────────┘     └──────┬───────┘     └──────┬──────┘
                           │                     │
                           ▼                     ▼
                    ┌─────────────┐       ┌─────────────┐
                    │ Map (可选)   │       │  Planning   │
                    └─────────────┘       └─────────────┘
```

| 集成点 | Cartographer / Lightning | AMCL |
|--------|--------------------------|------|
| TF 输出 | `map→odom` | 原生支持 |
| 静态地图 | OccupancyGrid / pbstream | 消费 `/map` |
| 传感器 | 激光 (+ IMU) | 激光 + 里程计 |

---

## 4.5 算法选型

| 场景 | 推荐 | 理由 |
|------|------|------|
| 激光 2D 建图/定位 | **Cartographer** | 默认后端 |
| 激光 + IMU LIO | **Atlas** 或 Lightning | Atlas 为当前 LIO 路径；Lightning 仍可单独启动 |
| 已知 2D 地图 + 激光 | AMCL | 成熟、低算力（待集成） |

---

## 4.6 故障排查

| 现象 | 可能原因 | 排查 |
|------|----------|------|
| Cartographer 无地图 | 话题名错误 | 见 [Cartographer §11](cartographer/guide.md#11-故障排查) |
| Lightning 无输出 | IMU/点云话题或外参 | 检查 `--lightning_*_topic` 与 yaml |
| Atlas 无输出 | IMU/点云话题或外参 | 检查 `--atlas_*_topic` 与 `autosim_lio.yaml` |
| AMCL 不工作 | 尚未集成 | 使用 Cartographer / Lightning / Atlas |

---

(atlas-lio-resources)=
## 4.7 资源占用（Atlas LIO）

测量对象：`autonomy.localization --localization_mode=atlas`，配置 `atlas/config/autosim_lio.yaml`（激光 + IMU，建图）。负载来自 autosim（`autosim/config/lightning.yaml`，Bistro，TurtleBot3，`/imu` + `/points`）。进程不占用 GPU。

采样时间 2026-09-24，进程已运行约 18 分钟。主机 32 核，内存 125.5 GiB。`ps` 的 CPU 百分比是相对单核的。

| 项 | 数值 |
|----|------|
| CPU | 36.7% 单核（约 0.37 个核，整机约 1.1%） |
| 常驻内存 RSS | 1.15 GiB |
| 峰值 RSS | 1.27 GiB |
| 独占内存 PSS | 1.08 GiB |
| 私有脏页 | 0.97 GiB |
| 虚拟地址 | 4.56 GiB（峰值 4.79 GiB） |
| 线程 | 57 |

条件、采样方法和话题说明见 [Atlas 资源占用](atlas/guide.md)。
