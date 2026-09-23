(localization-usage)=
# 4. 使用指南

本章按 **Cartographer / Lightning** 与 **AMCL（待集成）** 分节说明。

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
| 激光 + IMU LIO | **Lightning** | 独立 LIO 路径 |
| 已知 2D 地图 + 激光 | AMCL | 成熟、低算力（待集成） |

---

## 4.6 故障排查

| 现象 | 可能原因 | 排查 |
|------|----------|------|
| Cartographer 无地图 | 话题名错误 | 见 [Cartographer §11](cartographer/guide.md#11-故障排查) |
| Lightning 无输出 | IMU/点云话题或外参 | 检查 `--lightning_*_topic` 与 yaml |
| AMCL 不工作 | 尚未集成 | 使用 Cartographer / Lightning |
