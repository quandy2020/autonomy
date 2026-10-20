(localization-usage)=
# 4. 使用指南

## 4.1 Cartographer

> 完整步骤：[07_cartographer/guide.md](07_cartographer/guide.md)

| 项 | 值 |
|----|-----|
| 配置 | `autonomy/localization/conf/cartographer/` |
| 二进制 | `autonomy.localization --localization_mode=cartographer` |
| Launch | `localization.launch` / `cartographer_2d.launch` |

```bash
autonomy.localization \
--localization_mode=cartographer \
--configuration_directory=autonomy/localization/conf/cartographer \
--configuration_basename=autosim_2d.lua \
--save_state_filename=data/maps/cartographer.pbstream
```

## 4.2 Atlas

见 [08_atlas/guide.md](08_atlas/guide.md)。

```bash
autonomy.localization \
--localization_mode=atlas \
--atlas_config=src/autonomy/autonomy/localization/atlas/config/autosim_lio.yaml \
--atlas_imu_topic=/imu \
--atlas_lidar_topic=/points
```

## 4.3 Argus

见 [§6](06_argus.md)。离线数据：[Tools §8](../18_Tools/08_datasets.md)。

## 4.4 与导航栈

```
Driver → Localization (Carto / Atlas / Argus) → TF → Planning
→ /map → Map（可选）
```

| 场景 | 推荐 |
|------|------|
| 激光 2D 建图/定位 | Cartographer |
| 激光 + IMU LIO | Atlas / Argus |
| 相机 / 多传感器 | Atlas |

## 4.5 故障排查

| 现象 | 排查 |
|------|------|
| Cartographer 无地图 | 话题名；[Cartographer 排错](07_cartographer/guide.md#9-调参与排错) |
| Atlas / Argus 无输出 | `--*_topic` 与 yaml 外参 |

(atlas-lio-resources)=
## 4.6 资源占用（Atlas LIO）

测量：`autosim_lio.yaml` + autosim，约 18 min，无 GPU。主机 32 核。

| 项 | 数值 |
|----|------|
| CPU | ~0.37 核 |
| RSS / 峰值 | 1.15 / 1.27 GiB |
| 线程 | 57 |

采样命令见 [Atlas](08_atlas/guide.md#资源占用)。
