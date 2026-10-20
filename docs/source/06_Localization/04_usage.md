(localization-usage)=
# 4. 使用指南

三条后端共用 `autonomy.localization`，靠 `--localization_mode` 切换。Cartographer 用 Lua 目录；Atlas / Argus 用 yaml + 话题覆盖。

## Cartographer

完整步骤：[07_cartographer/guide.md](07_cartographer/guide.md)

| 项 | 值 |
|----|-----|
| 配置 | `autonomy/localization/conf/cartographer/` |
| Launch | `localization.launch`、`cartographer_2d.launch` 等 |

```bash
autonomy.localization \
  --localization_mode=cartographer \
  --configuration_directory=autonomy/localization/conf/cartographer \
  --configuration_basename=autosim_2d.lua \
  --save_state_filename=data/maps/cartographer.pbstream
```

纯定位：`--load_state_filename` + `pure_localization_trimmer`（见 Cartographer 指南）。

## Atlas

见 [08_atlas/guide.md](08_atlas/guide.md)。autosim 纯 LIO 示例：

```bash
autonomy.localization \
  --localization_mode=atlas \
  --atlas_config=src/autonomy/autonomy/localization/atlas/config/autosim_lio.yaml \
  --atlas_imu_topic=/imu \
  --atlas_lidar_topic=/points
```

## Argus

见 [06_argus.md](06_argus.md)。离线 bag：[Tools · 数据集](../18_Tools/08_datasets.md)。

## 与导航栈

```text
Driver → Localization → TF (map→odom→base) → Planning
                     → /map（可选）→ Map
```

| 场景 | 后端 |
|------|------|
| 激光 2D 建图/定位 | Cartographer |
| 激光 + IMU LIO/LIVO | Atlas 或 Argus |
| 多相机 / RGB-D | Atlas（选对 yaml mission） |

全栈 frame 名与 [Running](../04_Running/00_guide.md) 一致。

## 故障排查

| 现象 | 排查 |
|------|------|
| Cartographer 无 `/map` | Lua `publish_occupancy_grid`；是否 `echoes_*` 而非错 topic |
| Atlas / Argus 无输出 | `--*_topic`；yaml 外参；LIVO 是否缺图像 |
| TF 断 | 谁在发 `map→odom`；仿真是否只发 `odom→base` |
| 找 AMCL | 未合入；2D 激光用 Cartographer 纯定位或 SLAM |

(atlas-lio-resources)=
## Atlas LIO 资源占用（参考）

`autosim_lio.yaml` + autosim，约 18 min，无 GPU，32 核主机：

| 项 | 数值 |
|----|------|
| CPU | ~0.37 核 |
| RSS / 峰值 | 1.15 / 1.27 GiB |
| 线程 | 57 |

采样命令见 [Atlas 指南](08_atlas/guide.md#资源占用参考)。
