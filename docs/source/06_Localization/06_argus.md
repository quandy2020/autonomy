# 8. Argus LIO / LIVO

ESIKF 激光–惯性（可选视觉）里程计。进程：`--localization_mode=argus`。

### 8.1 输入 / 输出

| 方向 | 话题 |
|------|------|
| 入 | `/imu` · `/points` · `/camera/rgb/image_raw`（可选） |
| 出 | `/argus/odometry` · `/argus/trajectory` · `/argus/cloud_map` · `/argus/cloud_registered` · `/argus/occupancy` · `/tf` |

`cloud_map`：世界系体素持久图（默认 leaf 0.05 m）。

### 8.2 启动

```bash
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
autolink launch start localization_argus.launch
```

或：

```bash
autonomy.localization \
--localization_mode=argus \
--argus_config=autonomy/localization/conf/argus/livo_autosim.yaml \
--argus_imu_topic=/imu \
--argus_lidar_topic=/points \
--argus_image_topic=/camera/rgb/image_raw
```

话题需与 autosim / `datasets` 发布端一致。

### 8.3 数据源

| 源 | 说明 |
|----|------|
| autosim | `autosim/config/lio.yaml` 等 |
| Oxford Spires | [Tools §8](../18_Tools/08_datasets.md) |

LIO-only：发布端加 `--no-image`，配置侧关闭相机。

### 8.4 源码

`autonomy/localization/argus/` · `argus_node.*` · `launch/localization_argus.launch` · `conf/argus/`
