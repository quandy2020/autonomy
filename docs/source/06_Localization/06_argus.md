# 6. Argus LIO / LIVO

Argus 是仓库内的 ESIKF 激光–惯性（可选视觉）里程计后端，进程开关 `--localization_mode=argus`。适合 **IMU + 点云** 为主、需要 LIO 或 LIVO 的场景；Oxford Spires 等离线序列有配套脚本。 Argus 用 ESIKF 融合 IMU 与点云（LIVO 可选相机），适合 `--localization_mode=argus` 的 LIO/LIVO 部署。上线前对齐输入话题、输出 odom/TF 与 map/planning 使用的 frame。

## 输入 / 输出

| 方向 | 话题（autosim 典型名） |
|------|------------------------|
| 入 | `/imu`、`/points`、`/camera/rgb/image_raw`（LIVO 需要） |
| 出 | `/argus/odometry`、`/argus/trajectory`、`/argus/cloud_map`、`/argus/cloud_registered`、`/argus/occupancy`、`/tf` |

`cloud_map` 为世界系体素持久图（默认 leaf 约 0.05 m）。LIO-only：发布端 `--no-image`，yaml 里关闭相机相关块。

## 启动

Launch：

```bash
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
autolink launch start localization_argus.launch
```

或直接 CLI（话题须与仿真/数据集一致）：

```bash
autonomy.localization \
  --localization_mode=argus \
  --argus_config=autonomy/localization/conf/argus/livo_autosim.yaml \
  --argus_imu_topic=/imu \
  --argus_lidar_topic=/points \
  --argus_image_topic=/camera/rgb/image_raw
```

## 和 Atlas 怎么选

两者都能做 autosim LIO。Argus 侧 ESIKF LIVO 实现、Spires 批量评测脚本更集中；Atlas 侧 yaml mission 与 `/atlas/*` 话题、回环/Ceres 栈更统一。同一机器人上 **只起一个** 定位后端。

## 数据源

| 源 | 说明 |
|----|------|
| autosim | `autosim/config/lio.yaml` 等场景配置 |
| Oxford Spires | [Tools · 数据集](../18_Tools/08_datasets.md)、`argus/scripts/run_spires_full_eval.sh` |

## 常见坑

| 现象 | 先查 |
|------|------|
| 无里程计 | IMU/点云 topic；外参与时间同步 |
| LIVO 无图像仍开相机 | 改 LIO yaml 或关图像 topic |
| TF 与 planning 不一致 | `map`/`base_link` 全栈 conf |

## 源码位置

`autonomy/localization/argus/` · `argus_node.*` · `launch/localization_argus.launch` · `conf/argus/`
