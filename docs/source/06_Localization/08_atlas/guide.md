(atlas-guide)=
# Atlas 多传感器 SLAM

`autonomy/localization/atlas`：VO / VIO / LIO / LIVO。LIO 基于 Faster-LIO（ESKF + ivox）；回环用 Ceres。入口：`--localization_mode=atlas`。

autosim 纯 LIO：`atlas/config/autosim_lio.yaml`（相机关、TurtleBot3 `imu_link`→`laser_link` 外参）。

---

## 启动

终端 1：

```bash
CONFIG=src/autonomy/autosim/config/lightning.yaml \
src/autonomy/autosim/scripts/run.sh sim
```

终端 2：

```bash
export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
export LD_LIBRARY_PATH=/workspace/autonomy/build/autonomy/lib:${LD_LIBRARY_PATH:-}
export AUTOLINK_PATH=/workspace/autonomy/src/autonomy/autolink
autonomy.localization \
--localization_mode=atlas \
--atlas_config=src/autonomy/autonomy/localization/atlas/config/autosim_lio.yaml \
--atlas_imu_topic=/imu \
--atlas_lidar_topic=/points
```

日志：`AtlasNode started mode=lio mission=mapping`，帧内有 `LIO get cloud`。

| 话题 | 内容 |
|------|------|
| `/atlas/odometry` | 地图系位姿 |
| `/atlas/trajectory` | 轨迹 |
| `/atlas/cloud_registered` | 当前配准扫描 |
| `/atlas/cloud_map` | 关键帧稠密图（体素 `global_map_voxel`，autosim 0.05 m） |
| `/atlas/occupancy` | G2P5 占用栅格 |
| `/atlas/loop_edges` | 里程计 / 回环边 |
| `/tf` | `map→odom`（无轮式里程计时补 `odom→base_link`） |

---

(atlas-resources)=
## 资源占用

2026-09-24，上述负载约 18 min，无 GPU。

| 项 | 数值 |
|----|------|
| CPU | ~0.37 核（单核 36.7%） |
| RSS / 峰值 | 1.15 / 1.27 GiB |
| PSS | 1.08 GiB |
| 线程 | 57 |

```bash
ps -p <pid> -o etime,pcpu,pmem,rss,nlwp,cmd
awk '/^(VmRSS|VmHWM|VmSize|VmPeak|Threads):/{print}' /proc/<pid>/status
```
