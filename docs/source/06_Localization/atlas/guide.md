(atlas-guide)=
# Atlas 多传感器 SLAM

`autonomy/localization/atlas` 提供 VO、VIO、LIO、LIVO。激光里程计使用 Faster-LIO（ESKF 与 ivox），回环和定位位姿图使用 Ceres。进程入口是 `autonomy.localization --localization_mode=atlas`。

autosim 上的纯 LIO 配置是 `atlas/config/autosim_lio.yaml`：相机关闭，外参对应 TurtleBot3 的 `imu_link` → `laser_link`。

---

## 启动

终端 1，工作区根目录 `/workspace/autonomy`：

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

日志出现 `AtlasNode started mode=lio mission=mapping` 后，每帧应有 `LIO get cloud`。Ctrl+C 退出；若关闭卡住，再按一次会直接结束进程。

| 话题 | 内容 |
|------|------|
| `/atlas/odometry` | 地图系位姿 |
| `/atlas/trajectory` | 轨迹 |
| `/atlas/cloud_registered` | 当前配准扫描 |
| `/atlas/cloud_map` | 关键帧稠密图，体素 `global_map_voxel`（autosim 为 0.05 m） |
| `/atlas/occupancy` | G2P5 占用栅格 |
| `/atlas/loop_edges` | 里程计链与回环边 |
| `/tf` | `map→odom`，没有轮式里程计时再补 `odom→base_link` |

---

(atlas-resources)=
## 资源占用

2026-09-24 在上述 autosim 负载下采样。进程 pid 191295，已运行约 18 分钟，不占用 GPU。主机 32 核，`MemTotal` 131575972 kB（125.5 GiB）。

| 项 | 数值 | 来源 |
|----|------|------|
| CPU | 36.7% 单核（约 0.37 个核，整机约 1.1%） | `ps` `%cpu`，5 秒内稳定 |
| 常驻内存 RSS | 1.15 GiB（1210684 kB） | `VmRSS` |
| 峰值 RSS | 1.27 GiB（1334944 kB） | `VmHWM` |
| 独占内存 PSS | 1.08 GiB（1128415 kB） | `smaps_rollup` `Pss` |
| 私有脏页 | 0.97 GiB（1015568 kB） | `Private_Dirty` |
| 与其它进程共享 | 147 MiB | `Shared_Clean` + `Shared_Dirty` |
| 虚拟地址 | 4.56 GiB，峰值 4.79 GiB | `VmSize` / `VmPeak` |
| 线程 | 57 | `/proc/<pid>/status` |

`ps` 的 CPU 百分比按单核计算，不是整机 36.7%。内存主体是地图与点云的私有页。稠密图按约 1 秒发布一次时，RSS 会短时抬高几十 MiB。

复现采样（把 `<pid>` 换成 `autonomy.localization` 的进程号）：

```bash
ps -p <pid> -o etime,pcpu,pmem,rss,nlwp,cmd
awk '/^(VmRSS|VmHWM|VmSize|VmPeak|Threads):/{print}' /proc/<pid>/status
awk '/^(Rss|Pss|Private_Dirty|Shared_Clean|Shared_Dirty):/{print}' /proc/<pid>/smaps_rollup
```
