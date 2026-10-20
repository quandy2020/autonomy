(atlas-guide)=
# Atlas 多传感器 SLAM

## 背景

Atlas 在进程里通过 `--localization_mode=atlas` 启动，yaml 决定 mission（LIO、LIVO、VIO 等）和传感器外参。库目录：`autonomy/localization/atlas/`（VO/VIO 前端、LIO 基于 Faster-LIO 思路、回环用 Ceres）。


## 步骤

### 1. 什么时候用 Atlas

- 需要 **相机 + IMU + 激光** 或多种 mission 切换，而 Cartographer 覆盖不了。
- autosim / 实车已有 Atlas yaml，且话题名能对上。
- 和 Argus 二选一时：Atlas 更贴「Atlas 配置树 + `/atlas/*` 话题」；Argus 见 [06 Argus](../06_argus.md)（ESIKF、Spires 脚本）。

**注意：** `localization_atlas.launch` 默认常指向带相机的 `fusion_livo.yaml`。**autosim 纯 LIO 验证** 请显式指定 `autosim_lio.yaml`（相机关闭、TurtleBot3 `imu_link`→`laser_link` 外参），不要和 LIVO launch 混用。

### 2. autosim 纯 LIO 最短路径

终端 1（仿真）：

```bash
CONFIG=src/autonomy/autosim/config/lightning.yaml \
src/autonomy/autosim/scripts/run.sh sim
```

`lightning.yaml` 只是 AutoSim **场景名**，不是定位后端。

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

日志里应有 `AtlasNode started mode=lio mission=mapping`，帧日志含 `LIO get cloud`。词袋/相机相关告警在纯 LIO 配置下通常可忽略。

### 3. 话题与 TF

| 话题 | 内容 |
|------|------|
| `/atlas/odometry` | 地图系位姿 |
| `/atlas/trajectory` | 轨迹 Path |
| `/atlas/cloud_registered` | 当前帧配准点云（`map`） |
| `/atlas/cloud_map` | 关键帧稠密图（体素如 `global_map_voxel`，autosim 常 0.05 m） |
| `/atlas/occupancy` | G2P5 占用栅格 |
| `/atlas/loop_edges` | 里程计 / 回环边 |
| `/tf` | `map→odom`；无轮速里程计时可能补 `odom→base_link` |

下游 planning / control 只认全栈统一的 `map`、`base_link` 名。

### 4. 配置要点

- **`--atlas_config`**：传感器标定、mission、体素、回环开关等，改仿真机器人时必须改外参块。
- **`--atlas_imu_topic` / `--atlas_lidar_topic` / `--atlas_image_topic`**：覆盖 yaml 默认；与 bag/仿真发布名不一致时 **只会空转**。
- 库内工厂模式含 `vo` / `vio` 等，与 node 侧 LIO/LIVO mission 的对应关系以当前 yaml 和日志 `mode=` 为准。

### 5. 常见坑

| 现象 | 先查 |
|------|------|
| 无 `/atlas/odometry` | topic 名；yaml 是否 LIVO 却未发图像 |
| 开头一次 sync 失败然后正常 | 常见；若持续无 `LIO get cloud` 查 IMU 频率与点云时间戳 |
| 和导航 TF 对不齐 | `map`/`odom`/`base_link` 是否与 planner、control conf 一致 |
| CPU/RSS 偏高 | 回环、地图体素、相机分辨率；见下方资源占用 |

(atlas-resources)=
### 6. 资源占用（参考）

autosim + `autosim_lio.yaml`，约 18 min，无 GPU，32 核主机（2026-09-24 单次测量）：

| 项 | 数值 |
|----|------|
| CPU | ~0.37 核 |
| RSS / 峰值 | 1.15 / 1.27 GiB |
| 线程 | 57 |

```bash
ps -p <pid> -o etime,pcpu,pmem,rss,nlwp,cmd
awk '/^(VmRSS|VmHWM|VmSize|VmPeak|Threads):/{print}' /proc/<pid>/status
```

库结构与 ORB 前端说明见源码 `atlas/README.md`、`atlas/docs/design/`。
