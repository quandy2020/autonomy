# 7. 排查

```bash
autolink launch list
autolink --wait 3 channel list -v
pgrep -af autonomy.localization
autolink channel echo --once /tf
autolink channel echo --once /imu
```

| 现象 | 处理 |
|------|------|
| launch 立刻退出或找不到配置 | 当前目录必须是 `/workspace/autonomy/src/autonomy`；`AUTOLINK_LAUNCH_PATH` 指向 `autonomy/localization/launch` |
| 没有输出 | 输入话题是否在发、时间戳是否单调、图像编码、标定 YAML 是否匹配模式 |
| 双目 / RGB-D 静默 | 左右图或 RGB/深度是否同步；`--atlas-sensor` 是否与 launch 一致；是否先起 Atlas 再 publish |
| VIO 发散 | IMU 单位（rad/s、m/s²）、频率、时间戳、相机–IMU 外参 |
| 地图为空 | 跟踪是否已成功；相机是否在动、场景是否有纹理 |
| Autoviz 只有图像 | 等初始化；看 `/atlas/odometry` 是否有频率 |
| 播放过快丢帧 | `--rate 1.0`，视觉不要 `--rate 0` |
| 再次播放时间错乱 | `launch stop` 再 start；视觉和 Spires 都不要 `--loop` |
| 多个地图 / TF 打架 | `pgrep -af autonomy.localization`，只留一个后端 |
| Argus 无 `/argus/odometry` | `/imu` 与 `/points` 是否在播；单位；是否仍在 MotionInit |
| Autoviz 只有原始雷达、没有绿轨迹 | Fixed Frame 用 `map`；打开 ArgusCloudMap / ArgusTrajectory |
| TUM-VI 打不开 | 目录须含完整 `mav0/`，不完整分片不能跑 |
| TUM RGB-D 找不到 `rgb.txt` | 在可写盘解压 `.tgz`，不要往只读 `/data` 里解 |
| Spires `info` 失败 | 是否有 `raw/imu.csv` 与 `raw/lidar-clouds/`（或 zip）；是否下完并解压 |
| TartanAir 没有 `--dataset` | 发布器未接入 |
| Cartographer 无 `/map` | Lua `publish_occupancy_grid`；仿真是否发 `/scan` |
| 下载脚本写失败 | 把 `AUTONOMY_DATASETS` 指到可写宿主机路径，不要写只读 `/data` |
