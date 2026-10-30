# 06 测试与评测

## 单元测试

使用独立 build 目录启用测试：

```bash
cmake -S autonomy/localization/atlas \
  -B build/atlas-tests \
  -DATLAS_BUILD_TESTS=ON
cmake --build build/atlas-tests -j
ctest --test-dir build/atlas-tests --output-on-failure
```

当前测试覆盖 BlockingQueue、Config、相机工厂、ORB 提取、MapBuilder、
Map 和 ConfigLoader。

## EuRoC 功能评测

评测视觉惯性结果时统一使用：

- estimate frame：body。
- 时间关联门限：20 ms。
- 对齐：SE(3) 刚体对齐。
- 尺度：固定为 1，不进行 Sim(3) 尺度校正。
- ATE：每个关联位姿的平移误差范数及其 RMSE。

CPU 使用率报告进程总 CPU 百分比，峰值内存报告 RSS MiB。SLAM 的
LocalMapping 和 LoopClosing 为异步线程，同一序列的关键帧选择与 ATE
可能轻微波动；性能或精度回归判断应至少运行三次并报告中位数。

### 纯单目尺度评测

纯单目没有可观测的绝对尺度，必须显式使用 Sim(3) 对齐；评测器默认仍为
固定尺度 SE(3)，防止 RGB-D、双目和惯性结果被静默缩放。示例会在 SLAM
线程完成最终优化并退出后导出轨迹：

```bash
atlas_mono_euroc \
  config/orb_vocab.fbow \
  config/euroc_mono.yaml \
  /data/slam/EuRoC/data/MH_01_easy \
  /data/slam/EuRoC/data/MH_01_easy/mav0/cam0/data.csv \
  /tmp/atlas_mono_mh01.txt

python3 examples/evaluate_euroc.py \
  /data/slam/EuRoC/data/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv \
  /tmp/atlas_mono_mh01.txt \
  /tmp/atlas_mono_mh01_eval \
  --estimate-frame camera \
  --calibration-yaml \
    /data/slam/EuRoC/data/MH_01_easy/mav0/cam0/sensor.yaml \
  --alignment sim3 --report-fixed-scale
```

`metrics.json` 的顶层 ATE 是 Sim(3) 结果，同时
`fixed_scale_se3` 保留不估计尺度的对照结果。所有 metric 模式继续使用默认
`--alignment se3`。

### MH_01_easy 单目 + IMU 最终复测

最终二进制已直接读取官方 `mav0/cam0/data.csv` 的
`timestamp,filename` 两列格式。MH_01_easy 的独占复测结果为：

- tracking valid：3674/3682。
- 最终活动 map 导出 2483 个位姿；匹配 2461 个；estimate coverage
  99.114%；匹配时长 123.0 s。
- 固定尺度 SE(3) ATE RMSE：0.0478973 m；mean：0.039534 m；
  max：0.462725 m。
- CPU：平均 85.413%，峰值 131%；RSS：平均 405.03 MiB，峰值
  618.91 MiB（原始均值/峰值为 414747/633760 KiB）。

这次运行未发生数值发散，但仍暴露出鲁棒性限制：日志共记录 3 次
active-map reset，分别发生在启动初期跟踪失败、IMU 初始化运动不足以及
首次 VIBA2 期间丢跟踪；该次 VIBA2 约耗时 12.9 s。惯性优化保护阻止了
异常状态写回，但不能把该保护等同于消除跟踪中断。完整轨迹、
`metrics.json`、
PNG、`run.log` 和 `pidstat.log` 位于
`build/atlas_benchmark_results/vio_modes/mono_inertial_euroc_final/MH_01_easy/`。

### V1_01_easy 双目 + IMU 最终复测

V1_01_easy 的最终复测结果为：

- tracking valid：2809/2912；导出轨迹 2809 个位姿，匹配 2789 个，
  coverage 99.288%。
- 固定尺度 SE(3) ATE RMSE：0.0375516 m；mean：0.0345913 m；
  max：0.0883190 m。
- 运行时长 170.912 s；CPU 平均/峰值 95.27%/137%；RSS 平均/峰值
  282.8/453.5 MiB。

该结果是 V1_01_easy 双目 + IMU，不应与上面的 MH_01_easy 单目 + IMU
结果混用。完整产物位于
`build/atlas_benchmark_results/vio_modes/stereo_inertial_euroc_new/`。

当前 OpenMP 与内存优化报告位于：

```text
build/atlas_benchmark_results/openmp_memory_final/
```

其中包含 `benchmark_summary.csv`、对齐轨迹 CSV、轨迹/ATE PNG 和
CPU/RSS/ATE 总览图。

## TUM RGB-D 评测

TUM RGB-D 必须按 `freiburg1`、`freiburg2`、`freiburg3` 使用各自官方内参、
畸变参数、虚拟双目基线和深度比例；特别是 Freiburg2 的
`RGBD.DepthMapFactor` 为 5208，不能复用 Freiburg1 的 5000。归档准备工具
不会删除 `.tgz`，也不会覆盖已存在的同名目录：

```bash
python3 examples/prepare_tum_rgbd.py /data/slam/TUM --dry-run
python3 examples/prepare_tum_rgbd.py /data/slam/TUM
```

runner 在结果目录生成 RGB/depth association 和逐序列 YAML，不修改数据集。
`evaluate_tum_rgbd.py` 按 20 ms 门限进行一对一最近时间关联，固定尺度
SE(3) 对齐，并以 association 行数计算端到端轨迹覆盖率。它输出
`metrics.json`、对齐轨迹 CSV 和轨迹/ATE PNG。

## TUM-VI 鱼眼双目 + IMU 评测

TUM-VI 的 512×512 双鱼眼序列使用 `KannalaBrandt8` 相机模型和
`SensorMode::kStereoInertial`。reader 已支持官方
`mav0/cam0/data.csv` 的 `timestamp,filename` 两列格式，可直接传入；旧的
`timestamps.txt` 单列列表仍保留兼容。

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
atlas_stereo_inertial_tum_vi \
  config/orb_vocab.fbow \
  config/tum_vi_stereo_inertial.yaml \
  /data/slam/TUM-VI/dataset-room1_512_16 \
  /data/slam/TUM-VI/dataset-room1_512_16/mav0/cam0/data.csv \
  /data/slam/TUM-VI/dataset-room1_512_16/mav0/imu0/data.csv \
  /tmp/atlas_tum_vi_room1.txt

python3 examples/evaluate_euroc.py \
  /data/slam/TUM-VI/dataset-room1_512_16/dso/gt_imu.csv \
  /tmp/atlas_tum_vi_room1.txt \
  /tmp/atlas_tum_vi_room1_eval \
  --max-interval-ms 20
```

最终二进制在 `room1` 的独占复测结果：

- tracking valid：2764/2821（97.9794%）。
- 估计轨迹：2693 个位姿；匹配 2628 个；覆盖率 97.5863%。
- 固定尺度 SE(3) ATE RMSE：0.00960643 m；mean：0.008788 m；
  max：0.028056 m。
- CPU：平均 107.859%，峰值 153%；RSS：平均 310.75 MiB，峰值
  493.99 MiB（`pidstat -h -u -r -p PID 1`）。
- `estimate_frame=body`，真值使用 `dso/gt_imu.csv`；未做 Sim(3)
  尺度校正。
- 惯性 FullBA 的有限值保护拒绝了 1 次恶化结果，未将异常状态写回地图。
- 初始化阶段因运动不足发生 1 次 active-map reset；此后完成重建。

完整轨迹、`metrics.json`、对齐 CSV、PNG 和 `pidstat.log` 位于
`build/atlas_benchmark_results/vio_modes/stereo_fisheye_inertial_tum_vi_final/room1/`。

## TartanAir RGB-D 惯性评测

TartanAir V2 的 `depth_lcam_front` 是将 little-endian `float32` 米制
深度打包到四通道 PNG 的数据。离线 reader 解码后将非有限、非正以及
超过 80 m 的天空/无穷深度置零，避免超远地图点破坏惯性优化。

运行与评测命令：

```bash
atlas_rgbd_inertial_tartanair \
  config/orb_vocab.fbow \
  config/tartanair_rgbd_inertial.yaml \
  /data/slam/tartanair_v2/ArchVizTinyHouseDay/Data_easy/P003 \
  /tmp/atlas_tartanair_p003.tum

python3 examples/evaluate_tartanair.py \
  /data/slam/tartanair_v2/ArchVizTinyHouseDay/Data_easy/P003/pose_lcam_front.txt \
  /data/slam/tartanair_v2/ArchVizTinyHouseDay/Data_easy/P003/imu/cam_time.txt \
  /tmp/atlas_tartanair_p003.tum \
  /tmp/atlas_tartanair_p003_eval
```

An active-map reset starts a new, independent SLAM world frame. The TartanAir
runner uses the stable Atlas map id to split those frames without treating a
temporary invalid pose as a reset. It writes each raw map segment beside the
requested output as `.segment-NNN.tum`, preserves the final active-map
optimized output as `.active_optimized.tum`, and records map ids plus the
selected conventional output in `.segments.csv`. The evaluator detects this
manifest, aligns every map segment independently with fixed-scale SE(3), and
reports pose-weighted aggregate ATE, total/longest-segment coverage, and
`global_continuity=false`; it never presents reset maps as one trajectory.

评测器按 1 ms 门限做最近邻时间关联，使用固定尺度 SE(3) 刚体对齐，明确
不做 Sim(3) 尺度拟合；输出 `metrics.json`、`aligned_trajectory.csv`
和 `trajectory_and_ate.png`。

后修复基线：

| 序列 | samples/accepted/valid | 匹配位姿 | 估计/GT 覆盖率 | ATE RMSE | path ratio | CPU 平均/峰值 | Max RSS |
|------|------------------------|----------|----------------|----------|------------|-----------------|---------|
| P003 | 152/152/150 | 150 | 100%/98.6842% | 0.0212131 m | 0.998473 | 58.839%/82% | 244160 KiB |
| P000 | 116/116/108 | 70 | 100%/60.3448% | 0.0202723 m | — | 39.308%/74% | 180164 KiB |

CPU 和 RSS 使用 `pidstat -h -u -r -p PID 1` 每秒采样。P003 的 GT 覆盖
更完整，应作为主结果；P000 的 ATE 必须与 60.3448% 覆盖率一起解读。
完整产物位于
`build/atlas_benchmark_results/vio_modes/rgbd_inertial_tartanair_postfix/`。
TartanAir 属于合成数据集，这些数字不能替代真实传感器标定与精度验收。

## 全数据集顺序回归

`examples/run_all_benchmarks.py` 顺序执行每个 SLAM 进程，避免并行负载改变
跟踪与回环结果。默认矩阵包含 EuRoC 11 条双目惯性、KITTI 00--10、
已下载的全部 TUM RGB-D 归档/序列、TUM-VI room1，以及 TartanAir
ArchVizTinyHouseDay easy P000--P006：

```bash
python3 examples/run_all_benchmarks.py \
  --workspace /workspace/autonomy \
  --data-root /data/slam \
  --output-dir build/atlas_benchmark_results/all_current \
  --resume
```

每条序列分别保存轨迹、`run.log`、`evaluate.log`、`metrics.json` 和
`status.json`；根目录写出可恢复的 `summary.csv`/`summary.json`。
退出码非零表示运行、评测、数据完整性或显式精度门限至少有一项失败。
KITTI 配置会从每条序列自己的 `calib.txt`、`times.txt` 和首张 PNG
生成，不能把 00--02 的内参错误复用到 03--10。

纯单目默认不进入 metric 矩阵。要增加一条 Sim(3) 单目评测及固定尺度对照：

```bash
python3 examples/run_all_benchmarks.py \
  --output-dir build/atlas_benchmark_results/all_with_mono \
  --suites euroc,kitti,tum_rgbd,tum_vi,tartanair,mono \
  --mono-sequence MH_01_easy --resume
```

`--mono-all` 才会运行全部 11 条纯单目序列。ATE、coverage 和 KITTI
相对漂移门限均有命令行参数，报告中必须同时查看 coverage，避免低覆盖率造成
虚假的低 ATE。

runner 会把 `.tgz` 和已解压目录的并集写入
`tum_rgbd_archive_inventory.json`。归档未解压、目录不完整或缺少真值时，
对应序列明确记为 `missing_data`，不会被静默排除，也不会隐式解压。使用
`prepare_tum_rgbd.py` 后以 `--resume` 继续即可。

## 验收门限

- 平均 CPU 必须低于 200%。
- ATE 相对已确认基线的退化不得超过 10%。
- 测试期间不得使用 taskset 或 cgroup 隐藏真实 CPU 消耗。
- 性能评测不得与构建或其他重型任务并行。



## KITTI Odometry stereo evaluation

Atlas supports the KITTI Odometry grayscale stereo layout:
`sequences/XX/image_0`, `image_1`, and `times.txt`. Sequences 00-10
have official ground-truth poses; sequences 11-21 can only be tracked and
exported.

Example for sequence 00:

```bash
atlas_stereo_kitti \
  config/orb_vocab.fbow \
  config/kitti_00_02.yaml \
  /data/kitti/odometry/sequences/00 \
  /tmp/atlas_kitti_00.txt

python3 examples/evaluate_kitti.py \
  /data/kitti/odometry/poses/00.txt \
  /tmp/atlas_kitti_00.txt \
  /tmp/atlas_kitti_00_eval
```

Evaluation reports fixed-scale SE(3)-aligned ATE and official KITTI 100-800 m
segment metrics: relative translation drift (t_rel, percent) and relative
rotation drift (r_rel, degrees per 100 m). It writes `metrics.json`,
`aligned_trajectory.csv`, and `trajectory_and_ate.png`. The `kitti_00_02.yaml` calibration applies to
sequences 00-02. Use each sequence's `calib.txt` to prepare configurations
for the other calibration groups.
