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

评测双目惯性结果时统一使用：

- estimate frame：body。
- 时间关联门限：10 ms。
- 对齐：SE(3) 刚体对齐。
- 尺度：固定为 1，不进行 Sim(3) 尺度校正。
- ATE：每个关联位姿的平移误差范数及其 RMSE。

CPU 使用率报告进程总 CPU 百分比，峰值内存报告 RSS MiB。SLAM 的
LocalMapping 和 LoopClosing 为异步线程，同一序列的关键帧选择与 ATE
可能轻微波动；性能或精度回归判断应至少运行三次并报告中位数。

当前 OpenMP 与内存优化报告位于：

```text
build/atlas_benchmark_results/openmp_memory_final/
```

其中包含 `benchmark_summary.csv`、对齐轨迹 CSV、轨迹/ATE PNG 和
CPU/RSS/ATE 总览图。

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

Evaluation uses rigid SE(3) alignment with fixed scale. It writes
`metrics.json`, `aligned_trajectory.csv`, and
`trajectory_and_ate.png`. The `kitti_00_02.yaml` calibration applies to
sequences 00-02. Use each sequence's `calib.txt` to prepare configurations
for the other calibration groups.
