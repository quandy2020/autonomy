# Oxford Spires offline recovery evaluation

## Current status

The current verification uses all 3,401 Oxford raw LiDAR scans and native IMU
samples, without ROS or rosbag. Per-point timestamps are relative to each scan
start. The official calibration is applied in the native sensor frames.

The full-route result still exceeds the requested `ATE <= 0.05 m` and
`RMSE <= 0.05 m` targets. The remaining gap is measured below; earlier
parameter and input-convention experiments are retained as historical evidence.

### Latest full replay and runtime diagnosis (2026-10-27)

Metrics are reported separately: ATE is the mean Euclidean translation error
after rigid alignment, while RMSE is the root mean square of those same
per-pose translation errors. Earlier versions of the evaluator printed RMSE
under both labels; historical `ATE/RMSE` values below therefore mean RMSE.

Voxel-SLAM stores a scan pose every frame but makes a keyframe from a window
of 10 body-frame scans once the movement gate passes. Atlas now follows that
structure. Before this correction it made 3,390 keyframes and 678 submaps.
The latest replay made 68 submaps; final HBA took `2.74 s`, GTSAM pose-graph
optimization over 3,390 scan poses and 8,017 factors took `0.14 s`, and the
backend drained in `9.72 s`. The scan replay took about 2 minutes 23 seconds.
The earlier frame correction transformed deskewed LiDAR keyframe points into
the IMU frame used by HBA. Combining that correction with sparse aggregated
keyframes reduces both geometric error and backend work. iSAM2 itself was not
the main runtime cost.

The optimized text reader retains one-scan-at-a-time streaming and avoids
per-point stream/vector construction. It sorts a scan only if point times are
actually out of order. Timings show each 100-scan block takes about `2.2–5.4 s`
for loading and frontend processing combined; point-cloud text loading itself
is about `0.45–0.96 s` per 100 scans.

The latest full-system result is ATE mean **`0.080274 m`** and translation
RMSE **`0.091935 m`** over 3,389 matched poses (maximum time gap
`0.043527 s`). The paired frontend-only result remains ATE mean `0.173897 m`
and RMSE `0.200509 m`. The previous backend result after only the frame
correction was ATE mean `0.099893 m` and RMSE `0.112230 m`. The remaining
error peaks at `0.419801 m` near initialization and about `0.19 m` near the
end of the route, so the `0.05 m` goal remains unmet. The latest trajectory
and ATE/RMSE plot are in
[`output/trajectory.tum`](output/trajectory.tum) and
[`metrics/ate_rmse_plot.png`](metrics/ate_rmse_plot.png); the matched pose data
and per-pose errors are saved alongside the plot.

The source switches its final global HBA iteration to frontend voxel and
plane thresholds. An Atlas trial of that switch made the resulting graph
ill-conditioned: iSAM2 failed near pose 2,009, and the unoptimized trajectory
scored `0.173897 m` ATE mean / `0.200509 m` RMSE. The switch is not retained.
The remaining source-parity work is to understand those final-iteration
constraints and make the GTSAM solve well-conditioned. Atlas now reports a
failed pose-graph solve as a replay failure instead of silently accepting it.

A full-sequence frontend replay with the initialization-state fix enabled
local BA, disabled backend optimization, and used `0.01` gyroscope noise for
both EKF odometry and local mapping. It produced 3,389 matched poses, maximum
timestamp gap `0.043527 s`, ATE mean `0.173897 m`, and translation RMSE
`0.200509 m`. The Oxford Spires LiDAR
is Hesai; the matching upstream `hesai.yaml` sets both
odometry and local BA gyroscope covariance to `0.01`. Therefore
`evaluation.yaml` now uses `0.01` for both. The previous `0.1` odometry run
borrowed the Avia setting and was not sensor-configuration parity.

The paired full-system replay before the local-window and submap fixes
produced ATE/RMSE `0.731705 m` over the same 3,389 matched timestamps. That
run generated 678 submaps and 58,628 GTSAM factors, then applied hierarchical
BA and top-down PGO. It is retained as a pre-fix result; the latest full replay
and runtime diagnosis are recorded above.

An earlier full replay with global optimization and the dataset-specific local
BA inertial coefficient (`2.5e-5`) reported `142.766991 m` ATE/RMSE. A
source-default coefficient ablation on a continuous 600-scan middle window
changed no-global RMSE from `23.874911 m` to `22.863538 m`, while the full
replay with `1e-4` reported `375.219684 m` RMSE and GTSAM plus fallback
optimization failed. These earlier values predate the latest replay and
window-index fixes and remain history only. That earlier full-sequence estimate
drifted substantially; it does not describe the current result.

Two previous full-sequence results are invalid. The `1,292.41 m` replay applied
duplicate frame conversions. The later `163.02 m` replay kept LiDAR points in
the native frame but fed IMU values that had already been rotated into Oxford's
base frame, while the estimator interpreted them as native IMU-frame values.
The staged IMU values differ from the official raw `imu.csv`; the raw-data
converter now preserves the original sensor values. Both earlier runs are
retained only as debugging history.

The first raw gyroscope sample is approximately
`(0.028443, 0.000639, -0.026632) rad/s`; the old replay input held
`(-0.000261, 0.028497, -0.026581) rad/s`. The latter equals the raw sample
rotated by the official base-from-IMU rotation. This confirms the old input's
frame conversion from the values themselves, independent of its header.

## Dataset and calibration

- Sequence: `2024-03-14-blenheim-palace-05`.
- Input: 3,401 non-ROS point-cloud text scans and 135,429 IMU samples.
- Ground truth: the official `processed/trajectory/gt-tum.txt`, in the Oxford
  `base` frame.
- Calibration: official Oxford files copied to [`sensor.yaml`](sensor.yaml),
  [`cam-lidar-imu.yaml`](cam-lidar-imu.yaml), and [`imu.yaml`](imu.yaml).
- The configured LiDAR-to-IMU extrinsic was independently recomputed from the
  official camera-frame transforms as
  `$T_{imu,lidar}=T_{cam,imu}^{-1}T_{cam,lidar}$`; the resulting rotation and
  translation match `evaluation.yaml` to the stored precision. This rules out
  an accidental inverse of this extrinsic as the cause of the measured
  trajectory error.
- The Oxford platform uses a Hesai QT64. The upstream
  `VoxelSLAM/config/hesai.yaml` sets `cov_gyr: 0.01` for both odometry and
  local BA; `evaluation.yaml` now matches those values. The earlier `0.1`
  odometry trial came from the Avia profile and is not appropriate sensor
  profile parity for this sequence.
- Raw LiDAR points and raw IMU measurements are converted directly from the
  official files with [`prepare_oxford_spires.py`](prepare_oxford_spires.py).
  The converter keeps LiDAR coordinates in native LiDAR axes and IMU readings
  in native IMU axes. `evaluation.yaml` supplies the official LiDAR-to-IMU
  transform.
- The converter derives point-relative seconds from each raw point timestamp
  and the PCD filename timestamp. Replay uses `--point-time-scale 1.0`, enables
  point-wise deskewing, and synchronizes scans by their end time.
- The IMU runs at 400 Hz. `imu.yaml` preserves the published noise densities;
  `evaluation.yaml` uses the estimator covariance convention and tuned values
  from Voxel-SLAM (`0.01` gyroscope, `1.0` accelerometer, `1e-4` bias walks).
  These EKF/preintegration values are not the physical noise densities. The
  source parameters include observability thresholds `15.0` for motion
  initialization and `14.0` for per-scan odometry, plus a degradation limit of
  `10` consecutive failed scans.

## Source comparison and why the target is not met

The frontend-only full-sequence result is ATE mean `0.173897 m` and RMSE
`0.200509 m` over 3,389 matched poses. With the body-frame keyframe fix and
source-style 10-scan aggregation, the backend result is ATE mean `0.080274 m`
and RMSE `0.091935 m`. The intermediate frame-only result was
`0.099893 m` / `0.112230 m`. The previous `0.296609 m` / `0.333204 m`
result used LiDAR-frame keyframe clouds in an IMU-frame HBA problem; it is
historical evidence of that bug. Residual error still varies across the route
after full-trajectory alignment.

The source comparison rules out two tempting explanations. Upstream
`pcl_handler()` sorts each scan by point-relative time before deskewing, and
Atlas `FileReplaySource::LoadScan()` does the same. Oxford point timestamps
also start at the PCD filename timestamp in the inspected scans. The evaluator
converts Atlas IMU poses to Oxford's base frame using the published extrinsics
before alignment. Timestamp interpolation changed a previous full-sequence
score by only about `0.00036 m`. Point order and nearest-timestamp association
do not explain the current `0.041935 m` RMSE gap from the target. The fixed
keyframe frame mismatch was a separate error in the backend input.

The strongest measured frontend gap is local BA effectiveness. Atlas retains
the source's three-trial LM structure and closely matching inertial/voxel
factor equations, but the instrumented continuous 600-scan interval rejected
all `1,761` trials across `587` windows; the latest-pose BA correction stayed
zero. Its 75-dimensional Hessian had a median of 48 nonpositive diagonal
entries. A damping variant still accepted no trials. On the full route, turning
local BA on reduced the no-backend result from `0.213667 m` to `0.204052 m`, a
small gain that leaves the trajectory far outside the target. Passing factor
gradient checks does not prove that the assembled window Hessian, damping,
acceptance test, and state update behave like the upstream solver.

The remaining error may involve frontend tracking and local-map evolution.
Historical route diagnostics show the middle 600 scans drifting from
`1.2975 m` to `7.4952 m` to `11.4688 m` RMSE across consecutive 20-second
sections, while restarting at the middle interval substantially reduces its
error. This indicates state/map history dependence; the current observability
and degradation gates accept updates without detecting this accumulating error.
The next source-parity work should compare, scan by scan, upstream and Atlas
initialization state, point covariance/plane matches, EKF increments, sliding
map recutting, and local-BA Hessian/acceptance decisions on the same raw input.

The latest backend improves full-route RMSE from the paired frontend-only
`0.200509 m` to `0.091935 m`. The strongest remaining errors are at the start
and end of the sequence. A source-matched comparison of initialization,
per-scan odometry, and HBA end-window behavior is needed before changing more
weights or claiming source-equivalent accuracy.

## Previous result (invalid input convention)

The following result came from an earlier replay that used an incorrect Oxford
cloud-frame conversion. It is retained as historical debugging evidence only,
and must not be used as an Atlas or Voxel-SLAM performance result.

| Metric | Result |
|---|---:|
| Input scans | 3,401 |
| Valid output poses | 250 |
| Matched ground-truth poses | 250 / 250 |
| Active output duration (approx. 10 Hz) | 25.0 s |
| First-to-last output timestamp span | 156.27 s |
| Maximum output timestamp gap | 68.22 s |
| **ATE (translation RMSE, matched poses only)** | **24.170053 m** |
| Translation mean / median | 21.317877 m / 21.204963 m |
| Translation P95 / maximum | 46.709709 m / 62.282749 m |

The historical ATE and translation RMSE are the root mean square of 3D position
error over matched poses. The historical plot and associated files are invalid
for the corrected input convention; the corrected result follows below.

## Rejected full replay (incorrect staging)

The full run using the duplicate frame conversions above produced 3,388
timestamp matches and a maximum time gap of `0.043527 s`, but its ATE/RMSE was
`1,292.410545 m`. The raw estimate reached approximately 3,075 m from its origin
over a roughly 338 s sequence. Finalization processed 2,338 keyframes; the
incremental GTSAM update failed, and the top-down pose-graph fallback also
failed. The plot is retained outside the repository at
`/private/tmp/atlas_oxford_spires_full_kdtree/metrics/ate_rmse_plot.png` for
debugging only. Neither the score nor this trajectory is accepted as validation.
Its captured console output is retained as
[`rejected-frame-replay.log`](rejected-frame-replay.log); it is not the log for
the corrected run.

## Invalid full replay (base-frame IMU values)

The run reported below retained raw LiDAR coordinates but used IMU values that
had been rotated into the base frame. Since the inertial estimator expects raw
IMU sensor axes, the `163.021555 m` score is invalid and must not be treated as
an Atlas or Voxel-SLAM performance result. This replay still confirms that the
non-ROS pipeline and final GTSAM optimization complete, but it does not validate
their accuracy.

The invalid run used the raw native-frame point clouds, base-frame IMU values,
deskewing, millisecond point times, and source-matched thresholds. Its metrics
are preserved only to explain why that run was rejected.

| Metric | Result |
|---|---:|
| Matched poses | 3,372 / 3,372 output poses |
| ATE (translation RMSE after rigid alignment) | **163.021555 m** |
| Translation mean / median | 147.859176 m / 137.801027 m |
| Translation P95 / maximum | 286.275348 m / 356.285785 m |
| Final HBA keyframes | 754 |
| GTSAM pose-graph factors / updates | 8,342 / 6 |
| HBA / pose-graph status | completed / committed |

This invalid replay's estimated path and score cannot be used to judge the
corrected raw-frame replay.

### Raw-frame prefix diagnostic

The first 200 scans converted directly from the official raw files produced 195
poses, all matched within `0.05 s`, with maximum time gap `0.021920 s` and ATE/
RMSE `0.490457 m`. One frontend degradation reset still occurred. This result
uses point times in seconds (`--point-time-scale 1.0`) and native IMU values.

## Corrected raw-frame full replay

The corrected replay processed 3,401 raw scans and wrote 3,386 poses. All output
poses matched the official trajectory within `0.05 s`; maximum time difference
was `0.043527 s`, and maximum consecutive output gap was `0.100 s`. The full
run took about 77 minutes.

| Metric | Result |
|---|---:|
| Matched poses | 3,386 / 3,386 output poses |
| ATE (translation RMSE after rigid alignment) | **142.766991 m** |
| Translation mean / median | 127.666876 m / 111.537351 m |
| Translation P95 / maximum | 281.160504 m / 343.092556 m |
| HBA keyframes / residual | 3,316 / 274.96 |
| GTSAM factors / incremental updates | 49,926 / 6 |
| GTSAM and HBA status | committed / completed |

The aligned estimate spans about `315.8 × 458.0 × 26.7 m`; the ground-truth
route spans `102.8 × 110.5 × 4.8 m`. No adjacent output displacement exceeds
`10 m` (six exceed `5 m`), so the large error is not caused by one isolated
pose jump.

| Route segment | Poses | Error under full-trajectory alignment | Error after segment-only alignment |
|---|---:|---:|---:|
| First third | 1,128 | 103.305895 m RMSE | 9.060111 m RMSE |
| Middle third | 1,129 | 195.715217 m RMSE | 189.324436 m RMSE |
| Final third | 1,129 | 110.281848 m RMSE | 29.386012 m RMSE |

The middle segment remains very inaccurate even after a separate rigid
alignment, which points to local trajectory shape drift in addition to a global
pose offset. The next investigation should focus on frontend odometry and
reinitialization around the middle of the sequence, then verify whether HBA
preserves or worsens the local trajectory.

### Frontend versus backend diagnostics

With the same 600 raw scans around the middle of the route, using the earlier
`2.5e-5` coefficient, the full backend produced `23.718836 m` RMSE and the
no-global-mapping replay produced `23.874911 m`. Their estimated path lengths
were `132.818 m` and `133.307 m`. For this 60-second window, global mapping did
not cause the drift. Raising the coefficient to Voxel-SLAM's default `1e-4`
reduced no-global RMSE to `22.863538 m`, but the full run had `375.219684 m`
RMSE, 1,822.1 m estimated path length versus 386.1 m ground truth, and both
GTSAM and top-down fallback optimization failed. This coefficient change is not
retained in the evaluation configuration.

Restarting that same interval near its 40-second mark and replaying only 200
scans produced `1.050848 m` RMSE. In the continuous 600-scan run, the three
consecutive 20-second sections had independently aligned RMSE values of
`1.2975 m`, `7.4952 m`, and `11.4688 m`. The trajectory loses accuracy as the
frontend state and local map evolve. Scan-to-map updates were accepted on most
scans, so the acceptance/observability gate alone does not detect this drift.

The continuous window's local bundle adjustment was instrumented at the solver
boundary. Across 587 windows, all 1,761 LM trials were rejected and the
latest-pose BA correction remained zero. The 75-dimensional state Hessian had a
median of 48 nonpositive diagonal entries (maximum 54), so a trial using
absolute-value damping scales and ten LM attempts was replayed. It still
accepted zero trials and produced the same `23.874911 m` RMSE. That numerical
change was reverted because it did not alter the trajectory. Finite-difference
checks pass for the point-cloud factor gradient and the inertial preintegrator
gradient under the optimizer's state/bias update convention. Local BA rejection
is confirmed, but the damping experiment does not explain the trajectory drift;
the next investigation must trace map recutting and per-scan pose updates across
the same interval.

For the first 200 scans, enabling global mapping gave `0.490457 m` RMSE and
disabling it gave `0.644925 m`. This is a short-prefix diagnostic and does not
show that long-range GTSAM/HBA corrections are accurate; the full replay still
needs a pre-HBA/post-HBA comparison.

The C++ regression suite passed `123/123` in an earlier run; it was not rerun
for the latest runtime and metric-reporting changes. Those unit and integration
tests establish software behavior, but they do not establish trajectory parity.
Direct comparison of the original and Atlas IMU deskew paths also found the
same core integration and point compensation equations; remaining frontend
work should instrument the per-scan predicted-to-corrected state delta and map
match residual distribution on the continuous middle window before changing
estimator equations.

## Recovery changes

- `InertialState::ResetMotionPreservingPose` clears velocity, biases, and
  covariance while preserving timestamp, rotation, position, and gravity.
- `TrajectoryBuilder::SystemReset` uses that operation and records the retained
  world pose as the reinitialization anchor.
- A failed initialization advances the synchronized LiDAR, IMU, and timestamp
  buffers by one scan. It retains the recent state window and retries at half the
  configured window interval instead of repeatedly rebuilding the same window.
- The latest initialization state is rebased to the retained world pose. This
  prevents IMU-only retry periods from moving the trajectory origin without
  bound.
- The degradation counter now follows the original Voxel-SLAM hysteresis:
  failed scans increment it and successful scans decrement it by one.
- Initialization and odometry observability thresholds are configurable, with
  source-matched defaults of `15.0` and `14.0`, respectively.
- The degradation limit now defaults to the original Voxel-SLAM value of `10`.

Focused tests previously passed for window synchronization, pose rebasing,
reset semantics, degradation hysteresis, and YAML configuration round-trip. The
full C++ suite passed (`123/123`). Python regression tests verify native-frame
staging and warning-free alignment metrics for long trajectories.

## Earlier diagnostic parameter experiments (superseded)

These runs used temporary configurations and preceded the corrected Oxford
coordinate staging. They are superseded and must not be compared with the
corrected replay:

| Configuration / input | Poses | Output span | ATE / RMSE |
|---|---:|---:|---:|
| Initialization `5`, odometry `1`, degradation limit `100`; 400-scan prefix | 100 | 9.86 s | 6.095087 m |
| Initialization `5`, odometry `0.01`, degradation limit `50`; 400-scan prefix | 76 | 7.48 s | 2.432800 m |
| Initialization `5`, degradation limit `500`; 400-scan prefix | 386 | 38.39 s | 442.381711 m |
| Window size `20`, default thresholds; complete input | 650 | 237.87 s | 34.687888 m |

Relaxing the degradation limit produces more rows but allows unbounded inertial
drift. The longer window increases output coverage but worsens error. Neither
experiment is accepted as a solution.

## Remaining work

1. Instrument scan-to-map residuals and state increments across continuous
   windows; the current accepted flag does not catch the measured drift.
2. Identify which state or map update diverges as the same interval grows from
   a 1.05 m 20-second replay to an 11.47 m later section.
3. Compare the full trajectory before and after HBA/PGO.
4. Compare the frontend update and local mapping behavior against Voxel-SLAM on
   the same raw scans.
5. Reduce full-sequence ATE/RMSE and recheck all route segments, then improve
   the approximately 4-minute runtime without sacrificing accuracy.

## Reproduction

From the workspace root (`/Users/quandy/Workspace/github/autonomy`):

```bash
python3 atlas/docs/validation/oxford-spires-full/prepare_oxford_spires.py \
  --sequence-directory /Users/quandy/Downloads/oxford_spires_dataset/sequences/2024-03-14-blenheim-palace-05 \
  --output-directory /private/tmp/atlas_oxford_spires_raw_native_frames

atlas/build/current/app/replay \
  --config atlas/docs/validation/oxford-spires-full/evaluation.yaml \
  --dataset /private/tmp/atlas_oxford_spires_raw_native_frames \
  --output /private/tmp/atlas_oxford_full_fast_backend \
  --point-time-scale 1.0

MPLCONFIGDIR=/private/tmp/atlas-matplotlib-cache \
XDG_CACHE_HOME=/private/tmp/atlas-font-cache \
python3 atlas/docs/validation/oxford-spires-full/evaluate_trajectory.py \
  --estimated /private/tmp/atlas_oxford_full_fast_backend/trajectory.tum \
  --ground-truth /Users/quandy/Downloads/oxford_spires_dataset/sequences/2024-03-14-blenheim-palace-05/processed/trajectory/gt-tum.txt \
  --calibration-directory atlas/docs/validation/oxford-spires-full \
  --output-directory atlas/docs/validation/oxford-spires-full/metrics
```

## Artifacts

- Corrected native-frame full replay: [`output/trajectory.tum`](output/trajectory.tum)
- Aligned estimate: [`metrics/estimated_base_aligned.tum`](metrics/estimated_base_aligned.tum)
- Matched ground truth: [`metrics/ground_truth_matched.tum`](metrics/ground_truth_matched.tum)
- Per-pose errors: [`metrics/trajectory_comparison.csv`](metrics/trajectory_comparison.csv)
- Trajectory, error, ATE and RMSE plot: [`metrics/ate_rmse_plot.png`](metrics/ate_rmse_plot.png)
