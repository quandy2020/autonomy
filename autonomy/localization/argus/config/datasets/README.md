# Argus replay dataset format

Argus replays an intentionally transport-independent text format. ROS is not a
runtime dependency: a converter may read a ROS bag, but the generated dataset
and the SLAM runner use only ordinary files.

## Directory layout

```text
dataset/
  imu.txt
  scans.txt
  scans/
    000000.txt
    000001.txt
```

`imu.txt` contains one sample per line:

```text
# timestamp_seconds wx wy wz ax ay az
1690000000.000000000 0.0 0.0 0.0 0.0 0.0 9.81
```

Angular velocity is in radians per second and linear acceleration is in meters
per second squared. Timestamps must be finite and use one clock domain.

`scans.txt` associates each scan start time with a path relative to the dataset
directory:

```text
# timestamp_seconds relative_path
1690000000.000000000 scans/000000.txt
```

Each scan file contains:

```text
# x_m y_m z_m intensity relative_time_seconds
1.0 2.0 3.0 42.0 0.000125
```

The relative time is measured from the scan start and must be non-negative.
Set `voxel.points_have_timestamps: false` only for sensors or prepared datasets
that do not provide meaningful per-point timing.

Blank lines and lines beginning with `#` are ignored. `FileReplaySource`
sorts IMU samples and scan references by timestamp and rejects malformed rows
or missing scan files.

## Converting a ROS bag offline

The optional converter requires Python and the `rosbags` package. It is a data
preparation tool only and does not add ROS to Argus:

```bash
python3 app/tools/convert/rosbag_to_voxel_slam.py \
  /path/to/input.bag /path/to/dataset \
  --lidar-topic /points_raw --imu-topic /imu/data
```

Repeat either topic option when a bag uses multiple input topics. Verify field
units and the LiDAR-to-IMU extrinsic configuration before comparing accuracy.

## Replay and evaluation

Build Argus in `SpaceHero`, then run:

```bash
./build/app/replay --config config/voxel_slam_default.yaml \
  --dataset /path/to/dataset --output /path/to/results/default
```

The output trajectory uses TUM format. A matrix of configurations can be run
with `app/tools/eval/run_config_matrix.py`. When a timestamp-compatible TUM
ground-truth trajectory is available, aggregate aligned ATE RMSE with:

```bash
python3 app/tools/eval/aggregate_results.py /path/to/results \
  --ground-truth /path/to/groundtruth.tum \
  --output /path/to/results/summary.yaml
```

For parity work, replay the same source data and calibration through the
reference Voxel-SLAM and Argus, retain both trajectories and runtime logs, and
compare initialization success, processed scan count, ATE, loop closures,
saved-session reload, wall time, and peak memory.
