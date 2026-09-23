# Atlas offline dataset evaluation

Binary: `autonomy.localization.atlas_dataset`

| --format | --mode | Dataset | Config |
|----------|--------|--------|------|
| euroc | mono / vo / rgb | EuRoC cam0 | `euroc_mono.yaml` |
| euroc | mono_inertial / vio | EuRoC cam0+imu0 | `euroc_mono_inertial.yaml` |
| euroc | stereo | EuRoC cam0+cam1 | `euroc_stereo.yaml` |
| euroc | stereo_inertial | EuRoC cam0+cam1+imu | `euroc_stereo_inertial.yaml` |
| tum_rgbd | rgbd | TUM RGB-D | `tum_rgbd1/2/3.yaml` |
| kitti | mono | KITTI odometry `sequences/XX` | `kitti_mono_00-02/03/04-12.yaml` |
| kitti | stereo | KITTI `image_0`+`image_1` | `kitti_stereo_00-02/03/04-12.yaml` |

## EuRoC

```bash
./bin/autonomy.localization.atlas_dataset \
  --format=euroc --mode=mono_inertial \
  --dataset=/path/MH_01_easy \
  --config=autonomy/localization/atlas/config/euroc_mono_inertial.yaml \
  --traj_out=/tmp/kf.txt

python3 autonomy/localization/atlas/test/app/eval_ate.py \
  --gt_format=euroc \
  --gt /path/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv \
  --est /tmp/kf.txt
```

## TUM RGB-D

```bash
./bin/autonomy.localization.atlas_dataset \
  --format=tum_rgbd --mode=rgbd \
  --dataset=/path/rgbd_dataset_freiburg1_desk \
  --config=autonomy/localization/atlas/config/tum_rgbd1.yaml \
  --traj_out=/tmp/kf.txt

python3 autonomy/localization/atlas/test/app/eval_ate.py \
  --gt_format=tum --fix_scale \
  --gt /path/rgbd_dataset_freiburg1_desk/groundtruth.txt \
  --est /tmp/kf.txt
```

## KITTI odometry

Layout: `sequences/00/{times.txt,image_0/,image_1/}`, GT: `poses/00.txt`

```bash
# Stereo (recommended; metric scale)
./bin/autonomy.localization.atlas_dataset \
  --format=kitti --mode=stereo \
  --dataset=/path/dataset/sequences/00 \
  --config=autonomy/localization/atlas/config/kitti_stereo_00-02.yaml \
  --traj_out=/tmp/kf.txt

# Mono
./bin/autonomy.localization.atlas_dataset \
  --format=kitti --mode=mono \
  --dataset=/path/dataset/sequences/00 \
  --config=autonomy/localization/atlas/config/kitti_mono_00-02.yaml \
  --traj_out=/tmp/kf.txt

python3 autonomy/localization/atlas/test/app/eval_ate.py \
  --gt_format=kitti --fix_scale \
  --gt /path/dataset/poses/00.txt \
  --times /path/dataset/sequences/00/times.txt \
  --est /tmp/kf.txt
```

Calibration: seq 00–02 → `*_00-02.yaml`; 03 → `*_03.yaml`; 04–12 → `*_04-12.yaml`.

Trajectory: TUM `t tx ty tz qx qy qz qw`. Inertial writes **Twb**, vision-only **Twc**.

## One-shot ATE

```bash
# Umeyama self-check (no dataset)
python3 autonomy/localization/atlas/test/app/ate_selfcheck.py

# Settings field coverage (EuRoC platform YAML)
python3 autonomy/localization/atlas/test/app/settings_schema_check.py

# EuRoC MH_01 (needs atlas_dataset binary + dataset)
ATLAS_DATASET_BIN=./bin/autonomy.localization.atlas_dataset \
  ./autonomy/localization/atlas/test/app/run_ate.sh \
  euroc mono_inertial /path/MH_01_easy
```

## Visualization (`--viz`)

When enabled, calls `StartVisualization` and publishes channels from `system/constants.hpp`:

```bash
./bin/autonomy.localization.atlas_dataset \
  --format=euroc --mode=mono_inertial \
  --dataset=/path/MH_01_easy \
  --config=.../euroc_mono_inertial.yaml \
  --traj_out=/tmp/kf.txt \
  --viz --viz_rate=20 --viz_hold
```

- `--viz`: create Autolink Node and publish TF / odom / path / map points / cameras
- `--viz_rate=Hz`: playback throttle (for Autoviz subscribers)
- `--viz_hold`: hold after the run until Ctrl+C
