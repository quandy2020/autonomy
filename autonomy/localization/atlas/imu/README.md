# Atlas IMU / Visual-Inertial fusion

## Layout

```
atlas/imu/                         # preintegration, bias, buffer, config
atlas/data/                        # frame/keyframe velocity + bias + temporal IMU links
atlas/initialize/inertial.*        # MAP init: scale / gravity / velocities / bias + apply_to_map
atlas/optimize/imu_g2o/            # velocity/bias vertices, preintegration + bias-walk edges
atlas/optimize/local_bundle_adjuster_inertial.*
atlas/optimize/global_bundle_adjuster_inertial.*   # after loop closure
```

## Enable

```yaml
IMU:
  enabled: true
  frequency: 200.0
  noise_acc: 0.02
  noise_gyro: 0.0017
  random_walk_acc: 0.003
  random_walk_gyro: 0.00017
  gravity_magnitude: 9.81
  time_offset: 0.0          # imu_time = image_time + offset
  init_min_keyframes: 10
  R_c_b: [...]              # IMU -> camera (see EuRoC_mono_inertial.yaml)
  t_c_b: [...]
Mapping:
  use_inertial: true
```

Runtime:

```bash
autonomy.localization --localization_mode=atlas \
  --atlas_config=.../EuRoC_mono_inertial.yaml \
  --atlas_imu_topic=/imu0
```

Or call `system::feed_imu(t, acc, gyro)` directly.

## Pipeline

1. Visual initialization (existing)
2. Keyframe insertion builds temporal IMU chain + preintegration (raw samples retained)
3. `initialize::inertial` solves scale/gravity/velocities/bias and scales the map
4. Tracking predicts with IMU; local BA adds IMU + bias-walk edges (analytic Jacobians)
5. After VI-BA, bias write-back triggers `preintegrator::update_bias` (reintegrate if `|db|` large)
6. Loop closure runs visual GBA then inertial global refine
7. Map IO: msgpack/JSON and SQLite store velocity / bias / `imu_prev_id`

## EuRoC offline

```bash
autonomy.localization.euroc_vio \
  --dataset=/data/EuRoC/MH_01_easy \
  --config=.../EuRoC_mono_inertial.yaml \
  --vocab=.../orb_vocab.fbow \
  --traj_out=/tmp/kf_tum.txt

python3 atlas/example/euroc/eval_euroc_ate.py \
  --gt /data/EuRoC/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv \
  --est /tmp/kf_tum.txt
```
