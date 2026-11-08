# Oxford Spires trajectory recovery

## Implemented

1. Matched initialization/odometry observability gates and the degradation
   limit to Voxel-SLAM defaults: `15.0`, `14.0`, and `10`.
2. Added pose-preserving inertial reset and synchronized initialization-window
   sliding after failed motion initialization.
3. Added a raw Oxford data converter that preserves LiDAR coordinates in the
   LiDAR frame, preserves IMU measurements in native IMU axes, and converts
   epoch point timestamps to relative seconds.
4. Added frame validation to the generic staging script so it rejects IMU data
   marked as base-frame measurements.
5. Added regression checks for raw sensor frame preservation, point-time
   conversion, and warning-free trajectory alignment.

## Root cause found

The prior Oxford text input contained IMU samples rotated into the base frame.
Its first gyroscope sample was approximately `(-0.000261, 0.028497, -0.026581)`
rad/s, while the official raw CSV contains `(0.028443, 0.000639, -0.026632)`
rad/s. The former is the latter rotated by the official base-from-IMU rotation.
Atlas was using that base-frame sample as though it were in the IMU sensor frame.

The resulting full replay reported `163.021555 m` ATE/RMSE, with repeated
initialization failures and resets. That score is invalid as an algorithm
result. The earlier `1,292.410545 m` replay also had duplicate frame conversion
and is invalid.

## Raw-frame diagnostic

The first 200 scans converted directly from official raw files produced 195
poses. All poses matched ground truth within `0.05 s`; maximum timestamp gap
was `0.021920 s`; ATE/RMSE was `0.490457 m`. The run still experienced one
frontend degradation reset. This confirms the IMU frame mismatch was a major
accuracy problem, but does not establish full-sequence accuracy.

The full 3,401-scan raw-frame replay completed with deskew enabled, point-time
scale `1.0`, and source-matched thresholds. It produced 3,386 output poses,
3,316 HBA keyframes, 49,926 GTSAM factors, and committed six incremental
updates. HBA residual was `274.96`; full-sequence ATE/RMSE was `142.766991 m`.
The complete run took about 77 minutes.

The global-aligned RMSE by route third was `103.305895 m`, `195.715217 m`, and
`110.281848 m`. Aligning each segment independently reduced the first and last
segments to `9.060111 m` and `29.386012 m`, but the middle remained at
`189.324436 m`. Adjacent output displacements remain below 10 m, so the error
accumulates without one isolated catastrophic jump.

Two middle-route controls further localize the issue. A continuous 600-scan
window without global mapping reached `23.874911 m` RMSE; enabling the full
backend on the same window gave `23.718836 m`, with almost identical estimated
path length. The first, middle, and last 20-second chunks of the continuous
window had separate alignment RMSE values of `1.2975 m`, `7.4952 m`, and
`11.4688 m`. Restarting the same middle interval produced `1.050848 m` over 20
seconds. The observed drift grows with continuous frontend state/map evolution;
short-window backend optimization does not explain it.

## Remaining work

- Isolate the middle-route local drift and preceding initialization/reset
  intervals.
- Instrument scan-to-map residuals, match counts, and state increments through
  the continuous 60-second window. Accepted updates currently still permit the
  observed drift.
- Find which inertial state or local-map update causes error to grow across
  successive 20-second intervals.
- Compare the full trajectory before and after HBA/PGO.
- Compare frontend update and local mapping behavior against Voxel-SLAM on the
  same raw scans.
- Reduce ATE/RMSE and verify each route segment again.
- Reduce the approximately 77-minute replay runtime after preserving accuracy.
