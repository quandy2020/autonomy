# Neupan Controller

Loads a trained NeuPAN obstacle-point network and produces differential-drive commands `(v, ω)`.

Upstream algorithm: [NeuPAN](https://github.com/hanruihua/NeuPAN) (T-RO 2025).

## Export model

```bash
python3 tools/neupan/export_distance_encoder_onnx.py \
  --checkpoint /path/to/model_5000.pth \
  --edge-count 4 \
  --output /path/to/distance_encoder.onnx
```

`edge-count` must match the robot footprint edge count used in training (rectangle → 4).

## Configure

See [`conf/neupan_controller_example.pb.txt`](../../conf/neupan_controller_example.pb.txt):

- `controller_plugins: "neupan_controller:NeupanController"`
- `neupan_controller_options.neural_encoder_model_path`

## Architecture

```
SetPlan  → PathCache (SoA x/y/yaw, warm-started closest index)
SetScan  → shared LaserScan snapshot
Tick     → Eigen[2,N] obstacles (reused buffer)
         → DistanceEncoder (nearest features only)
         → MotionPlanner → Twist
```

| Module | Role |
|---|---|
| `types` | Shared POD (`LaserExtrinsic`, `DistanceFeatures`) |
| `path_cache` | Flat path cache + lookahead |
| `obstacle_extractor` | LaserScan → Eigen points |
| `robot_footprint` | `Polygon` → Eigen halfspaces `A p ≤ b` |
| `distance_encoder` | ONNX infer; only nearest dual kept |
| `motion_planner` | Distance-gated path tracking |
| `controller` | Plugin wiring / shared_ptr snapshots |

Hot-path caches: laser extrinsic POD, path SoA, reusable Eigen / float scratch buffers, reused FloatTensorMap.
