# Neupan Controller

NeuPAN local controller: **DUNE** (ONNX ObsPointNet) + **NRMP** (DAQP QP) + **PAN**
alternating loop. Differential and omni kinematics. No PyTorch / cvxpy / OSQP.

Algorithm: [NeuPAN](https://github.com/hanruihua/NeuPAN) (T-RO 2025). Layout
informed by the public `neupan_cpp` port; this tree is Apache-2.0 and solves
the QP with in-repo DAQP (`common/math/miqp`).

## Export DUNE (ONNX)

```bash
python3 autonomy/control/controller/neupan_controller/tools/export_distance_encoder_onnx.py \
  --checkpoint autonomy/control/conf/models/neupan_model_5000.pth \
  --output autonomy/control/conf/models/neupan_distance_encoder.onnx
```

`edge-count` must match the footprint used in training (rectangle → 4).

Without `neural_encoder_model_path`, the planner still tracks the global path
(empty obstacle duals). Collision avoidance requires the encoder.

## Configure

See `conf/neupan_controller_example.pb.txt`:

- `controller_plugins: "neupan_controller:NeupanController"`
- `neupan_controller_options.neural_encoder_model_path`
- `robot_kinematics`: `differential` / `omni`

## Tick

```
SetPlan  → NeuPanPlanner::SetInitialPath
SetScan  → LaserScan snapshot
Tick     → base-frame points → world
         → NeuPanPlanner (InitialPath + PAN[DistanceEncoder, NRMP])
         → Twist (v,ω) or omni (vx,vy)
```
