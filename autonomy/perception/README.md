# Autonomy Perception

> Open robots for everyone.

## Layout

```text
perception/
  conf/                 # component protobuf configs (AUTOLINK_CONF_PATH)
    base.pb.txt
    follow.pb.txt
  base/dag/base.dag
  follow/dag/follow.dag
  launch/perception.launch
```

## Base — YOLO26 visual backbone + MoGe depth

Shared RGB inference. YOLO26 covers detect / segment / classify / pose / OBB /
track. Monocular depth uses MoGe (`DEPTH_BACKEND_MOGE`, ONNX or TensorRT).

## Follow — person following

Consumes `/perception/base/tracks` and `/perception/base/depth`, builds a
rolling 2.5D `grid_map`, locks a person by track id, and publishes target +
path + grid for `task/TrackingClient` and MPPI `GridObstaclesCritic`.

```text
base(MoGe depth [, track]) → follow(grid + localize + plan) → TrackingClient / MPPI
```

```bash
cd /workspace/autonomy/src/autonomy
export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
export AUTOLINK_DAG_PATH=$PWD/autonomy
export AUTOLINK_CONF_PATH=$PWD/autonomy/perception
export AUTOLINK_LIB_PATH=/workspace/autonomy/build/autonomy/lib
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/perception/launch
autolink launch start perception.launch
```
