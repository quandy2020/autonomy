# Autonomy Perception

> Open robots for everyone.

## Base — YOLO26 visual backbone + MoGe depth

Shared RGB inference. YOLO26 covers detect / segment / classify / pose / OBB /
track. Monocular depth uses MoGe (`DEPTH_BACKEND_MOGE`, ONNX or TensorRT).

## Follow — person following

Consumes `/perception/base/tracks` and `/perception/base/depth`, builds a
rolling 2.5D `grid_map`, locks a person by track id, and publishes target +
local path for `task/TrackingClient`.

```text
base(detect+track, MoGe depth) → follow(grid + localize + plan) → TrackingClient
```
