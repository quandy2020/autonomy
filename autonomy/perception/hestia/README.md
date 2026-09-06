# Hestia — Home Environment Semantic Tracking and Intelligent Awareness

Hestia provides open-vocabulary home-scene perception for a ground mobile
robot. It detects objects in RGB, lifts them into metric 3D axis-aligned boxes
using aligned depth, assigns light multi-object track IDs, and publishes
existing automsgs detection messages.

## Runtime (C++)

Component entry: `component` (`libhestia_component.so`; `dag/hestia.dag`,
`launch/hestia.launch`, `conf/hestia.pb.txt`).

| File | Type |
| --- | --- |
| `runner` | `Runner` |
| `detector` | `Detector<PromptKind>` (`OpenDetector` / `ClosedDetector`) |
| `async` | `Async<T>` |
| `lift` | `Lifter` |
| `merger` | `Merger` |
| `tracker` | `Tracker` (class-gated IoU) |
| `options` | `ValidateHestiaOptions` |
| `component` | `Component` |

### Modes

- `MODE_OPEN`: open-vocabulary detector every frame.
- `MODE_DUAL`: closed-set every frame; open runs inline or via `open_async`.
  With `open_async=true`, dual uses **stale-OK** open results: each frame
  polls the latest completed open detections, then enqueues the current RGB.
  Empty open results still fuse (closed-set only).

### Frames and depth

- RGB and depth must share encoding constraints and **identical dimensions**.
- Optional Fathom refined depth: point the DAG depth reader at
  `/perception/fathom/refined_depth`.
- When `base_frame` is set, Component looks up TF
  `base_frame <- camera_frame` and lifts 3D boxes into base; on TF failure
  boxes stay in `camera_frame`.

### ONNX contract

`images` `[1,3,H,W]` → `output0` `[1,max_det,6]`
(`x1,y1,x2,y2,score,class_index`).

Export freezes the prompt/label order used in `open_prompts` / `home_labels`.
Set `nms_iou_threshold > 0` only if the artifact did **not** apply NMS.

### Smoke

```bash
HESTIA_SMOKE_MODEL=/path/to/hestia_open.onnx ctest -R hestia.smoke
```

## Licensing

Apache-2.0 for Hestia code. Weights are not stored in this repository.
