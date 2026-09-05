# Hestia — Home Environment Semantic Tracking and Intelligent Awareness

Hestia provides open-vocabulary home-scene perception for a ground mobile
robot. It detects objects in RGB, lifts them into metric 3D axis-aligned boxes
using aligned depth, assigns light multi-object track IDs, and publishes
existing automsgs detection messages.

Hestia never publishes `/cmd_vel`, does not replace Shadow selected-person
following, and does not run a depth network (it may consume Fathom refined
depth).

## Modes

| Mode | Behavior |
| --- | --- |
| `open` (default) | Every frame runs the open-vocabulary detector, then lift + track |
| `dual` | Fast closed-set home detector every frame; open path runs inline (`open_async_queue=0`) or on `OpenAsyncWorker` (`open_async_queue>=1`), then merge by IoU / score |

## Topics

| Topic (defaults) | Type | Direction |
| --- | --- | --- |
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | in |
| `/camera/depth/image_raw` or Fathom refined depth | `sensor_msgs/Image` | in |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | in |
| `/perception/hestia/detections_2d` | `vision_msgs/Detection2DArray` | out |
| `/perception/hestia/detections_3d` | `vision_msgs/Detection3DArray` | out |

`ObjectHypothesis.class_id` carries the text label. `Detection*.id` carries the
track id.

## Fixed detector contract

Export artifacts must match:

- input `images`: float32 `[1, 3, H, W]`, RGB in `[0, 1]`, letterboxed
- output `output0`: float32 `[1, max_detections, 6]` as
  `[x1, y1, x2, y2, score, class_index]`
- `class_index` indexes `open_prompts` (open) or `home_labels` (home)

Changing the prompt vocabulary requires re-exporting the model.
`OpenDetector::SetPrompts` only updates the string table used to fill
`class_id`.

## Deployment profiles

| Profile | Open model | Home model (dual) | Target rate |
| --- | --- | --- | --- |
| Orin | YOLO-World-S/M @ 640 | YOLO-n/s home set | 10–15 Hz |
| RK3588 | YOLO-World-S @ ≤480, INT8 | YOLO-n INT8 | ≥5 Hz; prefer dual + async open |

Rates are targets for on-device measurement, not CI asserts.

Point `open_model_path` / `home_model_path` at local ONNX or TensorRT files.
**Model weights are not stored in this repository.**

## Fathom depth

Set `use_fathom_depth: true` in `conf/hestia.pb.txt` as documentation, and wire
the DAG depth reader to Fathom's refined-depth topic (see comment in
`dag/hestia.dag`).

## Build and launch

Requires `BUILD_ONNXRUNTIME` and a found OnnxRuntime, same as Fathom.

```bash
# After a normal autonomy configure/build:
# libhestia_component.so is installed beside other perception components.
```

Standalone launch: `autonomy/perception/hestia/launch/hestia.launch`.

## Python fine-tuning and export

From `autonomy/perception/hestia/python`:

```bash
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
pytest -q
```

### Home closed-set (dual fast path)

1. Build a JSONL manifest (paths relative to the manifest directory):

```json
{"image":"rgb/0001.jpg","boxes":[[0,0.50,0.50,0.20,0.30]]}
```

2. Materialize Ultralytics `data.yaml` and fine-tune (requires installed `ultralytics`):

```bash
python -c 'from pathlib import Path; from hestia.dataset import HomeManifestDataset, write_yolo_dataset; from hestia.prompts import load_label_list; labels=load_label_list(Path("labels.json")); ds=HomeManifestDataset(Path("train.jsonl"), labels); print(write_yolo_dataset(ds, Path("yolo_data")))'
python -m hestia.train_home --checkpoint yolo11n.pt --data yolo_data/data.yaml --output runs/home --labels labels.json
```

3. Export a fixed ONNX profile for C++:

```bash
python -m hestia.export --kind home --checkpoint runs/home/home/weights/best.pt \
  --output /models/hestia_home.onnx --labels labels.json --image-size 640 --max-detections 100
```

### Open vocabulary

v1 does **not** train open-vocab models in-tree. Export an already-trained
YOLO-World / YOLOE checkpoint and write the prompt sidecar that must match
`HestiaOptions.open_prompts` order:

```bash
python -m hestia.export --kind open --checkpoint yoloworld.pt \
  --output /models/hestia_open.onnx --labels open_prompts.json --image-size 640 --max-detections 100
```

Tensor contract (must match `detector.cpp`):

| Tensor | Shape |
| --- | --- |
| `images` | float32 `[1, 3, H, W]`, RGB in `[0, 1]` |
| `output0` | float32 `[1, max_detections, 6]` = `x1,y1,x2,y2,score,class_index` |

## Licensing


Hestia integration code is Apache-2.0. Detector training/export typically
depends on YOLO-World, YOLOE, and/or Ultralytics. Deployers are responsible
for choosing license terms compatible with their application. This tree does
not vendor Ultralytics or detector weights.

## Layout

```text
hestia/
├── proto/hestia.proto
├── conf/hestia.pb.txt
├── dag/hestia.dag
├── launch/hestia.launch
├── options.*          # ValidateHestiaOptions
├── detector.*         # OpenDetector / HomeDetector
├── lifter.*           # DepthLifter
├── tracker.*          # ObjectTracker
├── merger.*           # DetectionMerger
├── open_worker.*      # OpenAsyncWorker
└── hestia_component.* # autolink component DSO
```
