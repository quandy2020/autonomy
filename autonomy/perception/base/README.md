# Base — YOLO26 visual backbone

Base is the shared RGB inference layer for mobile-robot perception. YOLO26 is
the foundation model. Each vision task lives under `tasks/<name>/` with its
own decode path. Depth is an optional RGB-to-metric head on the same loader.

Downstream modules should subscribe to these topics instead of loading their
own detectors.

## Layout

```text
base/
  engine/           # shared YOLO26 / network loader
  tasks/
    detect/         # 目标检测
    segment/        # 实例分割
    classify/       # 图像分类
    pose/           # 姿态估计
    obb/            # 旋转边界框
    track/          # 目标跟踪
    depth/          # 深度估计
  proto/base.proto
  base_component.*
```

## Tasks

| Task | Flag | Output | Topic |
| --- | --- | --- | --- |
| Detect | `tasks { kind: TASK_DETECT … }` | `Detection2DArray` | `/perception/base/detections` |
| Segment | `TASK_SEGMENT` | `Detection2DArray` (boxes) | `/perception/base/masks` |
| Classify | `TASK_CLASSIFY` | `Classification` | `/perception/base/classification` |
| Pose | `TASK_POSE` | `Detection2DArray` (boxes) | `/perception/base/poses` |
| OBB | `TASK_OBB` | `Detection2DArray` (`bbox.theta`) | `/perception/base/obb` |
| Track | `TASK_TRACK` | `Detection2DArray` (`id`) | `/perception/base/tracks` |
| Depth | `TASK_DEPTH` | `Image` `32FC1` (MoGe) | `/perception/base/depth` |

All wire types come from `automsgs` (`vision_msgs` / `sensor_msgs`).
`base.proto` defines shared `TaskOptions` entries in `repeated tasks`.
Per-pixel masks / keypoints are not in `vision_msgs` yet; segment and pose
currently carry boxes only.

Input for every graph: `images` `[1,3,H,W]` RGB in `[0,1]`, letterboxed /
stretched to `input_width` x `input_height` (divisible by 32 for YOLO; MoGe
commonly 518×518). Class index order is frozen in `class_names`.

Weights live under `config/perception/base/` and are not stored in git.
Enable a task only after its engine is exported.

## MoGe depth (ONNX / TensorRT)

Sample runtime: `config/perception/base/base_depth_moge.pb.txt`
(`tasks { kind: TASK_DEPTH … depth_backend: DEPTH_BACKEND_MOGE }`).

Point `tasks[].model_path` at a static MoGe-2 engine (see MoGe `docs/onnx.md`):

- input `image` `[1,3,H,W]` RGB `[0,1]`
- outputs `points`, `normal`, `mask`, `metric_scale`
- C++ recovers metric depth: `(points.z + shift) * metric_scale`
- invalid mask → NaN (`moge_mask_threshold`, default 0.5)
- optional `moge_fov_x_deg` when FOV is known

Export (avoids `squeeze`-induced ONNX `If` nodes that TensorRT rejects):

```bash
# Prefetch weights if HF is slow, then:
XFORMERS_DISABLED=1 python config/perception/base/export_moge_trt.py \
  --pretrained config/perception/base/moge_ckpt/model.pt
# → moge-2-vits-normal_518x518.onnx + moge-2-vits-normal_518x518_fp16.engine
```

## Export contract

- Detect: end-to-end `output0` `[1,max_det,6]` as
  `x1,y1,x2,y2,score,class_index`.
- Segment: boxes via `Detection2DArray` (pixel masks unbound; no shared type).
- Classify: class scores aligned with `class_names`.
- Pose: boxes via `Detection2DArray` (keypoints unbound; no shared type).
- OBB: `cx,cy,width,height,angle,score,class_index`.
- Track: detect then assign stable ids into `Detection2D.id`.
- Depth (MoGe): see above.

YOLO head decode (except MoGe depth) is not bound yet.
