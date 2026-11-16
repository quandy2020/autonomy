# Autonomy Perception

工程文档见 [docs/README.md](docs/README.md)。

> Open robots for everyone.

## Layout

```text
perception/
  conf/                 # component protobuf configs (AUTOLINK_CONF_PATH)
    base.pb.txt
    follow.pb.txt
  dag/perception.dag
  launch/perception.launch
```

## Base — YOLO26 visual backbone + MoGe depth

Shared RGB inference. Detect decodes a YOLO26 end-to-end export with float32
`output0 [1,N,6]` (xyxy, score, class index). Track runs that detector and
associates person boxes across adjacent frames. Ambiguous matches and gaps
produce new IDs. Monocular depth uses MoGe (`DEPTH_BACKEND_MOGE`).

Before starting the bundled launch, set the YOLO26 and MoGe engine paths in
`conf/perception.pb.txt`. The sample TensorRT profile uses 640×640 for YOLO26 and
518×518 for MoGe. Its one-entry class list exposes COCO person index 0.
For another export, configure the corresponding class order. Both messages
carry the source RGB timestamp. CameraInfo, odometry and the camera-to-map
transform must be available for Follow.

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

## Traversability semantics

`TASK_SEMANTIC` publishes `sensor_msgs.Image` on
`/perception/base/semantic`. Each mono8 pixel is a class index in the
export's label order. Class 255 denotes unknown. The image has the original
RGB dimensions, timestamp and optical frame. Semantic inference runs
independently of the tracking and depth outputs.

The SegFormer-B0 deployment export consumes RGB NCHW with ImageNet
normalization and stretched input geometry. Export a single float32
`logits` tensor of shape `[1,C,H,W]`, with 2–255 classes. The decoder
uses per-pixel argmax followed by nearest-neighbor resizing. Compare this
postprocessing with the training reference on the same images before
enabling the head. Set the engine path and dimensions in `conf/perception.pb.txt`.

### Local grid integration

1. Label data from the robot camera for floor, obstacle, threshold, stairs
   and drop-off. Fix the class index mapping alongside the export. Include
   shadows, reflections and low-light sequences in the holdout data.
2. In a replay consumer, pair semantic labels with depth by RGB timestamp and
   optical frame. Require matching pixel geometry and use valid depth and
   CameraInfo to project labels. Keep unobserved cells unknown, including
   drop-off pixels without valid depth. Use geometry to resolve traversability
   where the class depends on chassis clearance, such as thresholds.
3. Compare a semantic layer against the current geometry-only grid using
   hazardous-area misses, false blocked area and end-to-end P95 latency.
   Once validated, expose the layer to planning as extra evidence. Preserve
   measured obstacles on disagreement and retain the existing independent
   near-field safety inputs.

Follow now accepts an optional `semantic_topic` and explicit
`semantic_obstacle_labels` in `conf/follow.pb.txt`. A label image contributes
only when the timestamp, optical frame and dimensions match depth exactly.
CameraInfo must also match the optical frame and pixel dimensions.
Valid-depth projections of configured hazard classes add obstacles and
appear in the `semantic_obstacle` grid layer. Person clearing preserves
that layer, including inflated cells. Missing or mismatched semantics
leave the grid on its depth-only path. Pixels without valid depth cannot
produce semantic obstacle coordinates and supply no clearing evidence.

The example leaves fusion and the semantic head disabled until the export,
label mapping and replay checks are ready. The optional reader retains the
latest label image, so measure the exact-match rate on the target transport
before tuning the model or considering a bounded timestamp queue.

## Optional model configuration

The Python entry points accept arguments directly or accept a user-supplied
JSON file through `--conf`. Model weights are specified explicitly.

- `depth_completion.py` accepts an ONNX model with float32 NCHW inputs
  `rgb` in [0,1] and `sparse_depth` in metres, and an output `depth`
  at the source resolution. It preserves valid measured depth.
- `person_reid.py` accepts an ONNX export with float32 input
  `[1,3,256,128]` and a float32 embedding output `[1,D]`.
  To use its output in Follow, set the same tracks topic in both the
  Follow options and DAG.

Base's optional classification, OBB, keypoint and instance segmentation
heads are configured under `tasks` in `conf/perception.pb.txt`. The configured
weights must match the decoder's tensor and class order.

Base declares inference sessions under `models` in `conf/perception.pb.txt`.
Enabled tasks reference a session by `model_id`. An explicitly shared model
ID reuses one loaded backend. Detect and Track also reuse the same frame's
detections when their session and input dimensions match. Existing task
configs with inline `model_path` remain supported.

## Source layout

`engine/` owns the AutoLink adapter and task execution. `inference/`
provides session loading and image inference. `tasks/` interprets model
outputs and `behaviors/tracking/` owns frame association. Protobuf sources live in
`proto/`. The engine reads the `engine` section in `conf/perception.pb.txt`;
the exploration process continues to read its existing top-level fields.

The component defaults to `CreateTaskPipeline`. Other C++ assembly code
can provide a `Component::PipelineFactory` that returns a `Pipeline`
implementation using the same output contract. Task creation and
frame execution live in the default pipeline, while the AutoLink
component validates configuration, stamps and publishes observations.

The CMake component targets are `perception_core` and
`perception_behaviors`. Their shared library names remain compatible
with the AutoLink manifests. Start `perception.launch` for model outputs
and `behaviors.launch` for Follow. They exchange AutoMsgs topics across
processes.

YOLO-style tasks and semantic segmentation accept an optional task
`output_name` to select a graph output with the same tensor contract.
Instance segmentation also accepts `mask_output_name`. MoGe retains its
multi-tensor adapter contract. Detect and Track share per-frame detections
only when their model, input dimensions and configured output name match.

### Native object graph service

`autonomy.perception_object <options.pb.txt>` loads ONNX or TensorRT
graphs once and exposes a C++ AutoLink service. Configure an
`ObjectInferenceOptions` message from `proto/perception.proto`. Set
`service` to an absolute topic such as `/perception/object/infer`.
List graph stages in execution order, with each stage's `id`,
`model_path`, `backend`, input bindings and desired output tensor names.
An input binding uses the graph input name as its key. Its value is either
a request tensor name or an earlier output qualified as `stage.tensor`.
The service returns the final stage's selected tensors and copies the
request header. Tensor payloads are contiguous in the target host's byte
order, and the loaded graph metadata defines their dimensions.

For Grounding SAM 2, the export and request producer must agree on text
tokenization, image preprocessing and the box-to-prompt representation.
For FoundationPose, the export or C++ caller must provide the registration
steps around the neural models, including rendering, hypothesis generation
and refinement. The service supports chaining tensor-compatible stages.
It does not subscribe to cameras, tokenize prompts, generate mesh renders
or convert pose outputs to AutoMsgs. Use the C++ service with a producer that supplies prepared tensors.
Direct camera and prompt input requires the remaining native adapters.

Model families are selected with `family` on each model or object graph
stage. The RGB engine accepts YOLO for its existing YOLO tasks, SegFormer
for semantic segmentation and MoGe for depth. SAM 2 and FlashOcc graph
exports use the native tensor service, with explicit stage bindings.
The adapters load compatible exported graphs. SAM 2 prompt preparation
and FlashOcc camera synchronization, calibration and occupancy publishing
are caller responsibilities at this interface.

BOXER can be selected as `MODEL_FAMILY_BOXER` in the native graph service.
Supply a compatible export and bind its named tensors. The upstream lifting
pipeline uses 2D object proposals, a calibrated image and gravity, with
depth as optional evidence. Sequence lifting also needs camera poses.
The graph service returns tensors. Publishing 3D detections and fusing
boxes across frames require a task adapter with the export's pose convention.

For room contours, the native graph service accepts a final
`MODEL_FAMILY_RRCE` stage whose export wrapper exposes float32 packed
`vertices [V,2]` and int32 `vertex_counts [R]`. The caller supplies the
preprocessed grid tensor and the inverse preprocessing and map transform
in `room_map`. The response carries map-frame contours and oriented
rectangles with the source map header. This is the adapter's export
contract. The upstream checkpoint still needs an equivalent exported
graph and preprocessing. The service does not yet subscribe to SLAM
maps or maintain persistent room IDs.

For a single RRCE stage, a client may instead submit an AutoMsgs
`OccupancyGrid` as the request's `occupancy_grid` field. The native-grid
export must accept one float32 tensor `[1,1,H,W]` at the source resolution
and row order. The adapter maps occupancy 0–100 to 0–1, keeps unknown
cells at -1, and interprets integer output coordinates as cell centres.
It uses the map origin and orientation for the response polygons. Use
the prepared-tensor request when the export requires a different
normalization, crop, rotation or resize.

### Geometry and semantic annotations

RRCE room decoding lives in `tasks/room_segmentation/`. The
`tasks/scene_graph/` layer maps contours and rectangles into world
coordinates and can assign a map-frame object centre to a room polygon.
Boundary and overlapping cases are unassigned. Keep room labels and object
membership on semantic messages alongside the geometric graph. FAR graph
updates must depend on geometric observations. Exploration may use semantics
to rank reachable goals or fall back to geometric frontiers.

A final Boxer stage can enable `boxer_decode` in the object service
configuration. Set `boxes_output` to a bound float32 `[N,12]` output
with rows `cx,cy,cz,sx,sy,sz,qx,qy,qz,qw,score,class_index`, and provide
the export's `class_names`. Coordinates and dimensions are in metres.
Set `frame_id` to the export's output frame and use the same frame in
the request header. The response contains a stamped AutoMsgs
`Detection3DArray`. This contract expects an export wrapper that already
applies any camera-to-map transform. Raw tensor mode remains available
when the decoder is omitted.

A Boxer request may include a `room_snapshot` with room IDs and map-frame
contours. Its frame and revision must match the request. The response
returns memberships keyed by zero-based index in `detections_3d`.
Outside, boundary and ambiguous detections have no membership.
The caller owns persistent room IDs and snapshot revisions. The service
does not update the geometric routing graph.
