# Hestia Open-Vocabulary Home Perception Design

## Purpose

Hestia provides open-vocabulary home-scene perception for a ground mobile
robot. It detects everyday objects in RGB, lifts them into metric 3D bounding
boxes using aligned depth, maintains light multi-object track IDs, and
publishes results for navigation, interaction, and downstream semantic mapping.

Hestia is a perception component. It never publishes `/cmd_vel`, owns no
controller, and does not replace Shadow selected-person following or Fathom
depth refinement. It may consume Fathom refined depth when configured.

**HESTIA** stands for **Home Environment Semantic Tracking and Intelligent
Awareness**.

## Scope

The implementation includes:

- an autolink `HestiaComponent` following the Fathom component lifecycle
  (`Init` / `Proc` / `Clear`);
- **mode `open` (default path A):** open-vocabulary 2D detection from an
  externally supplied fixed-profile YOLO-World or YOLOE artifact, then depth
  lift to 3D axis-aligned boxes;
- **mode `dual` (path B):** a fast closed-set home ontology detector every
  frame, plus open-vocabulary detection on an async or query-triggered path,
  merged before publish;
- robust RGB-D lifting inspired by Shadow `TargetLocalizer`, extended to 3D
  box size;
- ByteTrack-style multi-class tracking with string `track_id` values;
- publication of existing `vision_msgs::Detection2DArray` and
  `Detection3DArray` only;
- Orin and RK3588 deployment profiles documented in README and sample
  `hestia.pb.txt`;
- unit tests with injectable runners and no repository weights;
- deployment documentation and license attribution for external detectors.

The implementation excludes:

- end-to-end open-vocabulary 3D detectors as the primary path;
- per-frame SAM / instance masks (optional grasp-time mask is out of v1);
- semantic map fusion, room topology, and cross-module ID alignment with
  Shadow;
- model weights, datasets, and vendored Ultralytics sources;
- cloud LLM backends and offline batch mapping pipelines as the realtime path.

## Upstream and Licensing Boundaries

Detector training and export remain external. Candidates:

- YOLO-World / YOLOE (or Ultralytics open-vocab export) for open prompts;
- a closed-set YOLO family model fine-tuned or filtered to a home ontology for
  `dual` fast path.

C++ consumes only fixed-profile ONNX / TensorRT (and RK NPU adapters behind the
same `common::network` runner seam). Hestia does not vendor Ultralytics or
detector weights. The README must state that deployers are responsible for
license terms compatible with their application. Hestia-authored integration
code uses the repository Apache-2.0 header.

## Architectural Decision

Constraints agreed during design:

- edge realtime (~5–15 Hz) for navigation / interaction;
- RGB-D primary sensing;
- must be deployable on Jetson Orin **or** RK3588-class NPU.

Therefore Hestia uses **open-vocabulary 2D detection + metric depth lift**,
not an end-to-end open 3D detector. One autolink component owns focused
internal modules so RGB-D, detection, lifting, tracking, and merge stay in one
processing transaction without a monolith class.

Alternatives rejected:

- end-to-end open 3D detection as the default (too heavy for RK3588 realtime);
- Grounding DINO + SAM every frame (Orin-only quality mode, not dual-platform
  default);
- always running two large open models synchronously in `Proc` (fails the RK
  budget).

Product policy: **A is default (`mode=open`); B is a config switch
(`mode=dual`)**.

## Directory Layout

```text
autonomy/perception/hestia/
├── CMakeLists.txt
├── README.md
├── conf/
│   └── hestia.pb.txt
├── dag/
│   └── hestia.dag
├── launch/
│   └── hestia.launch
├── proto/
│   └── hestia.proto
├── hestia_component.hpp
├── hestia_component.cpp
├── options.hpp
├── options.cpp
├── detector.hpp
├── detector.cpp
├── lifter.hpp
├── lifter.cpp
├── tracker.hpp
├── tracker.cpp
├── merger.hpp
├── merger.cpp
├── hestia_component_test.cpp
├── options_test.cpp
├── detector_test.cpp
├── lifter_test.cpp
├── tracker_test.cpp
└── merger_test.cpp
```

The C++ tree stays flat, matching Fathom and Shadow. First implementation does
not require an in-tree Python package; export recipes live in README. A later
`python/` tree may be added if home-ontology fine-tuning becomes first-party.

## Component Boundary

`HestiaComponent` derives from the three-input autolink specialization:

1. RGB `automsgs::msgs::sensor_msgs::Image`;
2. aligned depth `automsgs::msgs::sensor_msgs::Image`;
3. `automsgs::msgs::sensor_msgs::CameraInfo`.

Depth may be raw depth or Fathom refined `32FC1`. When
`use_fathom_depth=true`, the sample DAG subscribes to Fathom's refined-depth
topic.

Optional async input (dual / query):

- `automsgs::msgs::std_msgs::String` open-prompt override or query text
  (empty keeps configured `open_prompts`).

Publications use only existing automsgs types:

| Default topic | Type | Meaning |
| --- | --- | --- |
| `/perception/hestia/detections_2d` | `vision_msgs::Detection2DArray` | Current 2D boxes; `results[].hypothesis.class_id` is the text label; `id` is track id |
| `/perception/hestia/detections_3d` | `vision_msgs::Detection3DArray` | Lifted 3D AABB for detections with enough depth; same label / track id convention |

No custom public transport message is introduced. `HestiaOptions` is
configuration only.

Callable seams mirror Fathom: detect / lift / publish functions may be injected
in tests so lifecycle tests do not need model artifacts.

## Module Responsibilities

### `detector`

Shared `DetectorRunner` interface (same idea as Shadow):

```text
Run(TensorMap inputs, TensorMap* outputs, string* error) -> bool
```

Two facades may share the runner type:

- `OpenDetector` — open-vocabulary fixed profile; prompts from options or query;
- `HomeDetector` — closed-set home ontology; class indices map to
  `home_labels[]`.

Both preprocess RGB, run inference, filter by `confidence_threshold` /
`max_detections`, restore boxes to original image coordinates, and fill
`Detection2DArray`.

`ObjectHypothesis.class_id` is already a **string** in automsgs; open labels
and home labels are written there directly. `Detection2D.id` /
`Detection3D.id` store the tracker id (empty until tracking assigns one).

Exact ONNX tensor names and layouts are fixed per export profile and validated
against engine metadata in `Init()`, same as Fathom/Shadow. The README documents
the Orin and RK profiles (input size, output layout, INT8 notes).

### `lifter`

`DepthLifter` converts each 2D detection into an optional `Detection3D`:

1. shrink the box by `inner_box_scale`;
2. collect finite depths in `[min_depth_m, max_depth_m]`;
3. require at least `min_depth_samples` inliers after outlier rejection;
4. median depth along the box-center ray → box center;
5. horizontal/vertical pixel extent at that depth plus depth percentile span
   (`p10`–`p90`) → `BoundingBox3D.size`;
6. orientation is identity (axis-aligned AABB in the output frame).

Output `frame_id` defaults to `camera_frame`. If `base_frame` is set and TF is
available at the image stamp, centers are transformed into base; on TF failure
the lifter keeps camera frame rather than failing the whole frame.

Detections that fail the depth gate are omitted from the 3D array but remain in
the 2D array.

### `tracker`

`ObjectTracker` associates multi-class detections across frames (IoU and center
distance). It is separate from Shadow `PersonTracker` because the label space is
open string classes, but the association pattern may follow the same
ByteTrack-style two-pass idea. Track ids are monotonically increasing strings
within one component lifetime. Tracks expire after `lost_timeout_sec`.

### `merger`

Used only in `mode=dual`. Merges fast-path home detections with the latest
completed open-path results (or an empty open set if the async path is not
ready). v1 suppression is 2D IoU with preference for the higher score when boxes
overlap above the configured threshold. Label synonym tables are out of v1.
Open-path latency must not block publishing the fast path.

## Operating Modes

| Mode | Fast path | Open path |
| --- | --- | --- |
| `open` | — | Every `Proc` frame |
| `dual` | `HomeDetector` every frame | Async worker or query-triggered; merge on publish |

`mode=open` is the default product path. `mode=dual` is the switch for low-NPU
platforms that need stable home classes at rate while keeping open queries.

## Configuration

Single protobuf `HestiaOptions` validated by `ValidateHestiaOptions`, matching
Fathom/Shadow. Planned fields:

- `mode` (`open` | `dual`);
- `open_model_path`, `home_model_path`, `backend`;
- `open_width` / `open_height`, `home_width` / `home_height`;
- `confidence_threshold`, `max_detections`;
- repeated `open_prompts`, repeated `home_labels`;
- depth gate: `depth_scale`, `min_depth_m`, `max_depth_m`,
  `min_depth_samples`, `inner_box_scale`, `depth_outlier_m`;
- frames: `camera_frame`, `base_frame`;
- tracking: association IoU, `lost_timeout_sec`;
- topics: `detections_2d_topic`, `detections_3d_topic`, optional query topic;
- sync: `max_input_skew_sec`, `max_data_age_sec`;
- `use_fathom_depth` (documentation / DAG selection; component still receives
  whatever depth topic the DAG wires).

## Error Handling and Degradation

`Proc` returns true only when the frame is accepted and required publications
succeed (zero detections is success).

| Condition | Behavior |
| --- | --- |
| Null message, bad encoding, size mismatch, skew/age exceeded | Skip frame, log, return false |
| Model init failure | `Init` fails; component does not start |
| Dual open path not ready | Publish fast path only; still success |
| Insufficient depth samples for one box | Omit from 3D; keep in 2D |
| Base TF unavailable | Publish 3D in camera frame |
| Runner inference failure | Fail the frame; do not republish stale detections |

## Boundaries with Sibling Modules

| Module | Owns | Hestia does not |
| --- | --- | --- |
| Shadow | Person select, follow path, local grid | Selection / follow policy |
| Fathom | Depth network + refined depth / cloud | Depth network inference |
| Hestia | Open/closed object 2D→3D + tracks | `/cmd_vel`, semantic map |

Both Shadow and Hestia may emit `person`. Downstream consumers distinguish by
topic. Cross-module track-id alignment is out of v1 scope.

## Deployment Profiles

| Profile | Open model | Home model (dual) | Target rate |
| --- | --- | --- | --- |
| Orin | YOLO-World-S/M @ 640 | YOLO-n/s home set | 10–15 Hz |
| RK3588 | YOLO-World-S @ ≤480, INT8 | YOLO-n INT8 | ≥5 Hz; open async in dual |

Measured rates belong in README, not CI hard asserts.

## Testing

- `options_test` — invalid paths, topics, modes, dimensions;
- `detector_test` — fake `DetectorRunner` tensor outputs;
- `lifter_test` — synthetic planar / box depth with bounded size error;
- `tracker_test` / `merger_test` — association and dual suppression;
- `hestia_component_test` — Init/Proc/Clear with publisher seams, no weights.

## Acceptance Criteria

1. `mode=open` on a fixture RGB-D pair yields `Detection3DArray` with text
   `class_id` and track `id` when depth is sufficient.
2. `mode=dual` keeps the fast path at the configured rate while open results
   may lag without blocking publish.
3. Orin and RK profiles are documented with expected rates.
4. Glass / far / sparse-depth boxes do not invent 3D geometry (depth gate
   drops them from the 3D topic).
5. Unit tests compile and pass without repository model weights.

## Non-Goals for v1

- Oriented 3D boxes (OBB);
- Per-frame masks;
- Instance-level semantic map database;
- Replacing Shadow for following;
- Claiming “detect literally everything every frame” without confidence and
  prompt-set limits — open vocabulary is prompt- and score-gated.
