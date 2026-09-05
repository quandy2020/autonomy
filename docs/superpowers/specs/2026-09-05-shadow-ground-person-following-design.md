# Shadow Ground Person-Following Design

## Purpose

Shadow provides selected-person following for a ground mobile robot. It detects
people in RGB images, preserves a selected identity, estimates the target in
3D from aligned depth, maintains a rolling 2.5D local map, and publishes a safe
local path for the existing navigation and control stack.

Shadow is a planner-facing perception component. It never publishes
`/cmd_vel`, owns no low-level controller, and does not bypass the existing task,
navigation, or control boundaries.

## Scope

The implementation includes:

- an autolink `ShadowComponent` following the Fathom component lifecycle;
- YOLO26n person detection through an externally supplied ONNX or TensorRT
  artifact;
- ByteTrack-style multi-person association and selected-target locking;
- aligned RGB-D target localization;
- a rolling 2.5D `grid_map::GridMap`;
- a ground-robot policy adapted from YOPO's motion-primitive approach;
- deterministic safety filtering and automsgs publication;
- Python detector fine-tuning, policy training, and fixed-profile export;
- integration with the existing tracking task and combined perception launch;
- unit-test sources and deployment documentation.

The implementation excludes model weights, datasets, simulator execution,
low-level motion control, global route planning, and a reproduction of the
unreleased YOPOv2-Tracker implementation.

## Upstream and Licensing Boundaries

The design uses the following upstream references:

- YOPO `YOPO-Simple` at commit
  `58dda5b2d124aa3998593dd62798f1496c93b26d`, under the MIT License, as the
  reference for motion primitives, network structure, and guidance losses;
- the YOPOv2-Tracker paper as an architectural reference only because its
  tracking implementation is not currently published;
- Ultralytics YOLO26 as an external detector training and export dependency.

Shadow will not vendor the Ultralytics package or YOLO26 weights. Python code
imports an installed `ultralytics` package, while C++ consumes only an external
fixed-profile model artifact through the project common-network interface. The
README must state that Ultralytics publishes YOLO26 under AGPL-3.0 or a
commercial enterprise license and that deployers are responsible for choosing
terms compatible with their application.

Adapted YOPO source retains the upstream copyright and MIT notice.
Shadow-authored integration code uses the repository's Apache-2.0 header.

## Architectural Decision

Shadow uses one autolink component with focused internal modules. This keeps
RGB-D, tracking, mapping, and policy state in one processing transaction without
turning the component class into a monolith.

Alternatives rejected:

- one large implementation class, because it couples model parsing, tracking,
  mapping, planning, and transport;
- separate detector, mapper, and planner components, because their additional
  transport, configuration, and synchronization surfaces do not provide enough
  value for the first implementation.

All directory names are lowercase. C++ and Python filenames use `snake_case`.

## Directory Layout

```text
autonomy/perception/shadow/
├── CMakeLists.txt
├── README.md
├── conf/
│   └── shadow.pb.txt
├── dag/
│   └── shadow.dag
├── launch/
│   └── shadow.launch
├── proto/
│   └── shadow.proto
├── shadow_component.hpp
├── shadow_component.cpp
├── detector.hpp
├── detector.cpp
├── tracker.hpp
├── tracker.cpp
├── localizer.hpp
├── localizer.cpp
├── grid.hpp
├── grid.cpp
├── policy.hpp
├── policy.cpp
├── planner.hpp
├── planner.cpp
├── options.hpp
├── options.cpp
├── shadow_component_test.cpp
├── detector_test.cpp
├── tracker_test.cpp
├── localizer_test.cpp
├── grid_test.cpp
├── policy_test.cpp
├── planner_test.cpp
└── python/
    ├── shadow/
    │   ├── __init__.py
    │   ├── dataset.py
    │   ├── detector.py
    │   ├── policy.py
    │   ├── losses.py
    │   ├── train_detector.py
    │   ├── train_policy.py
    │   └── export.py
    └── test/
        ├── test_dataset.py
        ├── test_detector.py
        ├── test_policy.py
        ├── test_losses.py
        └── test_export.py
```

The C++ source tree stays flat, matching the accepted Fathom layout. The Python
package is grouped only by package and tests; it does not add generic
`configs/`, `scripts/`, `cpp/`, `inference/`, or `training/` directories.

## Component Boundary

`ShadowComponent` derives from the four-input autolink component specialization
with these reader types and this fixed DAG order:

1. RGB `automsgs::msgs::sensor_msgs::Image`;
2. aligned depth `automsgs::msgs::sensor_msgs::Image`;
3. `automsgs::msgs::sensor_msgs::CameraInfo`;
4. `automsgs::msgs::nav_msgs::Odometry`.

Depth may be raw metric depth or Fathom's refined `32FC1` output. The sample DAG
uses Fathom's refined-depth topic. An asynchronous
`automsgs::msgs::std_msgs::String` reader receives the selected target ID.

The component publishes only existing automsgs types:

| Default topic | Type | Meaning |
| --- | --- | --- |
| `/perception/shadow/detections` | `vision_msgs::Detection2DArray` | Current person detections with stable track IDs |
| `/perception/shadow/target` | `geometry_msgs::PoseStamped` | Selected person's filtered pose in `map` |
| `/perception/shadow/path` | `nav_msgs::Path` | Safe local following path in `map` |
| `/perception/shadow/grid` | `map_msgs::GridMap` | Rolling 2.5D safety map |
| `/perception/shadow/select` | `std_msgs::String` | Input target track ID; empty selects the nearest valid person |

No custom public data message is introduced. `ShadowOptions` is configuration,
not a transport message.

Each processing transaction validates non-null messages, supported encodings,
dimensions, header frame IDs, timestamp skew, and data freshness before changing
published state. Publications preserve the triggering RGB timestamp where the
output represents that frame.

## Module Responsibilities

### `detector`

`YoloDetector` owns one common-network engine and performs letterbox
preprocessing, inference, confidence filtering, coordinate restoration, and
conversion to `Detection2DArray`. Its runner seam allows tests to supply tensor
outputs without a model artifact.

The fixed deployment contract is:

- input `images`: float32 `[1, 3, detector_height, detector_width]`, RGB in
  `[0, 1]`;
- output `output0`: float32 `[1, max_detections, 6]`;
- output row: `[x1, y1, x2, y2, confidence, class_id]`;
- only configured `person_class_id` rows above `confidence_threshold` survive;
- YOLO26's end-to-end head supplies final detections, so C++ does not run NMS.

`max_detections`, width, and height are fixed export-profile values and are
validated against engine metadata during `Init()`.

### `tracker`

`PersonTracker` owns active tracks and selected-target state. It applies a
ByteTrack-style two-pass association: high-confidence detections first, then
low-confidence detections for unmatched tracks. Association uses predicted box
overlap and motion consistency. Track IDs are monotonically increasing strings
within one component lifetime.

The selected ID is never silently replaced by another visible person. An empty
selection request chooses the nearest confirmed person once and then locks its
ID. Short occlusions use the track predictor for no longer than
`prediction_timeout_sec`; after `lost_timeout_sec`, the target becomes lost and
the current path is invalidated. Automatic reacquisition may restore only the
same surviving track ID.

### `localizer`

`TargetLocalizer` samples finite positive depth values from the inner portion
of the selected bounding box, rejects depth discontinuity outliers, and uses
the median surviving depth. Camera intrinsics back-project the box center into
the camera frame. The transform buffer converts this point into `map` at the
image timestamp.

The localizer filters successive selected-target positions and derives the
planar target velocity used by the policy. A new target lock resets this filter
so motion from a previous person cannot leak into the new target state.

Localization fails closed when intrinsics are invalid, too few depth samples
remain, depth is outside the configured range, or the transform is unavailable.
It never emits a zero pose as a substitute for an unknown target.

### `grid`

`LocalGrid` owns a `grid_map::GridMap` centered on the robot. It uses the
repository's grid-map implementation and publishes through
`grid_map::GridMapConverter`.

It contains exactly these initial layers:

- `elevation`: filtered ground or surface height in metres;
- `variance`: elevation uncertainty;
- `obstacle`: `1.0` for an occupied cell and `0.0` for observed free space;
- `traversability`: normalized cost in `[0, 1]`, where `1.0` is impassable.

Depth samples are back-projected, transformed to `map`, range-filtered, and
downsampled before insertion. Height relative to the robot support plane,
neighbour slope, configured step height, and robot-radius inflation determine
obstacle and traversability values. Unknown cells remain non-finite and are
treated as impassable by `planner`.

The map rolls when the robot crosses the configured movement threshold. Cells
older than `cell_ttl_sec` return to unknown. A failed update does not partially
publish a new map.

### `policy`

`YopoPolicy` owns the second common-network engine and decodes motion-primitive
residuals and learned costs. It is ground-robot specific and does not preserve
YOPO's quadrotor attitude, thrust, differential-flatness, or aerial dynamics
outputs.

The fixed deployment contract is:

- `depth`: float32 `[1, 1, policy_height, policy_width]` in metres;
- `robot_state`: float32 `[1, 2]` containing linear and angular velocity;
- `target_state`: float32 `[1, 4]` containing target relative position and
  velocity `(x, y, vx, vy)` in `base_link`;
- `trajectories`: float32
  `[1, candidate_count, trajectory_steps, 3]` containing `(x, y, yaw)` in
  `base_link`;
- `scores`: float32 `[1, candidate_count]`, lower is better.

All dimensions are fixed by `ShadowOptions` and checked against the model
metadata at startup.

### `planner`

`FollowPlanner` transforms candidate trajectories into `map`, rejects samples
outside the rolling grid, and rejects any footprint intersecting unknown or
impassable cells. Remaining candidates receive deterministic costs for learned
score, clearance, traversability, curvature, progress, terminal follow
distance, and target visibility. The lowest finite cost wins.

The output path begins at the current robot pose and contains monotonically
ordered poses. Speed and curvature constraints are checked before publication.
If no candidate is safe, the planner returns a valid empty `Path` carrying the
current timestamp and map frame.

## End-to-End Data Flow

```text
RGB ───────────────> YoloDetector ─> PersonTracker ─┐
aligned depth ──────────────────────────────────────┼─> TargetLocalizer
CameraInfo + TF ────────────────────────────────────┘          │
                                                              v
                                                     selected target state

depth + CameraInfo + TF + Odometry ─> LocalGrid ───────────────┐
depth + robot state + selected target ─> YopoPolicy ─> candidates
                                                               │
                                                               v
                                                     FollowPlanner ─> Path
```

Detector and tracker outputs are published when detection succeeds. Target,
grid, and path are published only from a coherent transaction. An empty path
explicitly invalidates the previous path.

## Tracking Task Integration

The existing `TrackingClient` is extended without changing the public
`TrackerGoal`, `TrackerFeedback`, or `TrackerResult` schemas.

- `TRACKER_CMD_START` and `TRACKER_CMD_UPDATE_TARGET` publish the requested
  target ID to `/perception/shadow/select`.
- `TrackingClient` subscribes to `/perception/shadow/target` and
  `/perception/shadow/path`.
- person-following behavior uses Shadow's latest fresh path instead of the
  current placeholder pose generated for a string target ID.
- `TrackerFeedback.target_pose` and `distance_to_target` come from Shadow's
  latest fresh target pose.
- an empty or stale path, lost target, or cancel command cancels active motion;
  it never reuses the last valid path indefinitely.
- non-person target-pose mode keeps its existing navigation behavior.

The navigation and controller layers remain responsible for path execution and
velocity generation.

## Configuration

`proto/shadow.proto` defines one `ShadowOptions` message. It contains:

- detector and policy model paths, backend IDs, tensor profiles, thresholds,
  and person class ID;
- tracking association thresholds and prediction/loss timeouts;
- RGB-D sampling limits and valid depth range;
- map frame names, geometry, resolution, rolling threshold, cell TTL, step,
  slope, obstacle height, and robot inflation radius;
- primitive count, trajectory length, kinematic bounds, follow distance, and
  planner cost weights;
- input-selection and four output topic names;
- maximum input timestamp skew and maximum data age.

Validation rejects empty model paths, unsupported backends, empty frame/topic
names, non-positive dimensions or durations, thresholds outside their documented
ranges, non-positive map geometry, an inflation radius smaller than the robot
radius, and planner weights that are negative or collectively zero.

`conf/shadow.pb.txt` is the single sample configuration. There is no second
runtime options file.

## Python Training and Export

`python/shadow/detector.py` wraps the installed Ultralytics API for YOLO26n
fine-tuning restricted to the person class. It does not fork or copy
Ultralytics internals.

`python/shadow/policy.py` implements the ground motion-primitive network.
`dataset.py` reads a versioned JSONL manifest containing depth, odometry,
selected-target state, and privileged 2.5D map paths. `losses.py` computes
collision, traversability, smoothness, kinematic, terminal distance, and
visibility guidance costs. Training uses privileged maps; runtime inference does
not require them as model inputs.

`train_detector.py` and `train_policy.py` are separate entry points because the
detector and planner have different datasets and licensing boundaries.
`export.py` exports two independent fixed-profile artifacts and inspects their
names, dtypes, and dimensions before reporting success.

Checkpoints record schema version, model configuration, input normalization,
and upstream provenance. No model or dataset artifact is committed.

## Failure Handling

Shadow fails closed:

- unsupported or malformed messages reject the current transaction;
- stale RGB-D, odometry, or transforms produce an empty path;
- detector failure invalidates detections and the path for the frame;
- target-localization failure produces no target pose and an empty path;
- policy failure or a malformed tensor produces an empty path;
- a map with no safe candidate produces an empty path;
- writer failure makes `Proc()` return false after attempting all coherent
  outputs;
- `Clear()` releases readers, writers, callbacks, engines, tracks, and map
  state so reinitialization cannot observe stale state.

No failure falls back to an unvalidated learned trajectory or a previous path.

## Build and Deployment Integration

`shadow_component` is an autolink-loadable shared library. The root source
filter excludes `shadow_component.cpp` from `libautonomy` so registration is
owned only by `libshadow_component.so`. The component target is enabled only
when autolink, the project common-network backend, and ONNX Runtime support are
available.

`dag/shadow.dag` declares the fixed four-reader order and loads
`libshadow_component.so`. `launch/shadow.launch` starts Shadow alone.
`autonomy/perception/launch/perception.launch` gains Shadow after Fathom so its
sample refined-depth input has a producer. Installation mirrors Fathom under
`share/autonomy/shadow/{conf,dag,launch}` and installs the component library in
`lib`.

## Verification Strategy

C++ test sources cover:

- letterbox transforms, confidence/class filtering, and YOLO26 tensor parsing;
- high/low confidence association, stable IDs, explicit selection, occlusion,
  timeout, and forbidden target switching;
- robust depth sampling, projection, transform failure, and depth limits;
- grid insertion, rolling, expiry, slope, obstacle inflation, and unknown-cell
  behavior;
- policy tensor validation and trajectory decoding;
- collision rejection, cost ordering, kinematic limits, and empty-path fallback;
- configuration validation and component lifecycle/publication failures.

Python test sources cover manifest validation, deterministic augmentation,
guidance-loss values, checkpoint metadata, and both export contracts. Model and
transport dependencies are injected at seams so unit tests require neither
weights nor live middleware.

Per the explicit project constraint for this work, implementation does not run
compilation, C++ tests, Python tests, training, export, inference, or model
downloads. Verification is limited to source-level checks such as formatting,
`git diff --check`, expected-path inspection, include/reference searches, and
review of staged scope. Test sources are still written before their production
counterparts to preserve the intended contracts.

## Acceptance Criteria

The implementation is accepted when:

- `autonomy/perception/shadow` matches the lowercase layout above;
- all public runtime I/O uses existing automsgs messages;
- the component loads through standalone and combined autolink launch files;
- C++ has independent detector, tracker, localizer, grid, policy, and planner
  units behind `ShadowComponent`;
- Python provides separate detector and policy training plus fixed-profile
  export;
- the 2.5D map contains the four specified layers and fails closed on unknown
  space;
- target loss, stale data, inference failure, and unsafe candidates publish an
  empty path rather than stale motion;
- the tracking task consumes Shadow's fresh target and path without changing
  existing task message schemas;
- provenance and YOLO26 licensing constraints are documented;
- no model weights or datasets are added;
- static verification reports no whitespace, stale-reference, formatting, or
  unintended-scope problems.
