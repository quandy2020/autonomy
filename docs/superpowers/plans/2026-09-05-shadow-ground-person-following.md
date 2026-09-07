# Shadow Ground Person-Following Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a production-structured Shadow autolink component that detects and tracks a selected person, maintains a rolling 2.5D map, and publishes a safe local following path for a ground mobile robot.

**Architecture:** One `ShadowComponent` coordinates six focused C++ units: YOLO26 detection, ByteTrack-style association, RGB-D localization, rolling grid mapping, YOPO-derived motion-primitive inference, and deterministic path safety selection. Python owns detector fine-tuning, ground-policy training, and fixed-profile export; C++ owns deployment and publishes only existing automsgs messages.

**Tech Stack:** C++17, autolink, automsgs, Eigen, OpenCV, grid_map, GoogleTest, autonomy common/network, ONNX Runtime with optional TensorRT backend; Python 3.9+, PyTorch, Ultralytics YOLO26, NumPy, Pillow, pytest, ONNX.

**Spec:** `docs/superpowers/specs/2026-09-05-shadow-ground-person-following-design.md`

## Global Constraints

- All directories are lowercase; C++ and Python filenames use `snake_case`.
- Use one flat C++ source directory under `autonomy/perception/shadow`.
- Use exactly one runtime configuration message, `ShadowOptions`, in `proto/shadow.proto` and one sample file, `conf/shadow.pb.txt`.
- Use existing automsgs messages for every public runtime interface; do not add a Shadow data-message proto.
- Publish `Detection2DArray`, `PoseStamped`, `Path`, and `GridMap`; consume `Image`, `CameraInfo`, `Odometry`, and `std_msgs::String` selection.
- Shadow must never publish `/cmd_vel`; navigation and control retain motion execution.
- Use `autonomy::common::network::Engine`; do not introduce a generic inference framework.
- Detector and policy artifacts use fixed tensor profiles validated at component initialization.
- YOLO26 code and weights remain external. Document its AGPL-3.0 or enterprise licensing boundary.
- Adapt only the YOPO concepts needed for ground motion primitives and guidance losses. Retain the MIT attribution for code derived from YOPO commit `58dda5b2d124aa3998593dd62798f1496c93b26d`.
- Do not add model weights, TensorRT engines, ONNX files, datasets, checkpoints, or generated outputs.
- Treat unknown grid cells as impassable and reject unsafe, stale, or malformed planning inputs.
- An empty stamped `Path` invalidates previous motion whenever no safe current path exists.
- Do not modify or discard unrelated working-tree changes.
- Per the user's execution constraint, write source and test code but do not compile, run C++ tests, run Python tests, execute Python, train, export, infer, or download models.
- Because runtime execution is forbidden, each test source is written before its production source, then reviewed statically; no red/green execution claim may be made.

## File Map

### Shadow C++ and deployment

- `autonomy/perception/shadow/proto/shadow.proto`: single runtime configuration schema.
- `autonomy/perception/shadow/conf/shadow.pb.txt`: sample fixed-profile configuration.
- `autonomy/perception/shadow/options.hpp`: `ShadowOptions` validation declaration.
- `autonomy/perception/shadow/options.cpp`: complete range and cross-field validation.
- `autonomy/perception/shadow/detector.hpp`: detector runner seam and `YoloDetector` API.
- `autonomy/perception/shadow/detector.cpp`: letterbox, common-network adapter, YOLO26 parsing, and automsgs conversion.
- `autonomy/perception/shadow/tracker.hpp`: selected-person tracking API.
- `autonomy/perception/shadow/tracker.cpp`: two-stage association, track lifecycle, and target locking.
- `autonomy/perception/shadow/localizer.hpp`: RGB-D target localization API.
- `autonomy/perception/shadow/localizer.cpp`: robust depth estimate, projection, map transform, and velocity filter.
- `autonomy/perception/shadow/grid.hpp`: rolling 2.5D local-map API.
- `autonomy/perception/shadow/grid.cpp`: depth insertion, map rolling, expiry, obstacle classification, and inflation.
- `autonomy/perception/shadow/policy.hpp`: policy runner seam and `YopoPolicy` API.
- `autonomy/perception/shadow/policy.cpp`: policy preprocessing, fixed-contract inference, and candidate decoding.
- `autonomy/perception/shadow/planner.hpp`: deterministic candidate validation and ranking API.
- `autonomy/perception/shadow/planner.cpp`: grid-footprint checks, cost calculation, and `Path` selection.
- `autonomy/perception/shadow/shadow_component.hpp`: four-input autolink component declaration.
- `autonomy/perception/shadow/shadow_component.cpp`: lifecycle, TF lookup, selection reader, orchestration, and publication.
- `autonomy/perception/shadow/CMakeLists.txt`: isolated component DSO and component test target.
- `autonomy/perception/shadow/dag/shadow.dag`: component load and ordered input readers.
- `autonomy/perception/shadow/launch/shadow.launch`: standalone launch.
- `autonomy/perception/shadow/README.md`: provenance, contracts, topics, training, and deployment.

### Shadow tests

- `autonomy/perception/shadow/options_test.cpp`: invalid option boundaries and valid sample.
- `autonomy/perception/shadow/detector_test.cpp`: preprocessing and output parsing.
- `autonomy/perception/shadow/tracker_test.cpp`: IDs, association passes, selection, and loss.
- `autonomy/perception/shadow/localizer_test.cpp`: depth statistics, projection, transform, and velocity reset.
- `autonomy/perception/shadow/grid_test.cpp`: layers, rolling, expiry, slope, and inflation.
- `autonomy/perception/shadow/policy_test.cpp`: tensor construction, contract rejection, and path decoding.
- `autonomy/perception/shadow/planner_test.cpp`: safety rejection, cost order, and empty-path behavior.
- `autonomy/perception/shadow/shadow_component_test.cpp`: initialization, orchestration, and publisher failures.

### Python

- `autonomy/perception/shadow/python/shadow/__init__.py`: stable package exports.
- `autonomy/perception/shadow/python/shadow/dataset.py`: versioned JSONL policy dataset.
- `autonomy/perception/shadow/python/shadow/detector.py`: installed-Ultralytics detector facade.
- `autonomy/perception/shadow/python/shadow/policy.py`: ground motion-primitive policy.
- `autonomy/perception/shadow/python/shadow/losses.py`: differentiable guidance costs.
- `autonomy/perception/shadow/python/shadow/train_detector.py`: detector fine-tuning CLI.
- `autonomy/perception/shadow/python/shadow/train_policy.py`: policy training CLI and checkpointing.
- `autonomy/perception/shadow/python/shadow/export.py`: detector and policy fixed-profile export.
- `autonomy/perception/shadow/python/test/test_dataset.py`: manifest and tensor tests.
- `autonomy/perception/shadow/python/test/test_detector.py`: Ultralytics facade argument tests.
- `autonomy/perception/shadow/python/test/test_policy.py`: policy shape and primitive tests.
- `autonomy/perception/shadow/python/test/test_losses.py`: literal guidance-loss expectations.
- `autonomy/perception/shadow/python/test/test_export.py`: exported name and shape checks.

### Existing integration files

- `CMakeLists.txt`: add the Shadow component subdirectory beside Fathom.
- `cmake/autonomy_sources.cmake`: keep the component entrypoint out of `libautonomy` and exclude concrete model sources without ONNX Runtime.
- `cmake/autonomy_install.cmake`: install the Shadow DSO, DAG, config, and launch.
- `autonomy/perception/launch/perception.launch`: start Shadow after Fathom.
- `autonomy/task/tracking/tracking_client.hpp`: Shadow transport and latest-state API.
- `autonomy/task/tracking/tracking_client.cpp`: selection publication, subscriptions, freshness, and cancellation.
- `autonomy/task/tracking/plugins/action/follow_shadow_path_action.cpp`: continuously preempt FollowPath with fresh Shadow paths.
- `autonomy/task/tracking/plugins/condition/target_locked_condition.cpp`: require a fresh Shadow target in person mode.
- `autonomy/task/tracking/tracking.cpp`: populate feedback from Shadow state.
- `config/task/behavior_tree/tracking/follow_person.xml`: use `FollowShadowPath` rather than global planning.
- `config/task/behavior_tree/tree_nodes_model.xml`: declare the new BT node ports.

---

### Task 1: Add the single configuration contract

**Files:**
- Create: `autonomy/perception/shadow/proto/shadow.proto`
- Create: `autonomy/perception/shadow/conf/shadow.pb.txt`
- Create: `autonomy/perception/shadow/options.hpp`
- Create: `autonomy/perception/shadow/options.cpp`
- Create: `autonomy/perception/shadow/options_test.cpp`

**Interfaces:**
- Produces: `bool ValidateShadowOptions(const proto::ShadowOptions&, std::string* error)`.
- Produces: protobuf C++ type `autonomy::perception::shadow::proto::ShadowOptions`.
- Consumes: no Shadow production API.

- [ ] **Step 1: Write the option-contract test source first**

Create a `ValidOptions()` fixture with model paths, `onnx` backends, detector
profile `640x640x300`, policy profile `160x96`, 64 candidates, 12 steps, a
`10x10 m` map at `0.05 m`, and non-empty output topics. Add table-driven tests
that independently mutate one field and expect rejection for empty paths,
unknown backends, zero dimensions, invalid thresholds, non-positive timeouts,
invalid depth range, invalid map geometry, `inflation_radius < robot_radius`,
negative planner weights, and all-zero planner weights.

```cpp
TEST(ShadowOptionsTest, RejectsInflationSmallerThanRobot) {
    auto options = ValidOptions();
    options.set_robot_radius(0.30F);
    options.set_inflation_radius(0.20F);
    std::string error;
    EXPECT_FALSE(ValidateShadowOptions(options, &error));
    EXPECT_FALSE(error.empty());
}
```

- [ ] **Step 2: Define `ShadowOptions` with stable field numbers**

Use fields 1-9 for model paths/backends and detector profile, 10-15 for tracker
thresholds/timeouts, 16-20 for depth localization, 21-33 for frames and map,
34-39 for policy dimensions/kinematics, 40 for follow distance, 41-47 for
planner weights, 48-52 for selection/output topics, and 53-55 for timing/depth
scale. The exact field
names are:

```text
detector_model_path, policy_model_path, detector_backend, policy_backend,
detector_width, detector_height, max_detections, person_class_id,
confidence_threshold, track_high_threshold, track_low_threshold,
association_iou_threshold, min_confirmed_hits, prediction_timeout_sec,
lost_timeout_sec, inner_box_scale, min_depth_m, max_depth_m, min_depth_samples,
depth_outlier_m, map_frame, base_frame, camera_frame, map_length_x,
map_length_y, map_resolution, map_roll_threshold, cell_ttl_sec,
max_step_height, max_slope_rad, obstacle_min_height, robot_radius,
inflation_radius, policy_width, policy_height, candidate_count,
trajectory_steps, max_linear_speed, max_angular_speed, follow_distance,
learned_weight, clearance_weight, traversability_weight, curvature_weight,
progress_weight, distance_weight, visibility_weight, select_topic, detections_topic,
target_topic, path_topic, grid_topic, max_input_skew_sec, max_data_age_sec,
depth_scale
```

- [ ] **Step 3: Implement complete validation and the sample config**

Validate every invariant named in the test. Use sample thresholds
`confidence=0.35`, `high=0.50`, `low=0.10`, `association_iou=0.30`, prediction
timeout `0.35 s`, lost timeout `1.5 s`, valid depth `[0.20, 8.0] m`, robot
radius `0.25 m`, inflation radius `0.35 m`, maximum linear speed `0.8 m/s`,
maximum angular speed `1.2 rad/s`, and `follow_distance=1.5 m`.

- [ ] **Step 4: Perform static checks only**

Run `clang-format --dry-run --Werror` on the C++ files and `git diff --check` on
the five task files. Inspect that no second config/options message exists. Do
not invoke protoc, compile, or run `options_test`.

- [ ] **Step 5: Commit the configuration unit**

```bash
git add autonomy/perception/shadow/proto autonomy/perception/shadow/conf \
  autonomy/perception/shadow/options.hpp autonomy/perception/shadow/options.cpp \
  autonomy/perception/shadow/options_test.cpp
git commit -m "feat(perception): define Shadow options"
```

### Task 2: Implement YOLO26 person detection

**Files:**
- Create: `autonomy/perception/shadow/detector.hpp`
- Create: `autonomy/perception/shadow/detector.cpp`
- Create: `autonomy/perception/shadow/detector_test.cpp`

**Interfaces:**
- Consumes: Task 1 `proto::ShadowOptions` and common-network tensors.
- Produces: `DetectorRunner::Run(const TensorMap&, TensorMap*, std::string*)`.
- Produces: `YoloDetector::Create(const proto::ShadowOptions&, std::string* error = nullptr)`.
- Produces: `YoloDetector::Create(const proto::ShadowOptions&, std::unique_ptr<DetectorRunner>, std::string* error = nullptr)` for dependency injection.
- Produces: `bool YoloDetector::Detect(const sensor_msgs::Image&, vision_msgs::Detection2DArray*, std::string*)`.

- [ ] **Step 1: Write detector behavior tests first**

Use a fake runner that captures `images` and returns `output0` through
`Tensor::FromFloat32`. Verify RGB/BGR conversion, letterbox padding, normalized
NCHW order, filtering class `0`, threshold equality, clipping, inverse scaling,
invalid box rejection, wrong tensor size rejection, and output clearing on
failure.

```cpp
TEST(YoloDetectorTest, RestoresLetterboxedPersonBox) {
    auto runner = MakeRunner({100, 160, 300, 480, 0.9F, 0.0F});
    auto detector = YoloDetector::Create(Options640(), std::move(runner));
    automsgs::msgs::vision_msgs::Detection2DArray detections;
    ASSERT_TRUE(detector->Detect(MakeRgbImage(640, 320), &detections));
    ASSERT_EQ(detections.detections_size(), 1);
    EXPECT_EQ(detections.detections(0).results(0).hypothesis().class_id(),
              "person");
}
```

- [ ] **Step 2: Implement detector preprocessing and parsing**

Preserve aspect ratio with centered letterbox padding value `114`, accept
`rgb8` and `bgr8`, normalize to `[0,1]`, and bind `images`. Parse exactly
`[1,max_detections,6]` rows as `xyxy/confidence/class_id`; do not run NMS.
Populate automsgs box centers/sizes and hypothesis score. Leave detection IDs
empty for Task 3.

- [ ] **Step 3: Add the concrete common-network runner**

Create the engine with `InferenceOptions{backend_id, model_path}` and validate
the exact input/output names, float32 dtype, and static shapes before returning
the detector. Keep `common::network::Engine` private to `detector.cpp`.

- [ ] **Step 4: Perform static checks and commit**

Format-check the three C++ files, run `git diff --check`, and inspect that
`detector.hpp` exposes automsgs rather than a new public detection type. Do not
compile or run the test.

```bash
git add autonomy/perception/shadow/detector.hpp \
  autonomy/perception/shadow/detector.cpp \
  autonomy/perception/shadow/detector_test.cpp
git commit -m "feat(perception): add Shadow YOLO detector"
```

### Task 3: Add stable selected-person tracking

**Files:**
- Create: `autonomy/perception/shadow/tracker.hpp`
- Create: `autonomy/perception/shadow/tracker.cpp`
- Create: `autonomy/perception/shadow/tracker_test.cpp`

**Interfaces:**
- Consumes: `vision_msgs::Detection2DArray` from Task 2.
- Produces: `bool PersonTracker::Update(int64_t stamp_ns, Detection2DArray*, std::string*)`.
- Produces: `void PersonTracker::Select(const std::string& target_id)`.
- Produces: `bool PersonTracker::Selected(Detection2D*) const`.
- Produces: `void PersonTracker::Confirmed(Detection2DArray*) const`.
- Produces: `bool PersonTracker::selected_visible() const`, `bool PersonTracker::selected_predicted() const`, and `void PersonTracker::Clear()`.

- [ ] **Step 1: Write tracker tests first**

Add literal box sequences covering first confirmation, monotonic string IDs,
high-confidence association, low-confidence recovery, crossing people without
selected-ID switching, prediction timeout, lost timeout, explicit unknown ID,
empty-selection clearing, confirmed-track enumeration, and `Clear()`.

```cpp
TEST(PersonTrackerTest, DoesNotSwitchSelectedPersonAtCrossing) {
    PersonTracker tracker(ValidOptions());
    auto first = Detections({Box(10, 10, 40, 80, 0.9),
                             Box(100, 10, 40, 80, 0.9)});
    ASSERT_TRUE(tracker.Update(1'000'000'000, &first));
    tracker.Select(first.detections(0).id());
    auto crossed = Detections({Box(95, 10, 40, 80, 0.9),
                               Box(15, 10, 40, 80, 0.9)});
    ASSERT_TRUE(tracker.Update(1'100'000'000, &crossed));
    automsgs::msgs::vision_msgs::Detection2D selected;
    ASSERT_TRUE(tracker.Selected(&selected));
    EXPECT_EQ(selected.id(), first.detections(0).id());
}
```

- [ ] **Step 2: Implement two-pass association**

Keep the Kalman box state private. Predict all tracks, greedily apply globally
sorted IoU candidates for high-confidence detections, repeat for low-confidence
detections against unmatched confirmed tracks, create tracks only from high
confidence, and remove tracks only after `lost_timeout_sec`. Write stable IDs
back to the automsgs detections.

- [ ] **Step 3: Implement strict selected-target lifecycle**

An explicit ID selects only that track. Empty selection clears the current lock
and enables component-level automatic selection. Return confirmed visible
tracks in stable ID order so Task 8 can compare measured RGB-D ranges. Preserve
a selected ID through prediction timeout, mark it lost after loss timeout, and
never replace it implicitly.

- [ ] **Step 4: Perform static checks and commit**

Format-check, run `git diff --check`, and inspect tests against each lifecycle
branch. Do not compile or execute tests.

```bash
git add autonomy/perception/shadow/tracker.hpp \
  autonomy/perception/shadow/tracker.cpp \
  autonomy/perception/shadow/tracker_test.cpp
git commit -m "feat(perception): track selected person in Shadow"
```

### Task 4: Localize and filter the selected person in RGB-D

**Files:**
- Create: `autonomy/perception/shadow/localizer.hpp`
- Create: `autonomy/perception/shadow/localizer.cpp`
- Create: `autonomy/perception/shadow/localizer_test.cpp`

**Interfaces:**
- Consumes: selected `Detection2D`, aligned depth `Image`, `CameraInfo`, and camera-to-map `TransformStamped`.
- Produces: `bool TargetLocalizer::EstimateRange(const Detection2D&, const Image&, const CameraInfo&, float* range_m, std::string*) const` without changing filter history.
- Produces: `bool TargetLocalizer::Locate(const std::string& target_id, int64_t stamp_ns, const Detection2D&, const Image&, const CameraInfo&, const TransformStamped&, PoseStamped*, TwistStamped*, std::string*)`.
- Produces: `void TargetLocalizer::Clear()`.

- [ ] **Step 1: Write localization tests first**

Test `16UC1` scaling and metric `32FC1`, inner-box cropping, invalid values,
median depth, outlier rejection, too few samples, invalid intrinsics, known rigid
transform, two-frame planar velocity, non-monotonic time rejection, and velocity
reset when `target_id` changes. Verify `EstimateRange` leaves velocity history
unchanged while returning literal near/far ranges for automatic selection.

```cpp
TEST(TargetLocalizerTest, ResetsVelocityForNewTarget) {
    TargetLocalizer localizer(ValidOptions());
    automsgs::msgs::geometry_msgs::PoseStamped pose;
    automsgs::msgs::geometry_msgs::TwistStamped velocity;
    ASSERT_TRUE(localizer.Locate("1", 1'000'000'000, Box(), DepthAt(2.0F),
                                 Camera(), IdentityTransform(), &pose, &velocity));
    ASSERT_TRUE(localizer.Locate("2", 2'000'000'000, Box(), DepthAt(3.0F),
                                 Camera(), IdentityTransform(), &pose, &velocity));
    EXPECT_DOUBLE_EQ(velocity.twist().linear().x(), 0.0);
}
```

- [ ] **Step 2: Implement depth estimation and projection**

Decode image rows using `step`, sample the configured inner box, scale samples,
discard non-finite/out-of-range values, compute the median, discard values more
than `depth_outlier_m` from it, then recompute the median. Back-project the box
center with `x=(u-cx)z/fx`, `y=(v-cy)z/fy`, `z=depth`.

- [ ] **Step 3: Implement map transform and velocity filtering**

Apply the supplied rigid transform, stamp the output pose in `map`, set identity
orientation, and derive planar velocity from consecutive positions with an
exponential filter. Reset history on target change, invalid time order, or
`Clear()`.

- [ ] **Step 4: Perform static checks and commit**

Format-check, run `git diff --check`, and inspect that all output messages clear
before failure. Do not compile or run tests.

```bash
git add autonomy/perception/shadow/localizer.hpp \
  autonomy/perception/shadow/localizer.cpp \
  autonomy/perception/shadow/localizer_test.cpp
git commit -m "feat(perception): localize Shadow targets"
```

### Task 5: Maintain the rolling 2.5D grid map

**Files:**
- Create: `autonomy/perception/shadow/grid.hpp`
- Create: `autonomy/perception/shadow/grid.cpp`
- Create: `autonomy/perception/shadow/grid_test.cpp`

**Interfaces:**
- Consumes: aligned depth, `CameraInfo`, camera-to-map transform, and `Odometry`.
- Produces: `bool LocalGrid::Update(int64_t stamp_ns, const Image&, const CameraInfo&, const TransformStamped&, const Odometry&, std::string*)`.
- Produces: `const grid_map::GridMap& LocalGrid::map() const`.
- Produces: `bool LocalGrid::ToMessage(map_msgs::GridMap*, std::string*) const` and `void LocalGrid::Clear()`.

- [ ] **Step 1: Write grid behavior tests first**

Create small deterministic `1.0x1.0 m` maps at `0.1 m` resolution. Assert the
four exact layer names, finite elevation/variance for observed cells, unknown
NaN initialization, obstacle classification above `obstacle_min_height`, slope
cost, inflation radius, roll preservation, newly exposed unknown cells, TTL
expiry, and failed-update atomicity.

```cpp
TEST(LocalGridTest, UnknownCellsRemainImpassable) {
    LocalGrid grid(ValidOptions());
    const auto& map = grid.map();
    EXPECT_FALSE(std::isfinite(map.at("traversability", {0, 0})));
}
```

- [ ] **Step 2: Implement atomic depth insertion**

Copy the current grid to a candidate, move it around the odometry pose when the
roll threshold is crossed, back-project stride-sampled depth, transform points
to `map`, and update per-cell elevation mean/variance plus last-observed time.
Commit the candidate only after all validation succeeds.

- [ ] **Step 3: Derive obstacles and traversability**

Mark cells whose observed height relative to the robot support plane exceeds
`obstacle_min_height`, mark excessive neighbor step or slope, compute
traversability in `[0,1]`, inflate impassable cells through a circle iterator,
and restore expired cells to NaN in all four layers.

- [ ] **Step 4: Convert through the existing automsgs converter**

Call `grid_map::GridMapConverter::toMessage`; do not create another GridMap
serializer. Preserve the grid timestamp and `map` frame.

- [ ] **Step 5: Perform static checks and commit**

Format-check, run `git diff --check`, and search for exactly four `map.add`
layer declarations. Do not compile or run tests.

```bash
git add autonomy/perception/shadow/grid.hpp autonomy/perception/shadow/grid.cpp \
  autonomy/perception/shadow/grid_test.cpp
git commit -m "feat(perception): add Shadow local grid"
```

### Task 6: Adapt YOPO motion primitives for a ground robot

**Files:**
- Create: `autonomy/perception/shadow/policy.hpp`
- Create: `autonomy/perception/shadow/policy.cpp`
- Create: `autonomy/perception/shadow/policy_test.cpp`

**Interfaces:**
- Consumes: Task 1 options, metric depth, odometry, target pose, and target velocity.
- Produces: `PolicyRunner::Run(const TensorMap&, TensorMap*, std::string*)`.
- Produces: `YopoPolicy::Create(const proto::ShadowOptions&, std::string* error = nullptr)`.
- Produces: `YopoPolicy::Create(const proto::ShadowOptions&, std::unique_ptr<PolicyRunner>, std::string* error = nullptr)` for dependency injection.
- Produces: `bool YopoPolicy::Generate(const Image&, const Odometry&, const PoseStamped&, const TwistStamped&, std::vector<Path>*, std::vector<float>*, std::string*)`.

- [ ] **Step 1: Write policy tests first**

Capture and assert tensors `depth [1,1,h,w]`, `robot_state [1,2]`, and
`target_state [1,4]`. Return literal `trajectories` and `scores` and verify
candidate count, step order, base-link frame, quaternion yaw, non-finite output
rejection, wrong rank rejection, score-count rejection, and output clearing.

```cpp
TEST(YopoPolicyTest, DecodesGroundTrajectoryInBaseLink) {
    auto runner = MakePolicyRunner({0.0F, 0.0F, 0.0F,
                                    0.5F, 0.1F, 0.2F}, {0.25F});
    auto policy = YopoPolicy::Create(OneCandidateOptions(), std::move(runner));
    std::vector<automsgs::msgs::nav_msgs::Path> paths;
    std::vector<float> scores;
    ASSERT_TRUE(policy->Generate(Depth(), Odom(), Target(), TargetVelocity(),
                                 &paths, &scores));
    EXPECT_EQ(paths.front().header().frame_id(), "base_link");
    EXPECT_DOUBLE_EQ(paths.front().poses(1).pose().position().x(), 0.5);
}
```

- [ ] **Step 2: Implement preprocessing and decoding**

Resize sanitized metric depth to the fixed policy profile. Read odometry
`linear.x` and `angular.z`; transform target position/velocity from `map` to
the robot frame using odometry yaw. Bind the three exact input names and decode
`trajectories [1,candidate_count,trajectory_steps,3]` plus
`scores [1,candidate_count]`.

- [ ] **Step 3: Add the concrete model adapter and provenance comments**

Validate common-network metadata before inference. Document in file headers
which motion-primitive concepts derive from YOPO and retain the MIT notice for
any copied implementation fragment. Do not copy aerial attitude, thrust, or
differential-flatness code.

- [ ] **Step 4: Perform static checks and commit**

Format-check, run `git diff --check`, and search for forbidden aerial output
terms in the public API. Do not compile or run tests.

```bash
git add autonomy/perception/shadow/policy.hpp \
  autonomy/perception/shadow/policy.cpp \
  autonomy/perception/shadow/policy_test.cpp
git commit -m "feat(perception): add Shadow ground policy"
```

### Task 7: Select a safe following path

**Files:**
- Create: `autonomy/perception/shadow/planner.hpp`
- Create: `autonomy/perception/shadow/planner.cpp`
- Create: `autonomy/perception/shadow/planner_test.cpp`

**Interfaces:**
- Consumes: candidate `Path` values/scores, `GridMap`, odometry, target pose, and `ShadowOptions`.
- Produces: `bool FollowPlanner::Select(int64_t stamp_ns, const std::vector<Path>&, const std::vector<float>&, const grid_map::GridMap&, const Odometry&, const PoseStamped&, Path*, std::string*) const`.

- [ ] **Step 1: Write safety and ranking tests first**

Use hand-filled grids and two literal paths. Assert rejection outside the map,
unknown-cell rejection, obstacle/inflated-footprint rejection, linear and
angular speed limits, first pose at the robot, terminal-distance preference,
clearance preference, learned-score tie breaking, all-unsafe empty stamped path,
and mismatched score count failure.

```cpp
TEST(FollowPlannerTest, ReturnsStampedEmptyPathWhenEveryCandidateIsUnsafe) {
    automsgs::msgs::nav_msgs::Path output;
    ASSERT_TRUE(planner.Select(2'000'000'000, {PathThroughObstacle()}, {0.1F},
                               BlockedGrid(), Odom(), Target(), &output));
    EXPECT_EQ(output.header().frame_id(), "map");
    EXPECT_TRUE(output.poses().empty());
}
```

- [ ] **Step 2: Implement map-frame conversion and footprint sampling**

Transform every base-link pose through current odometry into `map`; prepend the
robot pose; sample segments no farther apart than half a grid cell; test a
circle of `robot_radius` at each sample. Reject non-finite, unknown,
`obstacle >= 0.5`, or `traversability >= 1.0` cells.

- [ ] **Step 3: Implement normalized deterministic costs**

Calculate weighted learned score, inverse clearance, mean traversability,
absolute curvature change, negative target progress, terminal follow-distance
error, and line-of-sight blockage. Reject negative/non-finite totals and choose
the lowest value with original candidate index as deterministic tie break.

- [ ] **Step 4: Perform static checks and commit**

Format-check, run `git diff --check`, and inspect every unsafe branch for an
empty-path or rejected-candidate result. Do not compile or run tests.

```bash
git add autonomy/perception/shadow/planner.hpp \
  autonomy/perception/shadow/planner.cpp \
  autonomy/perception/shadow/planner_test.cpp
git commit -m "feat(perception): select safe Shadow paths"
```

### Task 8: Assemble the autolink component and deployment files

**Files:**
- Create: `autonomy/perception/shadow/shadow_component.hpp`
- Create: `autonomy/perception/shadow/shadow_component.cpp`
- Create: `autonomy/perception/shadow/shadow_component_test.cpp`
- Create: `autonomy/perception/shadow/CMakeLists.txt`
- Create: `autonomy/perception/shadow/dag/shadow.dag`
- Create: `autonomy/perception/shadow/launch/shadow.launch`
- Modify: `CMakeLists.txt`
- Modify: `cmake/autonomy_sources.cmake`
- Modify: `cmake/autonomy_install.cmake`
- Modify: `autonomy/perception/launch/perception.launch`

**Interfaces:**
- Consumes: Tasks 1-7 and four ordered automsgs component readers.
- Produces: `ShadowComponent::Init`, four-argument `Proc`, and `Clear`.
- Produces: automsgs writers for detections, target pose, path, and grid.

- [ ] **Step 1: Write component lifecycle tests first**

Use callable seams for processing, TF lookup, and each publisher. Assert null
input rejection, not-initialized rejection, input-skew rejection, stale-data
empty path, selected-target propagation, processing failure, all four successful
publications, independent writer-failure reporting, and `Clear()` state reset.

- [ ] **Step 2: Implement initialization**

Load and validate `ShadowOptions`; create detector, tracker, localizer, grid,
policy, and planner once; create the selection reader and four writers; bind
callbacks; start `transform::AutolinkTfListener` on `/tf` and `/tf_static` so
the component process owns a populated transform buffer; fail with the
module-specific error and call `Clear()` on any partial initialization.

- [ ] **Step 3: Implement one coherent processing transaction**

Validate four messages and timestamp skew, obtain camera-to-map TF at RGB time,
detect, track, update the map, resolve an empty selection request by comparing
`EstimateRange` for confirmed tracks, localize the locked target, generate
candidates, select a path, and build all messages before publishing. Publish an empty path
on stale data, lost target, localization failure, policy failure, or no safe
candidate. Attempt all coherent output writes and return their conjunction.

- [ ] **Step 4: Add DSO, DAG, launch, source filtering, and installation**

Build `shadow_component` only with autolink plus ONNX Runtime; keep its
registration source out of `libautonomy`; exclude detector and policy sources
that directly depend on the concrete engine when the backend is unavailable.
Use DAG reader order RGB, refined depth, CameraInfo, Odometry. Start `fathom`
before `shadow` in the combined launch and install under
`share/autonomy/shadow`.

- [ ] **Step 5: Perform static integration checks and commit**

Format-check all component C++, run `git diff --check`, inspect DAG class and
library names, inspect the combined launch ordering, and use `rg` to confirm a
single registration macro. Do not configure or build CMake and do not start a
launch file.

```bash
git add autonomy/perception/shadow/CMakeLists.txt \
  autonomy/perception/shadow/shadow_component.hpp \
  autonomy/perception/shadow/shadow_component.cpp \
  autonomy/perception/shadow/shadow_component_test.cpp \
  autonomy/perception/shadow/dag autonomy/perception/shadow/launch \
  CMakeLists.txt cmake/autonomy_sources.cmake cmake/autonomy_install.cmake \
  autonomy/perception/launch/perception.launch
git commit -m "feat(perception): add Shadow component"
```

### Task 9: Connect Shadow to the tracking task

**Files:**
- Modify: `autonomy/task/tracking/tracking_client.hpp`
- Modify: `autonomy/task/tracking/tracking_client.cpp`
- Modify: `autonomy/task/tracking/tracking.cpp`
- Create: `autonomy/task/tracking/plugins/action/follow_shadow_path_action.cpp`
- Modify: `autonomy/task/tracking/plugins/condition/target_locked_condition.cpp`
- Modify: `config/task/behavior_tree/tracking/follow_person.xml`
- Modify: `config/task/behavior_tree/tree_nodes_model.xml`
- Create: `autonomy/task/tracking/tracking_client_test.cpp`
- Create: `autonomy/task/tracking/plugins/action/follow_shadow_path_action_test.cpp`

**Interfaces:**
- Consumes: `/perception/shadow/target` and `/perception/shadow/path`.
- Produces: `/perception/shadow/select` using `std_msgs::String`.
- Produces: `bool TrackingClient::GetShadowPath(Path*, uint64_t* revision) const`.
- Produces: `bool TrackingClient::GetShadowTarget(PoseStamped*) const`.
- Produces: BT action `FollowShadowPath` with `controller_id`, `error_code_id`, and `error_msg` ports.

- [ ] **Step 1: Write task-client behavior tests first**

Inject selection publication and Shadow message callbacks. Assert START and
UPDATE publish the requested ID, empty ID requests nearest selection, fresh
target/path are returned, empty path clears lock, stale messages fail, cancel
cancels the follow action, and target-pose mode retains existing behavior.

- [ ] **Step 2: Add Shadow transport state to `TrackingClient`**

The node-based factory creates the selection writer plus target/path readers.
Protect callback state with one mutex. Increment a monotonic path revision on
every received path, retain receive time, and make getters enforce the fixed
`kShadowDataTimeout = 500 ms` freshness bound.
The navigation-only factory remains usable in existing tests with Shadow
transport disabled.

- [ ] **Step 3: Implement continuous `FollowShadowPath` preemption**

On first fresh non-empty path, begin a `FollowPathAction` session. On every tick,
if the revision changed, call `ActionSession::Begin` with the new path; this
cancels the accepted old goal and sends the new goal to the controller's
preemption-capable action server. Empty or stale paths cancel the session and
return failure. Halt also cancels the session.

- [ ] **Step 4: Replace the person-follow behavior tree path**

Use `TargetLocked` followed by `FollowShadowPath`. Remove `ComputeFollowGoal`,
global `PlanPath`, and `SmoothPath` only from `follow_person.xml`; preserve them
for target-pose mode and other navigation trees. Register the new node model.

- [ ] **Step 5: Fill tracking feedback from Shadow**

For person mode, populate `target_pose` and Euclidean robot-target distance from
the fresh Shadow target. Map absent or stale target state to
`TRACKER_STATUS_TARGET_LOST` while the task remains eligible for reacquisition.

- [ ] **Step 6: Perform static checks and commit**

Format-check modified C++, run `git diff --check`, inspect the XML, and search
that person mode no longer invokes global `PlanPath`. Do not compile, run tests,
or start task/control processes.

```bash
git add autonomy/task/tracking/tracking_client.hpp \
  autonomy/task/tracking/tracking_client.cpp \
  autonomy/task/tracking/tracking_client_test.cpp \
  autonomy/task/tracking/tracking.cpp \
  autonomy/task/tracking/plugins/action/follow_shadow_path_action.cpp \
  autonomy/task/tracking/plugins/action/follow_shadow_path_action_test.cpp \
  autonomy/task/tracking/plugins/condition/target_locked_condition.cpp \
  config/task/behavior_tree/tracking/follow_person.xml \
  config/task/behavior_tree/tree_nodes_model.xml
git commit -m "feat(task): follow Shadow local paths"
```

### Task 10: Add the external YOLO26 Python facade

**Files:**
- Create: `autonomy/perception/shadow/python/shadow/__init__.py`
- Create: `autonomy/perception/shadow/python/shadow/detector.py`
- Create: `autonomy/perception/shadow/python/shadow/train_detector.py`
- Create: `autonomy/perception/shadow/python/test/test_detector.py`

**Interfaces:**
- Produces: `load_detector(checkpoint: str, factory: Optional[Callable[[str], Any]] = None) -> Any`.
- Produces: `train_detector(checkpoint: str, data: str, output: Path, epochs: int, image_size: int, batch_size: int, factory: Optional[Callable[[str], Any]] = None) -> Path`.
- Consumes: installed `ultralytics.YOLO`; never imports vendored Ultralytics code.

- [ ] **Step 1: Write facade tests first**

Inject a fake YOLO factory and assert exact checkpoint, `train`, person-class,
image-size, batch, epoch, project, and run-name arguments. Assert missing output
artifacts raise `RuntimeError` and imports report a licensing-aware dependency
message when Ultralytics is absent.

```python
def test_train_detector_restricts_training_to_person(tmp_path):
    fake = FakeYolo()
    train_detector("yolo26n.pt", "people.yaml", tmp_path, 10, 640, 8,
                   factory=lambda _: fake)
    assert fake.train_kwargs["classes"] == [0]
    assert fake.train_kwargs["imgsz"] == 640
```

- [ ] **Step 2: Implement the facade and CLI**

Lazy-import Ultralytics, create `YOLO(checkpoint)`, call `train` with
`classes=[0]`, and return the resolved best checkpoint. Require CLI arguments
`--checkpoint`, `--data`, and `--output`; expose explicit epochs, image size,
batch size, device, workers, and seed.

- [ ] **Step 3: Review Python source and commit**

Inspect imports, type hints, deterministic argument forwarding, license text,
and `git diff --check`. Do not invoke Python, pytest, training, or downloads.

```bash
git add autonomy/perception/shadow/python/shadow/__init__.py \
  autonomy/perception/shadow/python/shadow/detector.py \
  autonomy/perception/shadow/python/shadow/train_detector.py \
  autonomy/perception/shadow/python/test/test_detector.py
git commit -m "feat(perception): add Shadow detector training"
```

### Task 11: Add ground-policy data, losses, training, and export

**Files:**
- Create: `autonomy/perception/shadow/python/shadow/dataset.py`
- Create: `autonomy/perception/shadow/python/shadow/policy.py`
- Create: `autonomy/perception/shadow/python/shadow/losses.py`
- Create: `autonomy/perception/shadow/python/shadow/train_policy.py`
- Create: `autonomy/perception/shadow/python/shadow/export.py`
- Create: `autonomy/perception/shadow/python/test/test_dataset.py`
- Create: `autonomy/perception/shadow/python/test/test_policy.py`
- Create: `autonomy/perception/shadow/python/test/test_losses.py`
- Create: `autonomy/perception/shadow/python/test/test_export.py`

**Interfaces:**
- Produces: `ShadowDataset(manifest: Path)` samples with `depth`, `robot_state`, `target_state`, and privileged map tensors.
- Produces: `GroundYopoPolicy.forward(depth, robot_state, target_state) -> tuple[Tensor, Tensor]`.
- Produces: `guidance_loss(trajectories, scores, batch, weights) -> LossOutput`, where `LossOutput` contains `total`, `collision`, `traversability`, `smoothness`, `kinematic`, `distance`, and `visibility` tensors.
- Produces: `export_detector(checkpoint: str, output: Path, image_size: int, max_detections: int) -> Path`.
- Produces: `export_policy(checkpoint: Path, output: Path, policy_height: int, policy_width: int) -> Path`.

- [ ] **Step 1: Write dataset and model-contract tests first**

Use a version-1 JSONL record with relative `.npy` paths. Assert path resolution,
float32 conversion, exact tensor shapes, finite values, malformed schema
rejection, policy output `[B,K,T,3]` and `[B,K]`, and deterministic primitive
anchors.

```python
def test_policy_shapes():
    model = GroundYopoPolicy(candidate_count=8, trajectory_steps=6)
    trajectories, scores = model(
        torch.zeros(2, 1, 96, 160),
        torch.zeros(2, 2),
        torch.zeros(2, 4),
    )
    assert trajectories.shape == (2, 8, 6, 3)
    assert scores.shape == (2, 8)
```

- [ ] **Step 2: Write literal loss tests first**

Hand-calculate collision, traversability, smoothness, kinematic, terminal
distance, and visibility terms on one- and two-segment trajectories. Assert
zero-safe cases, invalid map masks, finite gradients, and the exact weighted
sum without reusing production helpers for expected values.

- [ ] **Step 3: Implement dataset and ground policy**

Load versioned records without executing arbitrary serialized objects. Build a
small convolutional depth encoder, concatenate robot/target state, predict
residuals around fixed differential-drive arc primitives, and return candidate
paths plus learned costs. Adapt only relevant YOPO concepts and include the MIT
provenance statement.

- [ ] **Step 4: Implement guidance losses and policy training**

Sample trajectories against privileged elevation/traversability tensors;
compute the six named costs; train the predicted residuals and ranking score;
save checkpoints containing schema version, full model configuration, optimizer
state, epoch, normalization, and YOPO provenance. Require CLI `--manifest` and
`--output` plus explicit reproducibility arguments.

- [ ] **Step 5: Implement fixed-profile detector and policy export**

Detector export calls the external YOLO API with ONNX, end-to-end output,
`max_det`, fixed `imgsz`, and static batch. Policy export uses opset 17 and exact
names `depth`, `robot_state`, `target_state`, `trajectories`, `scores`. Inspect
ONNX metadata and delete an artifact whose names, dtypes, or dimensions differ
from the C++ contract.

- [ ] **Step 6: Review Python source and commit**

Inspect all shapes and names against Tasks 2 and 6, scan for unsafe pickle loads,
confirm lowercase paths, and run `git diff --check`. Do not invoke Python,
pytest, training, export, inference, or dependency installation.

```bash
git add autonomy/perception/shadow/python/shadow \
  autonomy/perception/shadow/python/test
git commit -m "feat(perception): add Shadow policy training"
```

### Task 12: Complete documentation and static acceptance review

**Files:**
- Create: `autonomy/perception/shadow/README.md`
- Review: every file listed in this plan.

**Interfaces:**
- Consumes: completed C++, Python, task, CMake, DAG, launch, and configuration contracts.
- Produces: operator-facing provenance, topic, model, training, and deployment documentation.

- [ ] **Step 1: Document provenance and licensing**

State the exact YOPO commit and MIT status, that YOPO itself does not provide
person detection, that YOPOv2-Tracker code was unavailable for direct reuse,
and that Ultralytics YOLO26 remains an external AGPL-3.0 or enterprise-licensed
dependency. Distinguish copied/adapted code from Shadow-authored code.

- [ ] **Step 2: Document runtime contracts and safety**

List all five topics and message types, four DAG reader positions, required TF
frames, both model tensor contracts, map layers, configuration rules, target
selection semantics, stale/lost behavior, empty-path invalidation, and the
prohibition on direct `/cmd_vel` publication.

- [ ] **Step 3: Document Python workflows without executing them**

Provide command examples for detector fine-tuning, policy training, detector
export, and policy export. Explicitly state that weights, datasets, and generated
artifacts are external and that commands were documented but not run during
this source-only adaptation.

- [ ] **Step 4: Run repository-safe static acceptance checks**

Run `git diff --check` over the exact Shadow/task/build/launch scope,
`clang-format --dry-run --Werror` over all authored C++ files, `rg` for stale
names and forbidden custom public messages, `find` to compare the actual
lowercase layout to the File Map, and `git diff --summary` to detect unintended
mode changes. Inspect `git status --short` before staging. Do not run CMake,
CTest, binaries, Python, pytest, training, export, inference, launch files, or
model downloads.

- [ ] **Step 5: Commit the documentation and final static corrections**

Stage only Shadow-related documentation and corrections identified by Step 4.
Preserve unrelated working-tree changes.

```bash
git add autonomy/perception/shadow/README.md
git commit -m "docs(perception): document Shadow deployment"
```

- [ ] **Step 6: Report verification limits accurately**

Report the exact static commands and exit codes. State explicitly that build,
tests, Python execution, training, export, inference, launch, and model download
were not performed, so no runtime-pass claim is made.
