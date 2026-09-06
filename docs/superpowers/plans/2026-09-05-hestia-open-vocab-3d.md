# Hestia Open-Vocabulary 3D Perception Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship an autolink `HestiaComponent` that detects home objects with open-vocabulary (and optional dual closed-set) 2D models, lifts them to metric 3D AABBs from RGB-D, tracks IDs, and publishes `Detection2DArray` / `Detection3DArray`.

**Architecture:** One Fathom-style component owns focused modules—`OpenDetector` / `HomeDetector`, `DepthLifter`, `ObjectTracker`, `DetectionMerger`, plus an optional non-blocking open async worker for `mode=dual`. C++ uses injectable `DetectorRunner` and existing `autonomy::common::network::Engine`; no in-tree weights.

**Tech Stack:** C++17, autolink, automsgs, Eigen, OpenCV (if letterbox already uses it in Shadow), GoogleTest, protobuf, ONNX Runtime / TensorRT via common-network; external YOLO-World / YOLOE / closed-set YOLO export.

**Spec:** `docs/superpowers/specs/2026-09-05-hestia-open-vocab-3d-design.md`

## Global Constraints

- Flat C++ tree under `autonomy/perception/hestia/`; lowercase dirs; `snake_case` filenames.
- Single config message `HestiaOptions` in `proto/hestia.proto`; sample `conf/hestia.pb.txt`.
- Public runtime I/O uses only existing automsgs (`Image`, `CameraInfo`, `Detection2DArray`, `Detection3DArray`, optional `String` query).
- Never publish `/cmd_vel`; do not own following or semantic maps.
- Use `autonomy::common::network::Engine` / `TensorMap`; do not add a second inference framework.
- Fixed export profiles; validate width/height/`max_detections` at `Init`.
- Do not commit weights, ONNX, TensorRT engines, datasets, or checkpoints.
- Document YOLO-World / YOLOE / Ultralytics license responsibility in README.
- Mirror Fathom build: `hestia_component` is a loadable DSO; entrypoint not in `libautonomy`.
- Do not modify or discard unrelated working-tree changes.
- Follow Shadow/Fathom headers (Apache-2.0 OpenRobotic Beginner / project copyright style already used in those modules).
- TDD: write the failing test source before production code for each module; run the named test target when the build environment allows.

## File Map

### New Hestia sources

- `autonomy/perception/hestia/proto/hestia.proto` — `HestiaOptions`
- `autonomy/perception/hestia/conf/hestia.pb.txt` — Orin-oriented sample
- `autonomy/perception/hestia/options.hpp|.cpp` — `ValidateHestiaOptions`
- `autonomy/perception/hestia/detector.hpp|.cpp` — `DetectorRunner`, `OpenDetector`, `HomeDetector`
- `autonomy/perception/hestia/lifter.hpp|.cpp` — `DepthLifter`
- `autonomy/perception/hestia/tracker.hpp|.cpp` — `ObjectTracker`
- `autonomy/perception/hestia/merger.hpp|.cpp` — `DetectionMerger`
- `autonomy/perception/hestia/open_worker.hpp|.cpp` — non-blocking open inference for dual mode
- `autonomy/perception/hestia/hestia_component.hpp|.cpp` — autolink component
- `autonomy/perception/hestia/CMakeLists.txt` — `hestia_component` DSO + component test
- `autonomy/perception/hestia/dag/hestia.dag`
- `autonomy/perception/hestia/launch/hestia.launch`
- `autonomy/perception/hestia/README.md`

### Tests

- `options_test.cpp`, `detector_test.cpp`, `lifter_test.cpp`, `tracker_test.cpp`, `merger_test.cpp`, `open_worker_test.cpp`, `hestia_component_test.cpp`

### Existing integration

- `CMakeLists.txt` — `add_subdirectory(autonomy/perception/hestia)` beside Fathom
- `cmake/autonomy_sources.cmake` — exclude component entrypoint / component test from libautonomy lists; optionally exclude concrete engine adapter when `NOT BUILD_ONNXRUNTIME`
- `cmake/autonomy_install.cmake` — install DSO, dag, conf, launch
- `autonomy/perception/README.md` — Hestia blurb already present; add pointer to module README if needed
- `autonomy/perception/launch/perception.launch` — optional wire after Fathom (only if file already composes modules)

### Detector tensor contract (v1 fixed profile)

Document and implement this closed contract (adjust only with README + options together):

- input `images`: float32 `[1, 3, H, W]`, RGB in `[0, 1]`, letterboxed
- output `output0`: float32 `[1, max_detections, 6]` rows `[x1,y1,x2,y2,score,class_index]`
- open mode: `class_index` indexes into the active prompt list (or exported class names); write string into `ObjectHypothesis.class_id`
- home mode: `class_index` indexes `home_labels[]`
- C++ does not run NMS if the export is end-to-end; if export requires NMS, document and implement class-agnostic NMS once in `detector.cpp` behind `bool needs_nms` only if the chosen export requires it—default assume end-to-end like Shadow YOLO26

---

### Task 1: Configuration contract

**Files:**
- Create: `autonomy/perception/hestia/proto/hestia.proto`
- Create: `autonomy/perception/hestia/conf/hestia.pb.txt`
- Create: `autonomy/perception/hestia/options.hpp`
- Create: `autonomy/perception/hestia/options.cpp`
- Create: `autonomy/perception/hestia/options_test.cpp`

**Interfaces:**
- Produces: `bool ValidateHestiaOptions(const proto::HestiaOptions&, std::string* error = nullptr)`
- Produces: `autonomy::perception::hestia::proto::HestiaOptions`
- Consumes: nothing else in Hestia

- [ ] **Step 1: Write failing options tests**

```cpp
proto::HestiaOptions ValidOptions() {
  proto::HestiaOptions o;
  o.set_mode("open");
  o.set_open_model_path("/models/hestia_open.onnx");
  o.set_backend("onnx");
  o.set_open_width(640);
  o.set_open_height(640);
  o.set_max_detections(100);
  o.set_confidence_threshold(0.25F);
  o.add_open_prompts("chair");
  o.add_open_prompts("cup");
  o.set_depth_scale(0.001F);
  o.set_min_depth_m(0.2F);
  o.set_max_depth_m(8.0F);
  o.set_min_depth_samples(12);
  o.set_inner_box_scale(0.5F);
  o.set_depth_outlier_m(0.25F);
  o.set_camera_frame("camera_optical");
  o.set_base_frame("base_link");
  o.set_association_iou_threshold(0.3F);
  o.set_lost_timeout_sec(1.5F);
  o.set_merge_iou_threshold(0.5F);
  o.set_detections_2d_topic("/perception/hestia/detections_2d");
  o.set_detections_3d_topic("/perception/hestia/detections_3d");
  o.set_max_input_skew_sec(0.05F);
  o.set_max_data_age_sec(0.2F);
  o.set_use_fathom_depth(false);
  return o;
}

TEST(HestiaOptionsTest, AcceptsValidOpenMode) {
  std::string error;
  EXPECT_TRUE(ValidateHestiaOptions(ValidOptions(), &error));
  EXPECT_TRUE(error.empty());
}

TEST(HestiaOptionsTest, RejectsUnknownMode) {
  auto o = ValidOptions();
  o.set_mode("all");
  std::string error;
  EXPECT_FALSE(ValidateHestiaOptions(o, &error));
}

TEST(HestiaOptionsTest, DualRequiresHomeModelAndLabels) {
  auto o = ValidOptions();
  o.set_mode("dual");
  o.set_home_model_path("/models/hestia_home.onnx");
  o.set_home_width(640);
  o.set_home_height(640);
  o.add_home_labels("person");
  o.add_home_labels("chair");
  std::string error;
  EXPECT_TRUE(ValidateHestiaOptions(o, &error));
  o.clear_home_labels();
  EXPECT_FALSE(ValidateHestiaOptions(o, &error));
}
```

Also reject: empty open path in `open` mode; unknown backend; zero dims; `min_depth_m >= max_depth_m`; empty topics; identical 2d/3d topics; `inner_box_scale` outside `(0,1]`.

- [ ] **Step 2: Run options test (expect link/compile fail or missing symbol)**

Run: build target matching `*hestia*options*` or add temporary listing once CMake exists in Task 7. Until Task 7, keep the test file present and proceed—Task 7 wires `google_test` for module tests via `libautonomy` glob (`*_test.cpp`).

- [ ] **Step 3: Define `hestia.proto` field numbers**

```protobuf
syntax = "proto3";
package autonomy.perception.hestia.proto;

message HestiaOptions {
  string mode = 1;  // "open" | "dual"
  string open_model_path = 2;
  string home_model_path = 3;
  string backend = 4;  // "onnx" | "tensorrt"
  uint32 open_width = 5;
  uint32 open_height = 6;
  uint32 home_width = 7;
  uint32 home_height = 8;
  uint32 max_detections = 9;
  float confidence_threshold = 10;
  repeated string open_prompts = 11;
  repeated string home_labels = 12;
  float depth_scale = 13;
  float min_depth_m = 14;
  float max_depth_m = 15;
  uint32 min_depth_samples = 16;
  float inner_box_scale = 17;
  float depth_outlier_m = 18;
  string camera_frame = 19;
  string base_frame = 20;
  float association_iou_threshold = 21;
  float lost_timeout_sec = 22;
  float merge_iou_threshold = 23;
  string detections_2d_topic = 24;
  string detections_3d_topic = 25;
  string query_topic = 26;
  float max_input_skew_sec = 27;
  float max_data_age_sec = 28;
  bool use_fathom_depth = 29;
  uint32 open_async_queue = 30;  // 0 = inline open even in dual (tests); 1+ enables worker
}
```

- [ ] **Step 4: Implement `ValidateHestiaOptions`**

Mirror Shadow `SetError` prefix `"Hestia: "`. Require `open_prompts` non-empty in `open` mode. In `dual`, require `home_model_path`, positive home dims, and non-empty `home_labels`. Backend must be `onnx` or `tensorrt`.

- [ ] **Step 5: Write `conf/hestia.pb.txt` sample** matching `ValidOptions()` defaults (Orin 640 open).

- [ ] **Step 6: Commit** (only when the user has asked for commits)

```bash
git add autonomy/perception/hestia/proto/hestia.proto \
  autonomy/perception/hestia/conf/hestia.pb.txt \
  autonomy/perception/hestia/options.hpp \
  autonomy/perception/hestia/options.cpp \
  autonomy/perception/hestia/options_test.cpp
git commit -m "$(cat <<'EOF'
feat(hestia): add HestiaOptions protobuf and validation

EOF
)"
```

---

### Task 2: Detectors with injectable runner

**Files:**
- Create: `autonomy/perception/hestia/detector.hpp`
- Create: `autonomy/perception/hestia/detector.cpp`
- Create: `autonomy/perception/hestia/detector_test.cpp`

**Interfaces:**
- Consumes: `proto::HestiaOptions`
- Produces:
  - `class DetectorRunner { virtual bool Run(const TensorMap&, TensorMap*, string*) = 0; }`
  - `class OpenDetector` / `class HomeDetector` with
    `static unique_ptr<...> Create(const HestiaOptions&, string* error)`
    `static unique_ptr<...> Create(const HestiaOptions&, unique_ptr<DetectorRunner>, string* error)`
    `bool Detect(const Image& rgb, Detection2DArray* out, string* error)`
  - OpenDetector also: `void SetPrompts(const vector<string>& prompts)` for query overrides (label map only; v1 does not re-export embeddings mid-flight—document that prompt list must match export, query filters/subset by name when possible)

**Label mapping rule (v1):** exported model is frozen to the prompt/class list used at export. Runtime `open_prompts` / query must be the same ordered list as export, or a **prefix/subset by index** only if the artifact documents it. Prefer documenting “re-export when changing prompts”; `SetPrompts` updates the string table used to fill `class_id` from `class_index`.

- [ ] **Step 1: Write detector tests with `FakeRunner`**

Copy Shadow `detector_test.cpp` FakeRunner pattern. Feed one row `[100,80,200,180,0.9,0]` and expect:
- one `Detection2D`
- `results(0).hypothesis().class_id() == "chair"` when prompts[0]=="chair"
- `results(0).hypothesis().score()` near 0.9
- box restored from letterbox (assert center roughly in original image coords for a known 640→letterbox fixture)

Add failure path: runner returns false → `Detect` false, output cleared.

- [ ] **Step 2: Implement letterbox + parse** in `detector.cpp`

Reuse Shadow letterbox math where possible (do not create a shared dependency unless trivial copy is worse—prefer a private helper in `detector.cpp` to avoid cross-module coupling in v1).

- [ ] **Step 3: Implement `Create` with real Engine** behind `#`/runtime when ORT exists; tests always use injected runner.

- [ ] **Step 4: Run `detector_test`**

Expected: PASS without weights.

- [ ] **Step 5: Commit** (if user requested)

```bash
git add autonomy/perception/hestia/detector.hpp \
  autonomy/perception/hestia/detector.cpp \
  autonomy/perception/hestia/detector_test.cpp
git commit -m "$(cat <<'EOF'
feat(hestia): add open/home detectors with injectable runner

EOF
)"
```

---

### Task 3: Depth lifter → 3D AABB

**Files:**
- Create: `autonomy/perception/hestia/lifter.hpp`
- Create: `autonomy/perception/hestia/lifter.cpp`
- Create: `autonomy/perception/hestia/lifter_test.cpp`

**Interfaces:**
- Consumes: `HestiaOptions`, `Detection2D`, depth `Image`, `CameraInfo`, optional `TransformStamped camera_to_base`
- Produces:
  ```cpp
  class DepthLifter {
   public:
    explicit DepthLifter(const proto::HestiaOptions& options);
    // Fills detections_3d; skips boxes that fail the depth gate.
    bool Lift(const Detection2DArray& detections_2d,
              const Image& depth,
              const CameraInfo& camera,
              const TransformStamped* camera_to_base,  // nullable
              Detection3DArray* detections_3d,
              string* error) const;
  };
  ```

- [ ] **Step 1: Write lifter tests** (adapt Shadow `localizer_test` synthetic depth helpers)

Cases:
1. Constant depth plane `Z=2` under a centered box → center z≈2, size finite and positive.
2. Too few valid samples → that detection omitted from 3D array; function still returns true.
3. `16UC1` with `depth_scale=0.001` matches metric `32FC1`.
4. With identity `camera_to_base`, `header.frame_id == base_frame`.
5. `camera_to_base == nullptr` → `frame_id == camera_frame`.

- [ ] **Step 2: Implement median / percentile lift** per spec (inner box, outlier reject, AABB identity orientation). Copy inliers into each `Detection3D.results` from the 2D hypothesis; copy `id` when already set.

- [ ] **Step 3: Run `lifter_test`** — expect PASS.

- [ ] **Step 4: Commit** (if user requested)

---

### Task 4: Multi-class object tracker

**Files:**
- Create: `autonomy/perception/hestia/tracker.hpp`
- Create: `autonomy/perception/hestia/tracker.cpp`
- Create: `autonomy/perception/hestia/tracker_test.cpp`

**Interfaces:**
```cpp
class ObjectTracker {
 public:
  explicit ObjectTracker(const proto::HestiaOptions& options);
  // Assigns Detection2D.id track strings; drops expired tracks.
  void Update(double stamp_sec, Detection2DArray* detections);
  void Clear();
};
```

- [ ] **Step 1: Write tracker tests**

1. Same box two frames → same `id`.
2. Far second box → new `id`.
3. After `lost_timeout_sec` without matches, next sighting gets a new id.
4. Empty input clears active matches but does not crash.

Association: greedy IoU ≥ `association_iou_threshold`; prefer same `class_id` when IoU ties.

- [ ] **Step 2: Implement ByteTrack-lite** (single high-threshold pass is enough for v1; optional low-threshold second pass if tests need it).

- [ ] **Step 3: Run `tracker_test`** — PASS.

- [ ] **Step 4: Commit** (if user requested)

---

### Task 5: Dual-mode merger + open worker

**Files:**
- Create: `autonomy/perception/hestia/merger.hpp|.cpp`
- Create: `autonomy/perception/hestia/merger_test.cpp`
- Create: `autonomy/perception/hestia/open_worker.hpp|.cpp`
- Create: `autonomy/perception/hestia/open_worker_test.cpp`

**Interfaces:**
```cpp
class DetectionMerger {
 public:
  explicit DetectionMerger(const proto::HestiaOptions& options);
  // Prefer higher score when IoU >= merge_iou_threshold.
  void Merge(const Detection2DArray& home,
             const Detection2DArray& open,
             Detection2DArray* out) const;
};

class OpenAsyncWorker {
 public:
  OpenAsyncWorker(unique_ptr<OpenDetector> detector, uint32_t queue_depth);
  ~OpenAsyncWorker();  // joins thread
  // Non-blocking: returns false if busy/full; does not wait on inference.
  bool TrySubmit(const Image& rgb);
  // Copies last completed detections if any; false if never completed.
  bool TryGetLatest(Detection2DArray* out) const;
  void Shutdown();
};
```

- [ ] **Step 1: Merger tests** — overlapping boxes keep higher score; non-overlap concatenates.

- [ ] **Step 2: Implement merger**

- [ ] **Step 3: Worker tests** — inject slow Fake OpenDetector via a test double wrapper: submit twice while first runs → second `TrySubmit` false; after completion `TryGetLatest` true. Use a controllable fake that blocks on a promise if needed; keep the test deterministic.

- [ ] **Step 4: Implement worker** with one consumer thread, mutex, `latest_` buffer, `busy_` flag. `queue_depth==0` is unused (component runs open inline).

- [ ] **Step 5: Run merger + worker tests** — PASS.

- [ ] **Step 6: Commit** (if user requested)

---

### Task 6: `HestiaComponent` lifecycle

**Files:**
- Create: `autonomy/perception/hestia/hestia_component.hpp`
- Create: `autonomy/perception/hestia/hestia_component.cpp`
- Create: `autonomy/perception/hestia/hestia_component_test.cpp`

**Interfaces:**
```cpp
class HestiaComponent final
  : public autolink::Component<Image, Image, CameraInfo> {
 public:
  bool Init() override;
  bool Proc(const shared_ptr<Image>& rgb,
            const shared_ptr<Image>& depth,
            const shared_ptr<CameraInfo>& info) override;
 protected:
  void Clear() override;
 private:
  friend class HestiaComponentTestApi;
  using DetectFn = function<bool(const Image&, Detection2DArray*, string*)>;
  using LiftFn = function<bool(const Detection2DArray&, const Image&,
                               const CameraInfo&, Detection3DArray*, string*)>;
  using Publish2d = function<bool(const Detection2DArray&)>;
  using Publish3d = function<bool(const Detection3DArray&)>;
  // members: options, detectors, lifter, tracker, merger, worker, writers, seams
};
```

Pipeline in `Proc` (after skew/encoding checks):

1. `mode=open`: `OpenDetector::Detect` → tracker → lift → publish 2d+3d  
2. `mode=dual`: `HomeDetector::Detect` every frame; `OpenAsyncWorker::TrySubmit(rgb)` if `open_async_queue>=1`, else inline open; `TryGetLatest` (or inline result) → `Merger` → tracker → lift → publish  
3. On inference failure: return false, publish nothing  
4. Zero detections: publish empty arrays, return true if writers succeed  

Optional query: if `query_topic` non-empty, create async `Reader<String>` in `Init` that calls `OpenDetector::SetPrompts` / worker prompt update (document frozen-export constraint).

- [ ] **Step 1: Write component tests** using `HestiaComponentTestApi` like Fathom—inject detect/lift/publish seams; assert Init without weights; Proc publishes both topics; detect failure yields false and zero publishes; dual without latest open still publishes home.

- [ ] **Step 2: Implement component** loading options from autolink config path (copy Fathom `Init` pattern for `GetProtoConfig` / equivalent used by FathomComponent).

- [ ] **Step 3: Run `hestia_component_test`** — PASS (requires Task 7 CMake DSO link).

- [ ] **Step 4: Commit** (if user requested)

---

### Task 7: Build, DAG, launch, install

**Files:**
- Create: `autonomy/perception/hestia/CMakeLists.txt`
- Create: `autonomy/perception/hestia/dag/hestia.dag`
- Create: `autonomy/perception/hestia/launch/hestia.launch`
- Modify: `CMakeLists.txt` (add_subdirectory)
- Modify: `cmake/autonomy_sources.cmake`
- Modify: `cmake/autonomy_install.cmake`
- Modify: `autonomy/perception/launch/perception.launch` only if it already lists Fathom/Shadow modules

**CMakeLists.txt pattern (Fathom):**

```cmake
if(NOT BUILD_ONNXRUNTIME OR NOT OnnxRuntime_FOUND OR NOT TARGET autolink)
  return()
endif()

add_library(hestia_component SHARED hestia_component.cpp)
target_include_directories(hestia_component PRIVATE
  ${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR})
target_link_libraries(hestia_component PRIVATE autonomy autolink automsgs)
set_target_properties(hestia_component PROPERTIES OUTPUT_NAME hestia_component)

if(BUILD_TEST)
  set(_t autonomy.perception.hestia.component_test)
  google_test(${_t} hestia_component_test.cpp)
  add_dependencies(${_t} hestia_component)
  target_link_libraries(${_t} PRIVATE hestia_component)
endif()
```

**autonomy_sources.cmake:** add `HESTIA_COMPONENT_SRCS` / `HESTIA_COMPONENT_TEST_SRCS` and `list(REMOVE_ITEM ...)` like Fathom. If `detector.cpp` hard-requires Engine symbols when ORT is off, either keep Create(injected) path linkable without Engine, or exclude a thin `detector_engine.cpp` when `NOT BUILD_ONNXRUNTIME`—prefer keeping injected path always in libautonomy.

**hestia.dag:**

```text
module_config {
  module_library: "libhestia_component.so"
  components {
    class_name: "autonomy::perception::hestia::HestiaComponent"
    config {
      name: "hestia"
      config_file_path: "conf/hestia.pb.txt"
      readers { channel: "/camera/rgb/image_raw" }
      readers { channel: "/camera/depth/image_raw" }  # or Fathom refined topic
      readers { channel: "/camera/camera_info" }
    }
  }
}
```

- [ ] **Step 1: Wire CMake + install**
- [ ] **Step 2: Configure build and run**

```bash
# Adjust to the project's usual out-of-tree build dir
ctest -R 'hestia|Hestia' --output-on-failure
```

Expected: all Hestia unit tests PASS without weight files.

- [ ] **Step 3: Commit** (if user requested)

---

### Task 8: README and profile docs

**Files:**
- Create: `autonomy/perception/hestia/README.md`
- Modify: `autonomy/perception/README.md` if a deep-link to the module README helps

**README must include:**
- Purpose and modes `open` / `dual`
- Topic table from the spec
- Tensor / export contract
- Orin vs RK3588 profile table and expected rates (as targets, not guarantees)
- How to point depth at Fathom refined topic (`use_fathom_depth` + DAG channel)
- License disclaimer for YOLO-World / YOLOE / Ultralytics
- “No weights in repo” + example `open_model_path`

- [ ] **Step 1: Write README**
- [ ] **Step 2: Commit** (if user requested)

---

## Spec coverage checklist

| Spec item | Task |
| --- | --- |
| `HestiaOptions` + validation | 1 |
| Open + home detectors, runner seam | 2 |
| Depth lift AABB + gate | 3 |
| Multi-object tracks | 4 |
| Dual merge + non-blocking open | 5 |
| Component Init/Proc/Clear + publish | 6 |
| DSO / DAG / launch / install | 7 |
| Profiles + license README | 8 |
| No end-to-end 3D / no SAM / no map | explicit non-goals — no task |
| Fathom depth optional | 6 DAG + 8 docs |
| Shadow boundary | 8 docs |

## Placeholder / consistency review

- No TBD steps; tensor contract fixed in File Map.
- Prompt/export freeze documented in Task 2 (avoids fake “runtime CLIP prompts”).
- `open_async_queue` field ties dual async to options.
- Commits are gated on user request per repo rules.
