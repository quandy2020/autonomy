# Autonomy CMake Phase A Minimal Profile Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make enabled Autonomy domains determine dependency discovery and provide a reproducible `autonomy-minimal` configure, build, and CTest workflow without disturbing the current uncommitted modularization.

**Architecture:** Introduce a pure CMake module-selection layer, derive required package groups from the enabled domains, and keep package-specific `find_package()` calls in one dependency module. The root project consumes the resulting enabled-module list, while a CMake Preset and repository-owned verification script provide the same out-of-source workflow for developers and later CI.

**Tech Stack:** CMake 3.20, CMake Presets schema v2, Ninja, C++17, CTest, Python 3 `unittest`, Bash

**Spec:** `docs/superpowers/specs/2026-09-13-cmake-modularization-design.md`

## Global Constraints

- Work in the current checkout because the uncommitted modular architecture does not exist at `HEAD`.
- Record `git rev-parse HEAD`, `git status --short`, `git diff`, and `git diff --cached` before editing.
- Treat every pre-existing tracked, staged, unstaged, submodule, and untracked change as user-owned baseline state.
- Do not reset, restore, delete, stage, commit, or format pre-existing changes.
- Do not create implementation commits in this phase; report the new patch relative to the recorded working-tree baseline.
- Modify only the files named by the active task. If their live content no longer matches this plan, stop that task and preserve the other work.
- Do not change algorithms, runtime interfaces, directory layout, Autolink transport, message schemas, or robot behavior.
- CMake 3.20 remains the minimum version; therefore the root preset uses schema version 2.
- `autonomy` remains a compatibility umbrella target.
- This plan implements Phase A only. Public/private dependency visibility, executable linking, test ownership migration, explicit source lists, install-consumer validation, sanitizers, and CI belong to later plans.
- Evidence must be labeled `source-confirmed`, `configured`, `built`, `tested`, or `unverified`; one level never implies another.

---

## File Map

- `cmake/autonomy_options.cmake`: declares domain options, computes enabled modules, and validates the dependency graph without finding external packages.
- `cmake/autonomy_find_dependencies.cmake`: maps enabled modules and companion options to package groups, then performs package discovery.
- `cmake/tests/test_autonomy_options.py`: configures small temporary projects to test module selection and invalid graphs.
- `cmake/tests/test_autonomy_dependencies.py`: tests the pure required-package calculation without requiring third-party libraries.
- `CMakeLists.txt`: invokes module selection and dependency discovery before adding enabled subdirectories.
- `cmake/autonomy_build.cmake`: adds only `AUTONOMY_ENABLED_MODULES`.
- `cmake/autonomy_tests.cmake`: discovers tests only in enabled modules during this transitional phase.
- `CMakePresets.json`: defines `autonomy-minimal` configure, build, and test presets using schema version 2.
- `scripts/cmake/test_presets.py`: verifies the preset contract and option values.
- `scripts/cmake/verify_minimal.sh`: runs configure, build, selected CTest, and source-cleanliness comparison.
- `docs/quality/cmake-build-profiles.md`: records commands, supported scope, and actual evidence.

---

### Task 1: Domain option and dependency-graph contract

**Files:**
- Create: `cmake/autonomy_options.cmake`
- Create: `cmake/tests/test_autonomy_options.py`

**Interfaces:**
- Produces: `autonomy_declare_module_options()`.
- Produces: `autonomy_compute_enabled_modules(<out-variable>)`.
- Produces: `autonomy_validate_module_graph()`.
- Produces cache options `AUTONOMY_BUILD_<UPPERCASE_DOMAIN>` for every entry in `AUTONOMY_MODULE_ORDER`.
- Default: every current product domain is `ON`; presets override individual domains.

- [ ] **Step 1: Capture the working-tree baseline outside Git**

Run:

```bash
mkdir -p /tmp/autonomy-cmake-phase-a-baseline
git rev-parse HEAD > /tmp/autonomy-cmake-phase-a-baseline/head.txt
git status --short > /tmp/autonomy-cmake-phase-a-baseline/status.txt
git diff > /tmp/autonomy-cmake-phase-a-baseline/unstaged.patch
git diff --cached > /tmp/autonomy-cmake-phase-a-baseline/staged.patch
```

Expected: all commands exit 0. These files are evidence only and must not be added to Git.

- [ ] **Step 2: Write failing module-selection tests**

The Python test creates a temporary `CMakeLists.txt`, appends the repository
`cmake/` directory to `CMAKE_MODULE_PATH`, sets the canonical order, includes
`autonomy_options`, and writes the computed list to a file.

Required test methods are
`test_defaults_enable_every_domain`,
`test_minimal_enables_common_transform_vehicle`,
`test_transform_without_common_is_rejected`,
`test_control_without_map_is_rejected`, and
`test_bridge_without_system_is_rejected`. Each test must assert the exact
semicolon-separated enabled-module result or the exact required-dependency
diagnostic, rather than only checking the process exit code.

The minimal configure passes:

```text
-DAUTONOMY_BUILD_MAP=OFF
-DAUTONOMY_BUILD_PREDICTION=OFF
-DAUTONOMY_BUILD_CONTROL=OFF
-DAUTONOMY_BUILD_PLANNING=OFF
-DAUTONOMY_BUILD_PERCEPTION=OFF
-DAUTONOMY_BUILD_LOCALIZATION=OFF
-DAUTONOMY_BUILD_SENSOR=OFF
-DAUTONOMY_BUILD_TASK=OFF
-DAUTONOMY_BUILD_SYSTEM=OFF
-DAUTONOMY_BUILD_AUDIO=OFF
-DAUTONOMY_BUILD_BRIDGE=OFF
-DAUTONOMY_BUILD_VISUALIZATION=OFF
```

- [ ] **Step 3: Verify RED**

Run: `python3 -m unittest cmake.tests.test_autonomy_options -v`

Expected: every case fails because `autonomy_options.cmake` or its functions do not exist.

- [ ] **Step 4: Implement the module options**

Use this canonical dependency graph:

```cmake
set(AUTONOMY_MODULE_DEPENDENCIES_common "")
set(AUTONOMY_MODULE_DEPENDENCIES_transform common)
set(AUTONOMY_MODULE_DEPENDENCIES_map common transform)
set(AUTONOMY_MODULE_DEPENDENCIES_vehicle common)
set(AUTONOMY_MODULE_DEPENDENCIES_prediction common)
set(AUTONOMY_MODULE_DEPENDENCIES_control common transform map)
set(AUTONOMY_MODULE_DEPENDENCIES_planning common transform map)
set(AUTONOMY_MODULE_DEPENDENCIES_perception common transform map)
set(AUTONOMY_MODULE_DEPENDENCIES_localization common transform)
set(AUTONOMY_MODULE_DEPENDENCIES_sensor common control)
set(AUTONOMY_MODULE_DEPENDENCIES_task common transform map control)
set(AUTONOMY_MODULE_DEPENDENCIES_system common task)
set(AUTONOMY_MODULE_DEPENDENCIES_audio common)
set(AUTONOMY_MODULE_DEPENDENCIES_bridge common system task)
set(AUTONOMY_MODULE_DEPENDENCIES_visualization map)
```

`autonomy_validate_module_graph()` must fail with this exact message form:

```text
AUTONOMY_BUILD_<MODULE>=ON requires AUTONOMY_BUILD_<DEPENDENCY>=ON
```

Do not auto-enable dependencies. Explicit failure keeps preset behavior reviewable.

- [ ] **Step 5: Verify GREEN**

Run:

```bash
python3 -m unittest cmake.tests.test_autonomy_options -v
git diff --check -- cmake/autonomy_options.cmake cmake/tests/test_autonomy_options.py
```

Expected: five tests pass and the whitespace check is clean.

---

### Task 2: Enabled-domain package calculation

**Files:**
- Create: `cmake/autonomy_find_dependencies.cmake`
- Create: `cmake/tests/test_autonomy_dependencies.py`

**Interfaces:**
- Consumes: `AUTONOMY_ENABLED_MODULES` from Task 1.
- Produces: `autonomy_collect_required_package_groups(<out-variable>)`.
- Produces: `autonomy_find_dependencies()`.
- Package groups are stable semantic identifiers, not raw library filenames.

- [ ] **Step 1: Write failing package-group tests**

Create temporary script-mode CMake inputs and assert exact sorted group lists:

```text
common -> core;common_math;common_vision
common;transform;vehicle -> core;common_math;common_vision
common;transform;map -> core;common_math;common_vision;map
common;transform;localization -> core;common_math;common_vision;localization
common;transform;map;control;task -> core;common_math;common_vision;map;control;task
```

Also assert companion groups:

```text
BUILD_GRPC + bridge -> grpc
BUILD_AUTOVIZ -> autoviz
BUILD_PROMETHEUS + system -> prometheus
BUILD_SHERPA_ONNX + audio -> sherpa_onnx
```

- [ ] **Step 2: Verify RED**

Run: `python3 -m unittest cmake.tests.test_autonomy_dependencies -v`

Expected: failure because `autonomy_find_dependencies.cmake` is absent.

- [ ] **Step 3: Implement pure group calculation**

The mapping is:

```cmake
core: Protobuf, Eigen3, nlohmann_json, glog, yaml-cpp, Threads
common_math: Ceres, OSQP
common_vision: OpenCV
map: PCL components common/features/filters/io/kdtree/segmentation/surface, TBB, OpenMP optional
control: Ipopt optional
task: behaviortree_cpp
localization: LuaGoogle, FBow, G2o, SQLite3, Boost iostreams, PkgConfig CAIRO
grpc: gRPC
autoviz: Qt6 Core/Gui/Widgets/OpenGLWidgets/OpenGL/Xml/Svg/Network
prometheus: prometheus-cpp
sherpa_onnx: SherpaOnnx optional
```

Inference remains controlled by `BUILD_ONNXRUNTIME` and `BUILD_TENSORRT`; append
the `inference` group only when either is enabled and `common` or `perception`
is enabled.

- [ ] **Step 4: Implement package discovery from groups**

`autonomy_find_dependencies()` loops over the calculated groups and executes
each package block once. Retain the existing package names, imported targets,
components, `QUIET` semantics, and status messages from the root file. Keep
`EnsureProtobuf319` in the `core` block. Unknown groups are fatal.

Autolink and Automsgs are embedded projects, not entries in this package-group
calculation.

- [ ] **Step 5: Verify GREEN**

Run:

```bash
python3 -m unittest cmake.tests.test_autonomy_dependencies -v
git diff --check -- cmake/autonomy_find_dependencies.cmake cmake/tests/test_autonomy_dependencies.py
```

Expected: all exact-list cases pass.

---

### Task 3: Root integration and enabled-module traversal

**Files:**
- Modify: `CMakeLists.txt:32-155`
- Modify: `cmake/autonomy_build.cmake:20-39`
- Modify: `cmake/autonomy_tests.cmake:5-57`
- Modify: `cmake/tests/test_autonomy_options.py`

**Interfaces:**
- Consumes: all Task 1 and Task 2 functions.
- Produces root variable `AUTONOMY_ENABLED_MODULES` before test discovery and subdirectory traversal.
- `autonomy_add_modules()` and `autonomy_configure_tests()` consume `AUTONOMY_ENABLED_MODULES`, not the complete module order.

- [ ] **Step 1: Add failing root-contract tests**

Extend the Python fixture to inspect the root files and configure a fake project
that substitutes `autonomy_find_dependencies()` with a recording function.
Assert that:

- selection is computed before dependency discovery;
- validation occurs before dependency discovery;
- test discovery receives only enabled modules;
- module traversal receives only enabled modules;
- `bridge` still requires both `AUTONOMY_BUILD_BRIDGE=ON` and `BUILD_GRPC=ON`;
- `visualization` still requires both `AUTONOMY_BUILD_VISUALIZATION=ON` and
  `foxglove-sdk_FOUND`.

- [ ] **Step 2: Verify RED**

Run: `python3 -m unittest cmake.tests.test_autonomy_options -v`

Expected: the new root-contract assertions fail against the current unconditional flow.

- [ ] **Step 3: Integrate option and dependency modules**

In the root file:

```cmake
set(AUTONOMY_MODULE_ORDER
  common transform map vehicle prediction control planning perception
  localization sensor task system audio bridge visualization)
include(autonomy_options)
autonomy_declare_module_options()
autonomy_validate_module_graph()
autonomy_compute_enabled_modules(AUTONOMY_ENABLED_MODULES)

include(autonomy_find_dependencies)
autonomy_find_dependencies()
```

Remove the corresponding unconditional root `find_package()` blocks after their
logic is represented in `autonomy_find_dependencies.cmake`. Keep documentation,
Autolink, Automsgs, AutoDriver, AutoViz, and AutoSim orchestration in the root.

- [ ] **Step 4: Traverse enabled modules only**

Change `autonomy_add_modules()` and `autonomy_configure_tests()` loops to use
`AUTONOMY_ENABLED_MODULES`. Preserve feature conditions for bridge and
visualization as a second gate. Emit one status line for every explicitly
disabled module:

```text
autonomy: module '<name>' disabled by AUTONOMY_BUILD_<NAME>=OFF
```

- [ ] **Step 5: Verify root behavior**

Run:

```bash
python3 -m unittest cmake.tests.test_autonomy_options -v
python3 -m unittest cmake.tests.test_autonomy_dependencies -v
cmake -S . -B /tmp/autonomy-phase-a-configure -G Ninja \
  -DBUILD_DOCS=OFF -DBUILD_GRPC=OFF -DBUILD_PROMETHEUS=OFF \
  -DBUILD_AUTODRIVER=OFF -DBUILD_AUTOVIZ=OFF -DBUILD_AUTOSIM=OFF \
  -DBUILD_ONNXRUNTIME=OFF -DBUILD_TENSORRT=OFF \
  -DBUILD_SHERPA_ONNX=OFF -DBUILD_TOOLS=OFF \
  -DAUTONOMY_BUILD_MAP=OFF -DAUTONOMY_BUILD_PREDICTION=OFF \
  -DAUTONOMY_BUILD_CONTROL=OFF -DAUTONOMY_BUILD_PLANNING=OFF \
  -DAUTONOMY_BUILD_PERCEPTION=OFF -DAUTONOMY_BUILD_LOCALIZATION=OFF \
  -DAUTONOMY_BUILD_SENSOR=OFF -DAUTONOMY_BUILD_TASK=OFF \
  -DAUTONOMY_BUILD_SYSTEM=OFF -DAUTONOMY_BUILD_AUDIO=OFF \
  -DAUTONOMY_BUILD_BRIDGE=OFF -DAUTONOMY_BUILD_VISUALIZATION=OFF
```

Expected: configuration does not call `find_package(LuaGoogle)`, `FBow`, `G2o`,
`PCL`, `behaviortree_cpp`, `gRPC`, `Qt6`, or `prometheus-cpp`. A failure in one
of the explicitly retained common groups is evidence for Phase B, not permission
to re-enable a disabled-domain package.

- [ ] **Step 6: Check only the task patch**

Run:

```bash
git diff --check -- CMakeLists.txt cmake/autonomy_build.cmake cmake/autonomy_tests.cmake cmake/autonomy_options.cmake cmake/autonomy_find_dependencies.cmake cmake/tests
git status --short > /tmp/autonomy-cmake-phase-a-baseline/status-after-task3.txt
```

Expected: no whitespace errors; no unrelated path has changed because of configuration.

---

### Task 4: CMake Preset contract

**Files:**
- Create: `CMakePresets.json`
- Create: `scripts/cmake/test_presets.py`

**Interfaces:**
- Produces configure preset `autonomy-minimal`.
- Produces build preset `autonomy-minimal`.
- Produces test preset `autonomy-minimal`.
- Binary directory: `${sourceDir}/build/autonomy-minimal`.

- [ ] **Step 1: Write failing JSON contract tests**

The tests load `CMakePresets.json` with Python `json` and assert:

- `version` is exactly `2`;
- configure generator is `Ninja`;
- binary directory is `${sourceDir}/build/autonomy-minimal`;
- `CMAKE_BUILD_TYPE=Debug` and `BUILD_TEST=ON`;
- all non-minimal companions and domains listed in the design are `OFF`;
- `AUTONOMY_BUILD_COMMON`, `AUTONOMY_BUILD_TRANSFORM`, and
  `AUTONOMY_BUILD_VEHICLE` are `ON`;
- build/test preset configurePreset is `autonomy-minimal`;
- test output-on-failure is enabled and no-tests is an error.

- [ ] **Step 2: Verify RED**

Run: `python3 -m unittest scripts.cmake.test_presets -v`

Expected: failure because the root preset does not exist.

- [ ] **Step 3: Add schema-v2 presets**

Use exactly these cache values:

```json
{
  "CMAKE_BUILD_TYPE": "Debug",
  "BUILD_TEST": "ON",
  "BUILD_DOCS": "OFF",
  "BUILD_TOOLS": "OFF",
  "BUILD_GRPC": "OFF",
  "BUILD_PROMETHEUS": "OFF",
  "BUILD_GRID_MAP_DEMOS": "OFF",
  "BUILD_AUTODRIVER": "OFF",
  "BUILD_AUTOVIZ": "OFF",
  "BUILD_AUTOSIM": "OFF",
  "BUILD_ONNXRUNTIME": "OFF",
  "BUILD_TENSORRT": "OFF",
  "BUILD_SHERPA_ONNX": "OFF",
  "AUTONOMY_BUILD_COMMON": "ON",
  "AUTONOMY_BUILD_TRANSFORM": "ON",
  "AUTONOMY_BUILD_VEHICLE": "ON",
  "AUTONOMY_BUILD_MAP": "OFF",
  "AUTONOMY_BUILD_PREDICTION": "OFF",
  "AUTONOMY_BUILD_CONTROL": "OFF",
  "AUTONOMY_BUILD_PLANNING": "OFF",
  "AUTONOMY_BUILD_PERCEPTION": "OFF",
  "AUTONOMY_BUILD_LOCALIZATION": "OFF",
  "AUTONOMY_BUILD_SENSOR": "OFF",
  "AUTONOMY_BUILD_TASK": "OFF",
  "AUTONOMY_BUILD_SYSTEM": "OFF",
  "AUTONOMY_BUILD_AUDIO": "OFF",
  "AUTONOMY_BUILD_BRIDGE": "OFF",
  "AUTONOMY_BUILD_VISUALIZATION": "OFF"
}
```

- [ ] **Step 4: Verify GREEN and CMake parsing**

Run:

```bash
python3 -m unittest scripts.cmake.test_presets -v
cmake --list-presets
```

Expected: tests pass and CMake lists `autonomy-minimal`.

---

### Task 5: Minimal workflow and evidence

**Files:**
- Create: `scripts/cmake/verify_minimal.sh`
- Create: `scripts/cmake/test_verify_minimal.py`
- Create: `docs/quality/cmake-build-profiles.md`

**Interfaces:**
- Produces command `scripts/cmake/verify_minimal.sh [source-root]`.
- Uses the root `autonomy-minimal` configure/build/test presets.
- Compares tracked working-tree state before and after without requiring a clean baseline.

- [ ] **Step 1: Write failing verification-script tests**

Use a temporary fake `cmake` and `git` on `PATH`. Assert the script invokes, in order:

```text
cmake --preset autonomy-minimal
cmake --build --preset autonomy-minimal
ctest --preset autonomy-minimal --output-on-failure
```

The fake Git output contains a pre-existing modified path; the test proves that
unchanged baseline dirt is accepted, while a newly changed tracked path fails
with `minimal build changed tracked source files`.

- [ ] **Step 2: Verify RED**

Run: `python3 -m unittest scripts.cmake.test_verify_minimal -v`

Expected: failure because `verify_minimal.sh` does not exist.

- [ ] **Step 3: Implement source-clean verification**

Use `set -euo pipefail`. Snapshot both of these before and after:

```bash
git diff --binary
git diff --cached --binary
```

Compare the snapshots byte-for-byte. Do not use `git diff --exit-code` because
the approved baseline is already dirty. Run commands from the provided source
root and remove only the script-owned `build/autonomy-minimal` directory before
configuration.

- [ ] **Step 4: Document the profile**

Record:

- CMake 3.20+, Ninja, and compiler prerequisites;
- enabled and disabled domains;
- retained common dependency groups;
- exact preset and verification commands;
- the evidence vocabulary from the design;
- macOS and Ubuntu result tables with commit/snapshot, compiler, command, and outcome;
- explicit statement that full Autonomy, hardware, gRPC, AutoViz, inference,
  installation consumption, and runtime robot behavior remain unverified.

- [ ] **Step 5: Run static contract verification**

Run:

```bash
python3 -m unittest cmake.tests.test_autonomy_options -v
python3 -m unittest cmake.tests.test_autonomy_dependencies -v
python3 -m unittest scripts.cmake.test_presets -v
python3 -m unittest scripts.cmake.test_verify_minimal -v
bash -n scripts/cmake/verify_minimal.sh
git diff --check -- CMakeLists.txt CMakePresets.json cmake scripts/cmake docs/quality/cmake-build-profiles.md
```

Expected: all contract tests pass and no whitespace errors are reported.

- [ ] **Step 6: Run the real minimal workflow**

Run: `scripts/cmake/verify_minimal.sh "$PWD"`

Expected: configure, build, and at least one CTest pass; the staged and unstaged
tracked patches remain byte-identical to their pre-run snapshots. Record exact
test names and counts. If a retained common dependency or existing source error
prevents completion, record the exact first failure as `unverified`; do not
claim Phase A acceptance and do not modify runtime C++ under this plan.

- [ ] **Step 7: Compare with the original baseline**

Run:

```bash
git status --short > /tmp/autonomy-cmake-phase-a-baseline/status-final.txt
git diff --check
diff -u /tmp/autonomy-cmake-phase-a-baseline/head.txt <(git rev-parse HEAD)
```

Expected: `HEAD` is unchanged from the implementation baseline. Review the
initial and final status files manually; only plan-listed files may be new or
newly modified.

## Phase A Completion Gate

Phase A is complete only when:

- invalid domain graphs fail before package discovery;
- disabled domains are absent from subdirectory and test traversal;
- disabled domains do not trigger Lua, FBow, G2o, PCL, BehaviorTree.CPP, gRPC,
  Qt6, Prometheus, or Sherpa ONNX discovery;
- `CMakePresets.json` remains compatible with CMake 3.20;
- `autonomy-minimal` configures and builds common, transform, and vehicle;
- at least one named CTest passes;
- tracked staged and unstaged source patches are unchanged by the build;
- no pre-existing working-tree change has been staged or committed.

If the real build exposes dependency over-propagation inside common, that result
becomes the first failing test and input for Phase B; it is not hidden by adding
disabled-domain packages back to the minimal preset.
