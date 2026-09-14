# Autonomy Common Dependency Boundary Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove OSQP from the mandatory common-module dependency set and make Eigen/Glog discovery portable enough for `autonomy-minimal` to continue configuration.

**Architecture:** Move the OSQP-backed MPC implementation into an optional `autonomy_common_osqp` target while keeping the base `autonomy_common` target free of OSQP headers and libraries. Normalize custom find modules around imported targets and correct package-name casing, then disable the optional target in the minimal preset without changing runtime algorithms.

**Tech Stack:** CMake 3.20, C++17, Python 3 `unittest`, CMake configure fixtures

**Spec:** `docs/superpowers/specs/2026-09-13-cmake-modularization-design.md`

## Global Constraints

- Continue in the current dirty checkout using `/tmp/autonomy-cmake-phase-a-baseline` as the pre-change evidence boundary.
- Do not stage, commit, restore, delete, or format pre-existing user changes.
- Do not change `MpcOsqp` implementation behavior or public C++ declarations.
- Keep `AUTONOMY_BUILD_COMMON_OSQP` default `ON` for compatibility; `autonomy-minimal` sets it `OFF`.
- The optional library target is `autonomy_common_osqp` with build alias `autonomy::common_osqp`.
- Exported/install behavior for the new target is deferred to the install-consumer phase; register it in `AUTONOMY_MODULE_TARGETS` so the existing exporter includes it when enabled.
- Use imported targets `Eigen3::Eigen`, `glog::glog`, and `OSQP::OSQP` when available.
- Do not add TBB or another unrelated package to the minimal profile to conceal the next dependency boundary.
- A new configure failure after OSQP is removed is evidence for the next dependency slice, not permission for unrelated changes.

---

### Task 1: Portable Eigen and Glog discovery

**Files:**
- Create: `cmake/tests/test_find_modules.py`
- Modify: `cmake/modules/FindEigen3.cmake`
- Modify: `cmake/modules/FindGlog.cmake`
- Modify: `cmake/autonomy_find_dependencies.cmake`

**Interfaces:**
- `find_package(Eigen3 REQUIRED)` guarantees `Eigen3::Eigen`, `Eigen3_FOUND`, `EIGEN3_FOUND`, and `EIGEN3_INCLUDE_DIRS`.
- `find_package(Glog REQUIRED)` guarantees `glog::glog`, `Glog_FOUND`, `GLOG_FOUND`, `GLOG_INCLUDE_DIRS`, and `GLOG_LIBRARIES`.

- [ ] **Step 1: Write failing configure-fixture tests**

Create temporary projects that place fake headers/libraries under a temporary
prefix. The Eigen fixture provides `include/eigen3/Eigen/Core`; the Glog fixture
provides `include/glog/logging.h` and an empty platform-named library file.

Run CMake with `-Werror=dev` and assert the imported targets and compatibility
variables exist. The Glog test must call `find_package(Glog REQUIRED)` with the
same case as `FindGlog.cmake`.

- [ ] **Step 2: Verify RED**

Run: `python3 -m unittest cmake.tests.test_find_modules -v`

Expected: Eigen lacks a guaranteed imported target or Homebrew-compatible hint,
and the existing lowercase Glog package name emits a developer warning or fails
the expected variable contract.

- [ ] **Step 3: Normalize Eigen discovery**

First attempt config mode:

```cmake
find_package(Eigen3 QUIET NO_MODULE)
```

When `Eigen3::Eigen` is absent, search `Eigen/Core` using `PATH_SUFFIXES eigen3`
and normal CMake prefixes, including `/opt/homebrew`, `/usr/local`, and `/usr`.
Create an imported interface target with `INTERFACE_INCLUDE_DIRECTORIES`, then
set both standard and legacy result variables. Use
`find_package_handle_standard_args(Eigen3 REQUIRED_VARS EIGEN3_INCLUDE_DIR)`;
do not emit a separate warning before REQUIRED handling.

- [ ] **Step 4: Normalize Glog discovery**

Use `find_package_handle_standard_args(Glog DEFAULT_MSG GLOG_INCLUDE_DIR
GLOG_LIBRARY)`, expose compatibility uppercase
variables after success, and retain the `glog::glog` imported target plus bare
`glog` alias. Change dependency discovery to `find_package(Glog REQUIRED)`.

- [ ] **Step 5: Verify GREEN**

Run:

```bash
python3 -m unittest cmake.tests.test_find_modules -v
python3 -m unittest cmake.tests.test_autonomy_dependencies -v
git diff --check -- cmake/modules/FindEigen3.cmake cmake/modules/FindGlog.cmake cmake/autonomy_find_dependencies.cmake cmake/tests
```

Expected: all fixture and dependency-group tests pass without developer warnings.

---

### Task 2: Optional OSQP-backed common target

**Files:**
- Modify: `cmake/tests/test_autonomy_dependencies.py`
- Modify: `cmake/modules/FindOSQP.cmake`
- Modify: `cmake/autonomy_find_dependencies.cmake`
- Modify: `autonomy/common/CMakeLists.txt`
- Modify: `cmake/autonomy_tests.cmake`

**Interfaces:**
- Adds option `AUTONOMY_BUILD_COMMON_OSQP` with default `ON`.
- Adds dependency group `osqp` only when that option is enabled.
- Adds imported target `OSQP::OSQP`.
- Adds optional target `autonomy_common_osqp` and alias `autonomy::common_osqp`.
- The base `autonomy_common` source list excludes `math/mpc_osqp.cpp` in every configuration.
- `mpc_osqp_test.cpp` is excluded when the optional target is disabled.

- [ ] **Step 1: Add failing dependency-group tests**

Extend the dependency test harness to initialize
`AUTONOMY_BUILD_COMMON_OSQP=OFF`. Assert the minimal group list remains:

```text
core;common_math;common_vision
```

Add a second assertion with `AUTONOMY_BUILD_COMMON_OSQP=ON` expecting:

```text
core;common_math;common_vision;osqp
```

Update `common_math` semantics to contain Ceres only.

- [ ] **Step 2: Add failing source-boundary tests**

In `cmake/tests/test_autonomy_options.py`, assert that the common CMake file:

- excludes `math/mpc_osqp.cpp` from `_COMMON_SRCS`;
- declares `autonomy_common_osqp` only inside
  `if(AUTONOMY_BUILD_COMMON_OSQP)`;
- links `autonomy_common`, `Eigen3::Eigen`, and `OSQP::OSQP`;
- registers `autonomy::common_osqp` and appends the target to
  `AUTONOMY_MODULE_TARGETS`.

Assert `cmake/autonomy_tests.cmake` filters `mpc_osqp_test.cpp` when the option
is disabled.

- [ ] **Step 3: Verify RED**

Run:

```bash
python3 -m unittest cmake.tests.test_autonomy_dependencies -v
python3 -m unittest cmake.tests.test_autonomy_options -v
```

Expected: the new group and target-boundary assertions fail.

- [ ] **Step 4: Provide the OSQP imported target**

After successful header/library discovery, create:

```cmake
add_library(OSQP::OSQP UNKNOWN IMPORTED)
set_target_properties(OSQP::OSQP PROPERTIES
  IMPORTED_LOCATION "${OSQP_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${OSQP_INCLUDE_DIR}")
```

Guard target creation with `if(NOT TARGET OSQP::OSQP)`.

- [ ] **Step 5: Make OSQP discovery optional by target selection**

Declare the option in `autonomy_find_dependencies.cmake`. Append the `osqp`
group only when `common` and `AUTONOMY_BUILD_COMMON_OSQP` are enabled. Move
`find_package(OSQP REQUIRED)` from `common_math` to the new group.

- [ ] **Step 6: Split the implementation target**

In `autonomy/common/CMakeLists.txt`, remove `math/mpc_osqp.cpp` from
`_COMMON_SRCS` before `autonomy_library()`. When enabled, create the shared
target from that single source, link the three required targets publicly, apply
the normal workspace/build/install include directories, add the build alias,
and append it to `AUTONOMY_MODULE_TARGETS`.

- [ ] **Step 7: Filter the optional test**

After test discovery, remove the exact path
`${_root}/common/math/mpc_osqp_test.cpp` when
`AUTONOMY_BUILD_COMMON_OSQP=OFF`. Do not remove any other common test.

- [ ] **Step 8: Verify GREEN**

Run:

```bash
python3 -m unittest cmake.tests.test_autonomy_dependencies -v
python3 -m unittest cmake.tests.test_autonomy_options -v
python3 -m unittest cmake.tests.test_find_modules -v
git diff --check -- autonomy/common/CMakeLists.txt cmake
```

Expected: all tests pass.

---

### Task 3: Minimal preset update and configure evidence

**Files:**
- Modify or create: `CMakePresets.json`
- Modify or create: `scripts/cmake/test_presets.py`
- Modify: `docs/quality/cmake-build-profiles.md` only if it already exists from Phase A

**Interfaces:**
- `autonomy-minimal` sets `AUTONOMY_BUILD_COMMON_OSQP=OFF`.
- The existing Phase A preset contract otherwise remains unchanged.

- [ ] **Step 1: Add the failing preset assertion**

Assert the minimal configure preset contains exactly:

```json
"AUTONOMY_BUILD_COMMON_OSQP": "OFF"
```

- [ ] **Step 2: Verify RED**

Run: `python3 -m unittest scripts.cmake.test_presets -v`

Expected: failure because the option is absent, or because Task 4 of the Phase A
plan has not yet created the preset files.

- [ ] **Step 3: Add or update the preset**

Follow the complete cache-variable contract in
`docs/superpowers/plans/2026-09-13-cmake-phase-a-minimal-profile.md`, adding the
OSQP option above. Keep preset schema version 2 and the binary directory
`${sourceDir}/build/autonomy-minimal`.

- [ ] **Step 4: Run the focused configure**

Run:

```bash
python3 -m unittest scripts.cmake.test_presets -v
cmake --list-presets
cmake --preset autonomy-minimal
```

Expected: the previous OSQP error is absent. Record the first subsequent result
exactly. If configuration succeeds, label it `configured`; if another current
universal dependency blocks generation, label the profile `unverified` and use
that dependency as the next Phase B slice.

- [ ] **Step 5: Verify baseline preservation**

Run:

```bash
git diff --check -- CMakeLists.txt CMakePresets.json autonomy/common/CMakeLists.txt cmake scripts/cmake
git rev-parse HEAD
```

Expected: record the live `HEAD`; any concurrent advancement must be inspected
and contain no overlap with this plan's implementation files. No unrelated path
may be newly modified by this work.

## Completion Gate

This slice is complete when:

- Eigen and Glog find modules pass warning-as-error configure fixtures;
- OSQP is absent from the base common dependency groups;
- `autonomy_common` does not compile or publish OSQP through its link interface;
- enabling the compatibility option creates `autonomy_common_osqp`;
- disabling it removes only the implementation and its focused test;
- `autonomy-minimal` no longer fails on OSQP;
- no algorithm source or pre-existing working-tree change is overwritten,
  staged, or committed.

---

### Task 4: Minimal target dependency boundary

**Files:**
- Create: `cmake/tests/test_minimal_target_boundaries.py`
- Modify: `cmake/modules/FindCeres.cmake`
- Modify: `cmake/autonomy_find_dependencies.cmake`
- Modify: `cmake/autonomy_module.cmake`
- Modify: `cmake/autonomy_build.cmake`
- Modify: `cmake/autonomy_dependencies.cmake`
- Modify: `autonomy/common/CMakeLists.txt`
- Modify: `autonomy/transform/CMakeLists.txt`
- Modify: `autonomy/vehicle/CMakeLists.txt`

**Interfaces:**
- `autonomy_library(NO_CORE ...)` skips the transitional universal dependency bundle.
- `autonomy_proto` links only `protobuf::libprotobuf` and `automsgs` publicly.
- `autonomy_common` explicitly links its current required targets without TBB.
- `autonomy_transform` and `autonomy_vehicle` explicitly link `autonomy_common`
  and their generated message implementation target `autonomy_proto`.
- `find_package(Ceres REQUIRED)` guarantees `Ceres::ceres`.
- Dependency discovery creates internal interface target `autonomy_opencv` and alias `autonomy::opencv` from `OpenCV_INCLUDE_DIRS` and `OpenCV_LIBS`.
- An empty `autonomy_test_library` is an `INTERFACE` target.

- [ ] **Step 1: Write failing configure and target-property tests**

Create a temporary top-level include that uses `cmake_language(DEFER)` to write
`INTERFACE_LINK_LIBRARIES` and `INTERFACE_INCLUDE_DIRECTORIES` for
`autonomy_proto`, `autonomy_common`, `autonomy_transform`, and
`autonomy_vehicle`. Configure `autonomy-minimal` and assert:

- no target interface contains `TBB::tbb`;
- no interface contains a raw source-directory path outside a
  `$<BUILD_INTERFACE:...>` expression;
- proto contains Protobuf and Automsgs but not Ceres, OpenCV, yaml-cpp, or TBB;
- transform and vehicle depend on common;
- configuration does not contain the linker-language error for
  `autonomy_test_library`.

- [ ] **Step 2: Verify RED**

Run: `python3 -m unittest cmake.tests.test_minimal_target_boundaries -v`

Expected: current target interfaces contain TBB, raw module source paths, and
the empty test-library configuration fails.

- [ ] **Step 3: Add imported dependency targets**

Add `Ceres::ceres` to `FindCeres.cmake`. After `find_package(OpenCV REQUIRED)`,
create `autonomy_opencv` as an `INTERFACE` library with build-system include
directories and libraries, plus `autonomy::opencv` alias. Use
`yaml-cpp::yaml-cpp` consistently.

- [ ] **Step 4: Add the transitional `NO_CORE` switch**

Extend `autonomy_library()` flag parsing from `INTERFACE` to
`INTERFACE;NO_CORE`. Call `autonomy_link_core()` only when `NO_CORE` is absent.
This flag is temporary until every module declares explicit dependencies.

- [ ] **Step 5: Migrate the minimal targets**

Configure `autonomy_proto` directly with build/install include directories and
public links `protobuf::libprotobuf automsgs`.

Build common with:

```text
protobuf::libprotobuf
automsgs
Eigen3::Eigen
Ceres::ceres
autonomy::opencv
yaml-cpp::yaml-cpp
glog::glog
gflags::gflags
autolink
Threads::Threads
nlohmann_json::nlohmann_json
```

Build transform and vehicle with `NO_CORE`, `autonomy_common`, and
`autonomy_proto`. Do not add TBB.

- [ ] **Step 6: Handle an empty test-helper set**

When `TEST_LIBRARY_SRCS` is empty, create `autonomy_test_library` as
`INTERFACE`, propagate only its declared header/include/link usage, and do not
set compiler properties requiring a linker language. Preserve the existing
ordinary library path when helper sources exist.

- [ ] **Step 7: Verify GREEN**

Run:

```bash
python3 -m unittest cmake.tests.test_minimal_target_boundaries -v
python3 -m unittest cmake.tests.test_autonomy_options cmake.tests.test_autonomy_dependencies cmake.tests.test_find_modules scripts.cmake.test_presets -v
cmake --preset autonomy-minimal
cmake --build --preset autonomy-minimal
ctest --preset autonomy-minimal --output-on-failure
```

Expected: configuration and generation succeed without raw source-directory
exports, TBB, or empty-linker-language errors; build and named tests determine
the next evidence level.
