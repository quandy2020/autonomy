# Explicit Product Dependencies Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove implicit `autonomy` umbrella linkage from first-party executables and loadable components while preserving their names, runtime infrastructure, and installation behavior.

**Architecture:** `autonomy_binary()` requires explicit `DEPENDENCIES`; `autonomy_component()` requires at least one explicit public or private library. Domain `CMakeLists.txt` files declare the narrow owning module, while Autolink and Automsgs remain component-runtime infrastructure injected by the component helper.

**Tech Stack:** CMake 3.20, C++17, Python `unittest`, Ninja, CTest

**Spec:** `docs/superpowers/specs/2026-09-14-cmake-explicit-product-dependencies-design.md`

## Global Constraints

- No executable or component may implicitly link `autonomy` or `autonomy::autonomy`.
- Existing target names, output names, install destinations, RPATH, and CTest names remain unchanged.
- Components continue to receive `autolink` unless `NO_AUTOLINK` is specified and receive `automsgs` when that target exists.
- Every helper call declares at least one explicit domain or product dependency.
- The Minimal profile remains configurable, buildable, testable, and consumable after installation.
- Do not modify unrelated dirty-worktree paths.

---

### Task 1: Migrate all first-party call sites

**Files:**
- Modify: `cmake/tests/test_autonomy_options.py`
- Modify: `autonomy/control/CMakeLists.txt`
- Modify: `autonomy/task/CMakeLists.txt`
- Modify: `autonomy/localization/CMakeLists.txt`
- Modify: `autonomy/perception/CMakeLists.txt`
- Modify: `autonomy/visualization/CMakeLists.txt`
- Modify: `autonomy/bridge/CMakeLists.txt`
- Modify: `autonomy/planning/CMakeLists.txt`
- Modify: `autonomy/system/CMakeLists.txt`
- Modify: `autonomy/perception/base/CMakeLists.txt`
- Modify: `autonomy/perception/follow/CMakeLists.txt`
- Modify: `autonomy/audio/CMakeLists.txt`
- Modify: `autonomy/map/grid_map/grid_map_demos/CMakeLists.txt`

**Interfaces:**
- Consumes: the existing permissive helper APIs.
- Produces: every first-party product declaration with explicit owning dependencies, ready for the strict Task 2 API.

- [ ] **Step 1: Write the failing call-site inventory test**

Add `test_product_call_sites_declare_explicit_dependencies`, parsing balanced `autonomy_binary(...)` and `autonomy_component(...)` calls across first-party CMake files. Assert every binary call contains `DEPENDENCIES` and every component contains `PUBLIC_LINK_LIBS` or `PRIVATE_LINK_LIBS`.

- [ ] **Step 2: Verify RED**

```bash
python3 -m unittest \
  cmake.tests.test_autonomy_options.AutonomyOptionsTest.test_product_call_sites_declare_explicit_dependencies -v
```

Expected: failure listing existing declarations without explicit dependencies.

- [ ] **Step 3: Migrate binaries**

Add the owning dependency to each call:

```cmake
control -> control
task -> task
localization -> localization
perception -> perception
foxglove_bridge -> visualization
bridge -> bridge
planning -> planning
monitor -> system
grid_map demos -> map
```

Keep the existing OrbisView declaration on `autonomy_orbisview`.

- [ ] **Step 4: Migrate components**

Add explicit private owning libraries:

```cmake
base_component   -> autonomy_perception
follow_component -> autonomy_perception
audio_component  -> autonomy_audio and yaml-cpp
```

Do not self-link a component target.

- [ ] **Step 5: Verify GREEN and configure**

```bash
python3 -m unittest cmake.tests.test_autonomy_options -v
cmake --preset autonomy-minimal
```

Expected: all pass while the permissive helper still preserves compatibility.

- [ ] **Step 6: Commit**

Stage only the listed call-site files and test, then commit:

```bash
git commit -m "build: declare product target dependencies"
```

### Task 2: Enforce explicit helper contracts

**Files:**
- Modify: `cmake/tests/test_autonomy_options.py`
- Modify: `cmake/autonomy_module.cmake`

**Interfaces:**
- Consumes: Task 1 declarations and the APIs `autonomy_binary(NAME SRCS ... DEPENDENCIES ...)` and `autonomy_component(NAME SOURCES ... PUBLIC_LINK_LIBS ... PRIVATE_LINK_LIBS ...)`.
- Produces: configure-time validation and helper-generated targets without umbrella links.

- [ ] **Step 1: Write failing behavior tests**

Add temporary-project configure tests that assert:

```text
autonomy_binary(no_deps SRCS main.cpp)
  -> configure failure mentioning DEPENDENCIES

autonomy_component(no_libs NO_AUTOLINK SOURCES component.cpp)
  -> configure failure mentioning PUBLIC_LINK_LIBS or PRIVATE_LINK_LIBS
```

Add a successful target-property probe whose stub dependencies verify that the binary links only its resolved explicit dependency plus `glog::glog` and `gflags::gflags`, and that the component excludes `autonomy` while retaining explicit libraries and available runtime infrastructure.

- [ ] **Step 2: Verify RED**

```bash
python3 -m unittest \
  cmake.tests.test_autonomy_options.AutonomyOptionsTest.test_product_helpers_require_explicit_dependencies -v
```

Expected: failure because both helpers accept missing domain dependencies and inject the umbrella.

- [ ] **Step 3: Implement the contracts**

In `autonomy_binary()`, fail when `_ARG_DEPENDENCIES` is empty, remove `target_link_libraries(${_exe} PUBLIC ${PROJECT_NAME})`, resolve every declared dependency, and link the resolved list privately.

In `autonomy_component()`, remove the umbrella-target existence check and umbrella insertion. Fail when both `_ARG_PUBLIC_LINK_LIBS` and `_ARG_PRIVATE_LINK_LIBS` are empty. Preserve existing Autolink/Automsgs injection and visibility of explicit libraries.

- [ ] **Step 4: Verify GREEN and regression suite**

```bash
python3 -m unittest cmake.tests.test_autonomy_options -v
cmake --preset autonomy-minimal
```

Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add cmake/autonomy_module.cmake cmake/tests/test_autonomy_options.py
git commit -m "build: require explicit product dependencies"
```

### Task 3: Verify target boundaries and release consumption

**Files:**
- Modify: `cmake/tests/test_minimal_target_boundaries.py`
- Modify only Task 1 or Task 2 files if verification exposes a regression.

**Interfaces:**
- Consumes: strict helpers and migrated declarations.
- Produces: executable/component target-boundary evidence and full Minimal release evidence.

- [ ] **Step 1: Extend the failing target-property probe**

Report representative executable link properties. Assert any Minimal-profile product built by the fixture links its declared domain target and does not contain `autonomy` or `autonomy::autonomy`. Add a small temporary component fixture and assert it links explicit domain/runtime targets without the umbrella.

- [ ] **Step 2: Verify RED against the pre-migration base fixture**

Run the focused boundary test before applying the final assertions to the migrated implementation, or temporarily remove one explicit link in the isolated fixture. Confirm the assertion fails for the missing declared dependency or umbrella contamination.

- [ ] **Step 3: Verify GREEN**

```bash
python3 -m unittest \
  cmake.tests.test_minimal_target_boundaries.MinimalTargetBoundariesTest.test_minimal_profile_has_explicit_export_safe_target_boundaries -v
```

Expected: pass.

- [ ] **Step 4: Run the complete CMake contract suite**

```bash
python3 -m unittest discover -s cmake/tests -p 'test_*.py' -v
python3 -m unittest tools.cmake.test_presets tools.cmake.test_install_consumer -v
```

Expected: all tests pass; the consumer test may require the build produced below before its final rerun.

- [ ] **Step 5: Run the stable Minimal build and CTest**

Ensure no concurrent process is deleting or reconfiguring `build/autonomy-minimal`, then run:

```bash
cmake --preset autonomy-minimal
cmake --build --preset autonomy-minimal -- -j1
ctest --preset autonomy-minimal --output-on-failure
python3 -m unittest tools.cmake.test_install_consumer -v
```

Expected: all commands exit zero, with no missing test executables.

- [ ] **Step 6: Check formatting and commit verification fixes if any**

```bash
git diff --check -- \
  cmake/autonomy_module.cmake \
  cmake/tests/test_autonomy_options.py \
  cmake/tests/test_minimal_target_boundaries.py \
  CMakePresets.json tools/cmake/test_presets.py \
  autonomy
```

Commit only actual verification fixes; do not create an empty verification commit.
