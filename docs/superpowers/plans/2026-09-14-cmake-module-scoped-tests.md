# CMake Module-Scoped Tests Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every Autonomy test link its owning module instead of the `autonomy` umbrella target while preserving existing CTest names and behavior.

**Architecture:** `autonomy_configure_tests()` collects tests and helpers into per-module variables. `autonomy_add_tests()` creates one support target per enabled module and passes the owning module target explicitly to `autonomy_test()`.

**Tech Stack:** CMake 3.20, C++17, GoogleTest/GoogleMock, Python `unittest`, CTest, Ninja

**Spec:** `docs/superpowers/specs/2026-09-14-cmake-distributed-modules-design.md`

## Global Constraints

- Preserve existing test executable and CTest names.
- Preserve the root super-project workflow.
- Keep `autonomy-minimal` continuously buildable and testable.
- Do not introduce new third-party dependencies.
- Test targets link only their owning module, module test-support target, GMock main, and explicit test-only dependencies.

---

### Task 1: Define the module-scoped collection contract

**Files:**
- Modify: `cmake/tests/test_autonomy_options.py`
- Modify: `cmake/autonomy_tests.cmake`

**Interfaces:**
- Consumes: `AUTONOMY_ENABLED_MODULES` and module source directories.
- Produces: `AUTONOMY_TEST_MODULES`, `AUTONOMY_TESTS_<module>`, `AUTONOMY_TEST_HELPER_SRCS_<module>`, and `AUTONOMY_TEST_HELPER_HDRS_<module>`.

- [ ] **Step 1: Write the failing contract test**

Add `test_tests_are_grouped_by_owning_module` asserting the new per-module variables exist and the global `ALL_TESTS` and `TEST_LIBRARY_*` collections are absent.

- [ ] **Step 2: Verify RED**

```bash
python3 -m unittest cmake.tests.test_autonomy_options.AutonomyOptionsTest.test_tests_are_grouped_by_owning_module -v
```

Expected: failure because the per-module variables do not exist.

- [ ] **Step 3: Implement per-module collection**

Populate and export the four variable families. Apply the existing OSQP, Ipopt, fake-data, and base-component exclusions inside their owning module lists.

- [ ] **Step 4: Verify GREEN**

Run Step 2 again and expect one passing test.

- [ ] **Step 5: Commit**

```bash
git add cmake/autonomy_tests.cmake cmake/tests/test_autonomy_options.py
git commit -m "build: group tests by autonomy module"
```

### Task 2: Require an explicit test link target

**Files:**
- Modify: `cmake/tests/test_autonomy_options.py`
- Modify: `cmake/autonomy_common.cmake`

**Interfaces:**
- Consumes: `autonomy_test(NAME SOURCE LINK_TARGET)`.
- Produces: a test executable linked to `GMOCK_LIBRARIES` and `LINK_TARGET`, without implicit `${PROJECT_NAME}` linkage.

- [ ] **Step 1: Write the failing explicit-target test**

Add `test_autonomy_test_requires_an_explicit_target` asserting the three-argument signature and absence of implicit `${PROJECT_NAME}` links.

- [ ] **Step 2: Verify RED**

```bash
python3 -m unittest cmake.tests.test_autonomy_options.AutonomyOptionsTest.test_autonomy_test_requires_an_explicit_target -v
```

Expected: failure on the current two-argument helper.

- [ ] **Step 3: Implement the explicit target**

Change the function signature to `function(autonomy_test NAME ARG_SRC LINK_TARGET)`. Add workspace and binary include roots directly, retain GMock, RPATH, compile options, and CTest registration, then link `LINK_TARGET` instead of `${PROJECT_NAME}`.

- [ ] **Step 4: Verify GREEN**

Run Step 2 again and expect one passing test.

- [ ] **Step 5: Commit**

```bash
git add cmake/autonomy_common.cmake cmake/tests/test_autonomy_options.py
git commit -m "build: require explicit test link targets"
```

### Task 3: Create module-owned support targets

**Files:**
- Modify: `cmake/tests/test_minimal_target_boundaries.py`
- Modify: `cmake/autonomy_build.cmake`

**Interfaces:**
- Consumes: Task 1 variables and targets named `autonomy_<module>`.
- Produces: `autonomy_<module>_test_support` and module-linked test executables.

- [ ] **Step 1: Write the failing target-boundary probe**

Report representative Common and Transform test link interfaces. Assert Common tests link `autonomy_common` but not Transform, Vehicle, or the umbrella target; assert the Transform test links `autonomy_transform` but not Vehicle or the umbrella target.

- [ ] **Step 2: Verify RED**

```bash
python3 -m unittest cmake.tests.test_minimal_target_boundaries.MinimalTargetBoundariesTest.test_minimal_profile_has_explicit_export_safe_target_boundaries -v
```

Expected: failure because tests currently inherit `${PROJECT_NAME}`.

- [ ] **Step 3: Implement module-owned support targets**

Loop over `AUTONOMY_TEST_MODULES`, create `autonomy_<module>_test_support` from only that module's helpers, and call `autonomy_test(TEST SOURCE autonomy_<module>)`. Link the matching support target privately. Preserve conditional gRPC and Prometheus test dependencies.

- [ ] **Step 4: Verify GREEN**

Run Step 2 again and expect one passing test.

- [ ] **Step 5: Commit**

```bash
git add cmake/autonomy_build.cmake cmake/tests/test_minimal_target_boundaries.py
git commit -m "build: link tests to owning modules"
```

### Task 4: Verify the Minimal profile

**Files:**
- Modify only files from Tasks 1-3 if verification exposes a regression.

**Interfaces:**
- Consumes: the module-scoped test graph.
- Produces: fresh configure, build, CTest, consumer, and formatting evidence.

- [ ] **Step 1: Configure and build**

```bash
cmake --preset autonomy-minimal
cmake --build --preset autonomy-minimal -- -j1
```

Expected: exit zero without duplicate GMock warnings.

- [ ] **Step 2: Run CTest**

```bash
ctest --preset autonomy-minimal --output-on-failure
```

Expected: 40 of 40 tests pass with unchanged names.

- [ ] **Step 3: Run contract and consumer tests**

```bash
python3 -m unittest cmake.tests.test_autonomy_options cmake.tests.test_autonomy_dependencies cmake.tests.test_find_modules cmake.tests.test_minimal_target_boundaries tools.cmake.test_presets tools.cmake.test_install_consumer -v
```

Expected: all tests pass.

- [ ] **Step 4: Check formatting**

```bash
git diff --check -- cmake/autonomy_tests.cmake cmake/autonomy_common.cmake cmake/autonomy_build.cmake cmake/tests/test_autonomy_options.py cmake/tests/test_minimal_target_boundaries.py
```

Expected: no output and exit zero.

- [ ] **Step 5: Commit verification fixes if needed**

```bash
git add cmake/autonomy_tests.cmake cmake/autonomy_common.cmake cmake/autonomy_build.cmake cmake/tests/test_autonomy_options.py cmake/tests/test_minimal_target_boundaries.py
git commit -m "test: verify module-scoped CMake targets"
```
