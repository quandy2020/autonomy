# Autonomy CMake Modularization and Validation Design

Date: 2026-09-13

Status: Proposed

Scope: the current in-progress CMake modularization in the Autonomy working tree

## 1. Context

Autonomy is being migrated from a broad aggregate build toward domain libraries
such as `autonomy_common`, `autonomy_transform`, `autonomy_map`,
`autonomy_control`, and `autonomy_planning`. The current working tree contains
the uncommitted module CMake files, helper APIs, configuration relocation, and
related source adjustments that define this new architecture.

The target names and directory structure are substantially improved, but the
build still behaves like a monolith in several important ways:

- all enabled modules receive the same large `PUBLIC` core dependency set;
- root configuration discovers many domain-specific dependencies
  unconditionally;
- executables link the aggregate `autonomy` target by default;
- production sources and tests are discovered recursively;
- installed exports have not been proven by a clean consumer project;
- no root CMake preset describes a reproducible minimal configuration.

This design stabilizes the existing modularization before adding further
features. It does not discard or rewrite the current working-tree migration.

## 2. Working-Tree Protection

The current uncommitted state is the implementation baseline. Work must:

1. record `HEAD` and `git status --short` before every phase;
2. preserve all pre-existing changes and untracked module files;
3. avoid repository-wide formatting or mechanical rewrites;
4. modify only files required by the active phase;
5. report new changes separately from the pre-existing baseline;
6. avoid committing pre-existing changes unless the user explicitly requests
   that consolidation;
7. stop a specific edit when its ownership or intended content is ambiguous,
   while continuing independent work where safe.

Because a conventional Git worktree created from `HEAD` would omit the current
uncommitted architecture, implementation occurs in the current checkout.

## 3. Goals

1. Provide a reproducible `autonomy-minimal` configuration shared by developers
   and continuous integration.
2. Make module targets express their actual public and private dependencies.
3. Prevent internal executables from linking every Autonomy module by default.
4. Give every first-party test an explicit owning module and dependency set.
5. Prove that an installed package can be consumed through
   `find_package(autonomy CONFIG REQUIRED)`.
6. Preserve a compatibility umbrella target without using it as the internal
   default dependency.

## 4. Non-Goals

- algorithm or runtime behavior changes;
- source-directory reorganization beyond CMake ownership files;
- immediate elimination of every recursive glob in imported or adapted code;
- physical-hardware, vendor-SDK, visualization, or GPU qualification;
- ROS 2 migration;
- functional-safety or production-readiness claims;
- automatic commit of the current working-tree migration.

## 5. Target Model

### 5.1 Domain targets

Each domain remains a separately linkable target with a namespaced build alias:

```text
autonomy_common        -> autonomy::common
autonomy_transform     -> autonomy::transform
autonomy_map           -> autonomy::map
autonomy_vehicle       -> autonomy::vehicle
autonomy_prediction    -> autonomy::prediction
autonomy_control       -> autonomy::control
autonomy_planning      -> autonomy::planning
autonomy_perception    -> autonomy::perception
autonomy_localization  -> autonomy::localization
autonomy_sensor        -> autonomy::sensor
autonomy_task          -> autonomy::task
autonomy_system        -> autonomy::system
autonomy_audio         -> autonomy::audio
autonomy_bridge        -> autonomy::bridge
autonomy_visualization -> autonomy::visualization
```

The `autonomy` / `autonomy::autonomy` interface target remains available as a
compatibility umbrella. First-party libraries, components, executables, and
tests must not link it merely for convenience.

### 5.2 Dependency visibility

The module API accepts distinct dependency classes:

- `PUBLIC_DEPENDENCIES`: required by installed public headers or ABI;
- `PRIVATE_DEPENDENCIES`: required only to compile or link the implementation;
- `FEATURES`: optional dependency bundles whose availability is controlled by
  an explicit build option or package result.

There is no universal dependency bundle containing Ceres, OpenCV, TBB,
Autolink, Automsgs, Protobuf, and every common utility. A small helper may
provide compiler settings and build/install include directories, but dependency
targets remain explicit at the module call site.

The intended domain direction is:

```text
common <- transform <- map/localization <- planning/control <- task/system
   ^          ^             ^                    ^             ^
   +----------+-------------+---- perception ----+             |
   +----------------------------- bridge/audio/sensor ----------+
```

Any intentional reverse or cross-domain edge must be visible in the owning
module CMake file and supported by its public-header requirements.

### 5.3 Executables and components

`autonomy_binary()` and `autonomy_component()` consume explicitly named domain
targets. They do not automatically link the compatibility umbrella.

Examples:

```cmake
autonomy_binary(
  autonomy.control
  SRCS control_main.cpp
  DEPENDENCIES autonomy_control)

autonomy_component(
  follow_component
  SOURCES ${_FOLLOW_SRCS}
  PRIVATE_DEPENDENCIES autonomy_perception autonomy_task)
```

The exact dependencies are derived from includes and link errors, then recorded
at the narrowest correct visibility.

## 6. Minimal Build Profile

Create a root `CMakePresets.json` using a schema supported by CMake 3.20 or
raise the documented minimum version deliberately. The initial preset is named
`autonomy-minimal` and uses an out-of-source Ninja build.

The profile disables at least:

- documentation;
- AutoViz and AutoSim;
- gRPC bridge;
- Prometheus;
- ONNX Runtime and TensorRT;
- Sherpa ONNX;
- AutoDriver vendor SDKs and examples;
- grid-map demos;
- optional tools that require dependencies outside the selected domains.

Tests remain enabled. The initial domain set is `common`, `transform`,
`vehicle`, and the smallest additional dependencies required to exercise a
real executable or consumer. Disabled domains must not trigger their
domain-specific `find_package()` calls.

Acceptance requires:

```text
cmake --preset autonomy-minimal
cmake --build --preset autonomy-minimal
ctest --preset autonomy-minimal --output-on-failure
```

The commands must not modify tracked source files.

## 7. Dependency Discovery

Dependency discovery follows enabled targets:

- common build requirements are discovered once at the root;
- domain-only packages are discovered only when their domain is enabled;
- optional packages use `QUIET` only when the source has a tested stub or
  disabled path;
- a requested feature without its required package fails with a message naming
  the controlling option and expected package;
- no developer-home absolute path is used as a fallback.

Module options use the `AUTONOMY_BUILD_<DOMAIN>` prefix. Compatibility with the
existing broad build is preserved by defaulting current product domains to
`ON`; the minimal preset chooses its smaller set explicitly.

## 8. Source Ownership

First-party production modules transition from recursive globbing to explicit
source lists. Migration proceeds one verified module at a time:

1. common;
2. transform and vehicle;
3. map;
4. planning and control;
5. task and system;
6. perception and localization;
7. bridge, sensor, audio, and visualization.

Imported source trees with large upstream-maintained file sets may retain a
local `GLOB_RECURSE CONFIGURE_DEPENDS` when their owning CMake file contains a
documented boundary and excludes tests, examples, benchmarks, and tools.

An explicit first-party list must make adding a new implementation file require
a reviewed CMake change.

## 9. Test Ownership

Tests are registered in the owning module's `CMakeLists.txt` using a helper with
explicit sources and dependencies. A module test links only the module under
test plus declared test utilities.

The shared test-helper target is split when helpers introduce domain
dependencies. Filename-based recursive discovery is removed after every test is
accounted for. Adapted upstream tests are distinguishable from Autonomy
integration tests.

Each migrated module must support:

```text
cmake --build <build-dir> --target <module-test-target>
ctest --test-dir <build-dir> -R <module-prefix> --output-on-failure
```

## 10. Installation and Consumer Contract

Installed targets use `autonomy::` consistently. `autonomy-config.cmake` calls
`find_dependency()` for every external target appearing in an exported public
interface, including conditionally enabled features.

An install-consumer fixture performs this sequence:

1. configure and build Autonomy;
2. install into a clean temporary prefix;
3. configure a separate CMake project using only `CMAKE_PREFIX_PATH`;
4. resolve requested components through
   `find_package(autonomy CONFIG REQUIRED COMPONENTS common transform)`;
5. compile and link a minimal executable against the installed targets;
6. run the executable when it has no runtime hardware dependency.

The fixture must not access headers, libraries, generated files, or CMake files
from the source or original build directories.

## 11. Continuous Integration

After the local minimal and consumer paths are reproducible, CI runs:

- CMake configure and build with GCC;
- CMake configure and build with Clang;
- module-owned CTest selection;
- install-consumer validation;
- AddressSanitizer and UndefinedBehaviorSanitizer on the minimal profile;
- a source-tree cleanliness check after configure and build.

CI does not require hardware, private SDKs, credentials, AutoViz, GPU inference,
or gRPC in this increment.

## 12. Delivery Phases

### Phase A: Minimal profile and configuration boundary

- introduce module enable options;
- gate domain-specific dependency discovery;
- add `autonomy-minimal` configure/build/test presets;
- prove source-tree cleanliness.

Acceptance: the minimal preset configures without disabled-domain dependencies
and produces at least one meaningful passing test.

### Phase B: Real dependency boundaries

- replace the universal public core dependency set;
- add public/private dependency arguments;
- migrate common, transform, and vehicle first;
- verify their targets independently.

Acceptance: these targets' exported link interfaces contain only dependencies
required by their public contract.

### Phase C: Executable and test ownership

- remove automatic umbrella linking from binaries and components;
- migrate executable dependencies explicitly;
- move test registration into owning modules;
- retain a temporary compatibility path only for unmigrated tests.

Acceptance: migrated binaries and tests build without linking the compatibility
umbrella.

### Phase D: Explicit source ownership

- replace first-party recursive globs in the established migration order;
- document the retained boundaries of imported-source globs.

Acceptance: adding an arbitrary first-party `.cpp` file does not silently alter
an existing target.

### Phase E: Install consumer and CI

- make exported public dependencies complete and symmetric;
- add the clean install-consumer fixture;
- add GCC, Clang, sanitizer, and cleanliness CI jobs.

Acceptance: a clean consumer configures, links, and runs exclusively against
the install prefix, and the same workflow succeeds in CI.

## 13. Evidence Rules

Results are classified as:

- **source-confirmed**: established by inspecting CMake or source files;
- **configured**: a named configuration completed successfully;
- **built**: the selected target set compiled and linked successfully;
- **tested**: named CTest cases completed successfully;
- **consumer-verified**: a clean installed-package consumer succeeded;
- **unverified**: not exercised in the recorded environment.

No later evidence level is inferred from an earlier one. Every result records
the commit or working-tree snapshot, OS, compiler, preset, command, and outcome.

## 14. Completion Criteria

The stabilization is complete when:

- `autonomy-minimal` is reproducible locally and in CI;
- disabled domains do not impose their dependencies;
- migrated domain libraries declare accurate public/private dependencies;
- internal binaries and migrated tests do not link the umbrella target;
- first-party source ownership is explicit for all migrated modules;
- the install-consumer fixture passes from a clean prefix;
- GCC, Clang, ASan, and UBSan jobs pass for the minimal profile;
- configuration and build do not dirty tracked sources;
- the original uncommitted working-tree baseline has not been overwritten or
  silently included in unrelated commits.
