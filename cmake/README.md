# Autonomy CMake (Gazebo / gz-sim analogue)

CMake APIs in `autonomy_*.cmake` and `modules/` use **Doxygen-style** comments
(`@file`, `@brief`, `@param`, `@var`, …). Generate docs with
[Doxygen](https://www.doxygen.nl/) (`EXTENSION_MAPPING = cmake=C`) or CMake
`.rst:` blocks.

Inspired by [gz-sim](https://github.com/gazebosim/gz-sim) /
[gz-cmake](https://github.com/gazebosim/gz-cmake).

| gz | Autonomy |
|----|----------|
| `gz_configure_project` | `autonomy_configure_project` |
| options / `find_package` | root `CMakeLists.txt` + `autonomy_superproject.cmake` |
| `gz_configure_build` | `autonomy_configure_tests` → codegen → `autonomy_configure_build` |
| `gz_create_packages` | `autonomy_create_packages` |
| `gz_create_core_library` | `autonomy_library` |
| `gz_add_component` | `autonomy_component` |

## Reading order

1. **Root** [`CMakeLists.txt`](../CMakeLists.txt): product `option`s,
   `AUTONOMY_SUPERPROJECT ON`, `include(autonomy_superproject)`, superproject
   macros, `configure_build` / `create_packages`.
2. **Domain** `autonomy/<mod>/CMakeLists.txt`: only `include(autonomy_module)` +
   `autonomy_library(...)` (does not load build/package).
3. **Orchestration** — table below.

## Module recipe

```cmake
include(autonomy_module)
autonomy_module(planning)
autonomy_glob_srcs(_PLANNING_SRCS)
autonomy_library(
  DEPENDENCIES autonomy_common autonomy_transform autonomy_map
  SRCS ${_PLANNING_SRCS})
autonomy_binary(autonomy.planning SRCS planning_main.cpp)
```

## `cmake/` files

| File | Role |
|------|------|
| `autonomy_module.cmake` | Domain API: `module` / `glob` / `library` / `binary` / `component` |
| `autonomy_superproject.cmake` | Module graph, embeds (autolink/autodriver/autosim), test collect, proto, version |
| `autonomy_project.cmake` | `configure_project`, version / ConfigVersion / `version.cpp` |
| `autonomy_deps.cmake` | `find_dependencies`, `link_core` / `link_feature` |
| `autonomy_options.cmake` | Module switches and dependency graph (included by superproject) |
| `autonomy_build.cmake` | `collect_proto_sources`, modules, umbrella, unit-test registration |
| `autonomy_package.cmake` | install / export / uninstall |
| `autonomy_common.cmake` | `autonomy_test` / compile flags |
| `autonomy_config.cmake.in` | `find_package` Config |
| `modules/` | `Find*` / `EnsureProtobuf319` |

## FEATURES

`pcl` · `slam` · `bt` · `grpc` · `grpc_reflection` · `otel` · `inference` · `osqp` · `cairo` · `boost_iostreams` · `ipopt` · `foxglove` · `prometheus` · `lua`

`link_core`: Eigen / Protobuf / Ceres / OpenCV / yaml / glog / autolink / TBB, …
More specialized deps go through FEATURES.
