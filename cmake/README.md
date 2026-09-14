# Autonomy CMake（对照 Gazebo / gz-sim）

参考 [gz-sim](https://github.com/gazebosim/gz-sim) / [gz-cmake](https://github.com/gazebosim/gz-cmake)。

| gz | Autonomy |
|----|----------|
| `gz_configure_project` | `autonomy_configure_project` |
| options / `find_package` | **写在根** `CMakeLists.txt` |
| `gz_configure_build` | `autonomy_configure_tests` → codegen → `autonomy_configure_build` |
| `gz_create_packages` | `autonomy_create_packages` |
| `gz_create_core_library` | `autonomy_library` |
| `gz_add_component` | `autonomy_component` |

## 模块写法

```cmake
include(autonomy_module)
autonomy_module(planning)
autonomy_glob_srcs(_PLANNING_SRCS)
autonomy_library(
  DEPENDENCIES autonomy_common autonomy_transform autonomy_map
  SRCS ${_PLANNING_SRCS})
autonomy_binary(autonomy.planning SRCS planning_main.cpp)
```

模块仅作为超级工程的 `add_subdirectory` 使用（不再支持 `cmake -S autonomy/<mod>` 独立编译）。

## 模块条件

```cmake
set(AUTONOMY_MODULE_CONDITION_bridge BUILD_GRPC)
set(AUTONOMY_MODULE_CONDITION_visualization foxglove-sdk_FOUND)
```

## `cmake/` 文件

| 文件 | 职责 |
|------|------|
| `autonomy_module.cmake` | `module` / `glob_srcs` / `library` / `binary` / `component` |
| `autonomy_project.cmake` | `configure_project` |
| `autonomy_version.cmake` | `version.json` / ConfigVersion / `version.cpp` |
| `autonomy_dependencies.cmake` | `link_core` / `link_feature` |
| `autonomy_common.cmake` | `autonomy_test` / 编译标志 |
| `autonomy_tests.cmake` | 收集 `*_test.cpp` 与 test helpers |
| `autonomy_build.cmake` | proto / modules / umbrella |
| `autonomy_package.cmake` | install / export / uninstall |
| `autonomy_protobuf.cmake` | proto / gRPC codegen |
| `autonomy_*_embed.cmake` | autolink / autodriver / autosim |
| `autonomy_config.cmake.in` | `find_package` Config |
| `autonomy_config_version.cmake.in` | ConfigVersion |

## FEATURES

`pcl` · `slam` · `bt` · `grpc` · `inference` · `osqp` · `cairo` · `boost_iostreams` · `ipopt` · `foxglove` · `prometheus`

`link_core`：Eigen / Protobuf / Ceres / Lua / OpenCV / yaml / glog / autolink / TBB 等通用依赖；更专的走 FEATURES。

## 刻意保留（非冗余）

- 根 `find_package(... REQUIRED)` 全家桶：产品默认全量依赖，不按 FEATURES 拆 optional find
- `EnsureProtobuf319`：版本钉死有现实约束
- `link_core` 仍含 Ceres/Lua/OpenCV：再拆易破构建，收益有限
