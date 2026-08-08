# Autoviz C++ Tutorials

C++ publishers/subscribers mirroring [`../python`](../python) tutorials **01–24**.
Same channel names and message semantics for live Autoviz verification.

## Build

From the autonomy / autoviz CMake build (requires `autolink` + `automsgs` targets):

```bash
cmake --build build --target autoviz_cpp_01_poses
# or build all:
cmake --build build --target autoviz_cpp_24_teleop_echo
```

Binaries land in `build/bin/examples/autoviz_cpp_*` (exact prefix depends on
`CMAKE_BINARY_DIR` layout).

URDF-backed demos (06 / 08 / 18) read
`examples/python/urdf/pr2_simple.urdf`. Run from the **autoviz** package root, or:

```bash
export AUTOVIZ_EXAMPLES_DIR=/path/to/autoviz/examples
```

## Run

```bash
cd src/autonomy/autoviz   # or your checkout of autoviz/
./build/bin/examples/autoviz_cpp_01_poses --rate 10
./build/autonomy/bin/autoviz   # path may vary
```

Common flags: `--rate <Hz>` (most publishers); `--static` (01 poses).

## Index (↔ Python)

| Binary | Source | Python counterpart |
|---|---|---|
| `autoviz_cpp_01_poses` | `01_tutorial_poses.cpp` | `01_tutorial_poses.py` |
| `autoviz_cpp_02_occupancy_grid` | `02_tutorial_occupancy_grid.cpp` | `02_…` |
| `autoviz_cpp_03_image` | `03_tutorial_image.cpp` | `03_…` |
| `autoviz_cpp_04_sensor` | `04_tutorial_sensor.cpp` | `04_…` |
| `autoviz_cpp_05_tf` | `05_tutorial_transform_tree.cpp` | `05_…` |
| `autoviz_cpp_06_urdf` | `06_tutorial_urdf.cpp` | `06_…` |
| `autoviz_cpp_07_marker` | `07_tutorial_visualization_marker.cpp` | `07_tutorial_visualizarion_marker.py` |
| `autoviz_cpp_08_force` | `08_tutorial_force_joint.cpp` | `08_…` |
| `autoviz_cpp_09_plot` | `09_tutorial_plot.cpp` | `09_…` |
| `autoviz_cpp_10_log` | `10_tutorial_log.cpp` | `10_…` |
| `autoviz_cpp_11_state` | `11_tutorial_state_transitions.cpp` | `11_…` |
| `autoviz_cpp_12_channel_graph` | `12_tutorial_channel_graph.cpp` | `12_tutorial_channel_grpah.py` |
| `autoviz_cpp_13_raw` | `13_tutorial_raw_messages.cpp` | `13_…` |
| `autoviz_cpp_14_table` | `14_tutorial_table.cpp` | `14_tutiorial_table.py` |
| `autoviz_cpp_15_diagnostics` | `15_tutorial_diagnostics.cpp` | `15_…` |
| `autoviz_cpp_16_parameters` | `16_tutorial_parameters.cpp` | `16_…` |
| `autoviz_cpp_17_visions` | `17_tutorial_visions.cpp` | `17_…` |
| `autoviz_cpp_18_robot_all` | `18_tutorial_robot_all_data_sim.cpp` | `18_…` |
| `autoviz_cpp_19_geometry` | `19_tutorial_geometry_extras.cpp` | `19_…` |
| `autoviz_cpp_20_scalar` | `20_tutorial_scalar_env.cpp` | `20_…` |
| `autoviz_cpp_21_audio` | `21_tutorial_audio.cpp` | `21_…` |
| `autoviz_cpp_22_service` | `22_tutorial_service.cpp` | `22_…` |
| `autoviz_cpp_23_map_gps` | `23_tutorial_map_gps.cpp` | `23_…` |
| `autoviz_cpp_24_teleop_echo` | `24_tutorial_teleop_echo.cpp` | `24_…` |

Shared helpers: [`common/tutorial_utils.hpp`](common/tutorial_utils.hpp).

JSON-only channels (`foxglove.Log` / `foxglove.RawAudio`) use
`autolink::message::RawMessage` with explicit `message_type` on the writer.

Panel / Display acceptance steps match the Python [coverage matrix](../python/README.md#coverage-matrix).
