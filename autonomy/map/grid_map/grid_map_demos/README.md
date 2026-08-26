# grid_map_demos（无 ROS）

对照 ANYbotics/ROS `grid_map_demos`，在本仓库以**离线可执行程序**形式适配：无 `roscpp` / `rviz` / `pluginlib` / bag / octomap。输出写入本地 PNG / protobuf，便于 CI 与调试。

布局与 `grid_map_cv` / `grid_map_pcl` 一致：源码平铺在包根目录（`snake_case`），公共工具为 `helpers.hpp`，聚合头为 `grid_map_demos.hpp`，命名空间 `grid_map::grid_map_demos`。

## 构建

```bash
colcon build --packages-select autonomy \
  --cmake-args -DBUILD_GRID_MAP_DEMOS=ON -DBUILD_AUTOVIZ=OFF -DBUILD_AUTOSIM=OFF
```

产物在 `build/autonomy/bin/grid_map_*`。

## Demo 对照

| ROS 节点 | 本工程二进制 | 说明 |
|----------|--------------|------|
| `simple_demo` | `grid_map_simple_demo` | 正弦 elevation，写 PNG/PB |
| `tutorial_demo` | `grid_map_tutorial_demo` | 噪声/滤波/子图迭代 |
| `opencv_demo` | `grid_map_opencv_demo` | Gaussian blur；可选 `--gui` |
| `move_demo` | `grid_map_move_demo` | `move` vs `setPosition` |
| `resolution_change_demo` | `grid_map_resolution_change_demo` | `GridMapCvProcessing::changeResolution` |
| `iterators_demo` | `grid_map_iterators_demo` | 各迭代器结果图 |
| `filters_demo` | `grid_map_filters_demo` | YAML `FilterChain` + `FilterFactory` |
| `image_to_gridmap_demo` | `grid_map_image_to_gridmap_demo` | 读 PNG → GridMap |
| `grid_map_to_image_demo` | `grid_map_gridmap_to_image_demo` | GridMap/PB → PNG |
| `sdf_demo` | `grid_map_sdf_demo` | SDF + PointCloud2 protobuf |
| `normal_filter_comparison` | `grid_map_normal_filter_comparison_demo` | area vs raster 法向误差 |
| `interpolation_demo` | `grid_map_interpolation_demo` | 插值误差统计 |
| `iterator_benchmark` | `grid_map_iterator_benchmark` | 迭代性能对比 |
| `octomap_to_gridmap_demo` | （未迁） | 无 octomap 依赖 |
| launch / rviz / bag loader | （未迁） | 用文件 I/O + autoviz 替代 |

## 示例

```bash
# 简单 demo
./build/autonomy/bin/grid_map_simple_demo 30 /tmp/gm/simple

# 滤波器链（默认使用包内 config）
./build/autonomy/bin/grid_map_filters_demo \
  autonomy/map/grid_map/grid_map_demos/config/filters_demo_filter_chain.yaml \
  /tmp/gm/filters

# 图像 → 地图
./build/autonomy/bin/grid_map_image_to_gridmap_demo \
  autonomy/map/grid_map/grid_map_demos/data/terrain.png /tmp/gm/img

# 插值
./build/autonomy/bin/grid_map_interpolation_demo Sine Cubic /tmp/gm/interp
```

安装后可将 `AUTONOMY_PREFIX` 指到 install 前缀，以便解析 `share/autonomy/map/grid_map/grid_map_demos/{config,data}`。
