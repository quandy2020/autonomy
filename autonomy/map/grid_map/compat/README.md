# grid_map compat（OCS2 / ROS 风格 include）

本目录挂到 `autonomy_map` 的 PUBLIC include，使下列写法解析到 `autonomy/map/grid_map`：

| 经典 include | 实际头文件 |
|:---|:---|
| `<grid_map_core/GridMap.hpp>` | `grid_map_core/grid_map.hpp` |
| `<grid_map_core/grid_map_core.hpp>` | 同名 umbrella |
| `<grid_map_sdf/SignedDistanceField.hpp>` | `grid_map_sdf/signed_distance_field.hpp` |
| `<grid_map_cv/GridMapCvConverter.hpp>` | `grid_map_cv/grid_map_cv_converter.hpp` |
| `<grid_map_filters_rsl/lookup.hpp>` 等 | `grid_map_filters_rsl/*.hpp` |

`grid_map_filters_rsl`（lookup / inpainting / smoothing / …）已迁入  
`autonomy/map/grid_map/grid_map_filters_rsl/`，随 `autonomy_map` 一并编译。
