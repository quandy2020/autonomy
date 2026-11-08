# Mapping 可视化数据视图

## 模块定位

`mapping/visualization/` 若保留，只负责把只读地图/轨迹 snapshot 转成 transport-neutral 的展示数据；它不拥有地图真值、不参与匹配/优化、不创建窗口，也不发布 ROS/RViz 消息。其目的是让 CLI、RViz adapter、Web viewer 或离线工具复用同一视图转换，而核心 `mapping/` 不依赖展示代码。

该模块是可选层：当前 Atlas 不包含轨迹绘图或地图渲染工具。若未来只有单个离线工具需要导出 PLY/图片，可将转换留在对应应用；不要为了目录树齐全而保留空 `mapping/visualization/`。

## 推荐文件布局

```text
mapping/visualization/
├── map_view.hpp                           # 只读地图展示值类型
├── trajectory_view.hpp                    # 轨迹路径/坐标系展示值
├── map_view_builder.hpp/.cpp        # MapSnapshot 到点/平面/体素视图
├── trajectory_view_builder.hpp/.cpp       # pose snapshot 到路径/关键帧视图
├── visualization_options.hpp             # 降采样、范围和视图选择
└── internal/
    ├── voxel_surface_projection.hpp/.cpp # 平面/边界几何展开
    └── color_mapping.hpp/.cpp             # 按高度/强度/session 上色
```

若展示数据只是序列化点云，无须建立 `MapView` 和两层 adapter；直接由 app 工具读取 `MapSnapshot` 或 serializer 输出即可。若确有多个 consumer 使用，则让 view 只包含展示需要的紧凑值，不复制完整原始地图。

## 文件与类型职责

| 文件 | 类型 | 输出 |
|---|---|---|
| `map_view.hpp` | `MapView` | 地图版本、坐标系、点/平面/体素边界 buffer；不保存 ROS msg |
| `trajectory_view.hpp` | `TrajectoryView` | 带 timestamp、session ID 和位姿的路径视图 |
| `map_view_builder.*` | `MapViewBuilder` | 对只读 `MapSnapshot` 做范围裁剪/采样并生成视图 |
| `trajectory_view_builder.*` | `TrajectoryViewBuilder` | 由不可变 pose snapshot 生成路径、关键帧和约束线段 |
| `visualization_options.hpp` | `VisualizationOptions` | 视图类型、最大输出点数、空间范围及颜色字段 |
| `internal/color_mapping.*` | `ColorMapping` | 纯数据上色，不依赖渲染引擎 |

类名使用 `ViewBuilder` 表达“从地图快照构建展示值”的职责；只有实际拥有 GPU/窗口渲染流程时才使用 `Renderer`。应保证视图输出和 snapshot version 对应，导出耗时可以在工具/后台线程执行，不阻塞地图更新。

## 不应放入该目录

- ROS `PointCloud2`、Marker、TF 发布器：放 app 的 transport adapter。
- PCL visualizer、OpenGL/Vulkan、窗口和 UI：放独立可选应用层。
- 地图存储、ID 分配或地图拼接：分别属于 `mapping/` 和 `backend/global_mapping/`。
- 仅做 PCD/PLY 文件编码：属于 `mapping/serialization/` 或应用导出工具，不能由 visualizer 私自定义磁盘格式。

## 当前实现边界

旧 `frontend/visual/` 与 `map/visual/` 仅有未构建、无外部消费者的视觉 SLAM 占位文件，已从 Voxel-SLAM 移植工程清理。该清理不代表 Voxel-SLAM 可视化投影功能已完成；未来仅在出现实际消费者和数据契约时再创建本模块。

## 实施建议

需要可视化时先用点云/轨迹快照验证导出需求。出现多个独立 consumer 后，再从重复转换中抽出 `mapping/visualization` 公共只读投影层；这样不会让地图核心依赖具体展示工具。
