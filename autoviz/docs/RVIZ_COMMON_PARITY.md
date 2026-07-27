# Autoviz ↔ rviz_common / rviz_default_plugins 对齐

> 参考：  
> - `/home/quandy/workspace/github/ros/rviz/rviz_common`  
> - `/home/quandy/workspace/github/ros/rviz/rviz_default_plugins`

## rviz_common 框架对照

| rviz_common | Autoviz (`autoviz/common`) | 状态 |
|-------------|----------------------|------|
| `VisualizationFrame` | `ui/visualization_frame` | ✅ |
| `VisualizationManager` | `VisualizationManager` | ✅ Display 调度 + Global Options |
| `Display` / `DisplayGroup` | `display/` + 嵌套 Group | ✅ |
| `DisplayContext` | `display_context.hpp` | ✅ queueRender / frameCount / 子管理器指针 |
| `FrameManager` | `frame_manager.hpp` | ✅ Fixed Frame + TF lookup + SyncMode |
| `ViewManager` | `view_manager.hpp` | ✅ Saved / Current + Registry 类型 |
| `SelectionManager` | `selection_manager.hpp` | ✅ 中央选择态 + focusOnSelection |
| `Property` 树 | `DisplayPropertyMap` + Displays 面板 | ✅ |
| `ToolManager` | `ToolManager` + `ToolRegistry` | ✅ 8 工具 + 动态加载 |
| `ViewControllerRegistry` | `view_controller_registry.hpp` | ✅ |
| `Config` + YAML | `SessionConfig` + `SessionConfigIO` | ✅ `.autoviz` / `.rviz` |
| `DisplayFactory` | `DisplayFactory` → `DisplayRegistry` | ✅ |
| `PluginInfo` | `plugin_info.hpp` | ✅ 元数据结构 |
| pluginlib 工厂 | `DisplayRegistry` + `ToolRegistry` + `ViewControllerRegistry` + `AUTOVIZ_PLUGIN_PATH` | ✅ |
| Config 树 | `config.hpp` + `YamlConfigReader`/`Writer` + `config_session` | ✅ 原生 `.autoviz` |
| GPU Pick | `PickRegistry` + `HandlerManager` + pick pass FBO | ✅ OpenGL + Ogre（`GlPickFramebuffer`） |
| Display Config | `Display::load/save(Config)` + `loadFromConfig/saveToConfig` | ✅ Group 递归 Children |

## rviz_default_plugins Display（32 项）

| RViz Display | Autoviz | 状态 |
|--------------|------|------|
| Grid / Axes / TF / RobotModel | 同名 | ✅ |
| LaserScan / PointCloud2 / Map / Path / Odometry | 同名 | ✅ |
| Marker / MarkerArray / Pose / PoseArray | 同名 | ✅ |
| Image / Camera / InteractiveMarkers | 同名 | ✅ |
| Wrench / Effort | 同名 | ✅ Ogre：`WrenchVisual` / `ScrewVisual` / `CovarianceVisual` |
| GridCells / PointStamped / Polygon / Range | 同名 | ✅ |
| PoseWithCovariance / TwistStamped / AccelStamped | 同名 | ✅ Ogre 视觉对齐 rviz |
| CameraInfo / DepthCloud | 同名 | ✅ |
| PointCloud (legacy) | PointCloud → PointCloud2Display | ✅ 注册别名 + 导入映射 |
| FluidPressure / Illuminance / Temperature / RelativeHumidity | 同名 | ✅ |
| Imu | Imu | ✅ |
| Group | Group | ✅ |

## rviz_default_plugins Tool（8 项）

| RViz Tool | Autoviz ID | 状态 |
|-----------|---------|------|
| MoveCamera | MoveCamera | ✅ |
| Select | Select | ✅ → `SelectionManager` |
| Interact | Interact | ✅ |
| FocusCamera | FocusCamera | ✅ |
| Measure | Measure | ✅ |
| SetInitialPose | PoseEstimate | ✅ |
| SetGoal | NavGoal | ✅ |
| PublishPoint | PublishPoint | ✅ |

## ViewController（6 项）

| RViz | Autoviz | 状态 |
|------|------|------|
| Orbit / XYOrbit / TopDownOrtho / ThirdPersonFollow / FPS | 同名 | ✅ |
| FrameAligned | → FPS | ⚠️ 导入映射 |
| TopDown | TopDown | 扩展 |

## Panel

| RViz Panel | Autoviz | 状态 |
|------------|------|------|
| Displays / Selection / Tool Properties / Views / Time | 同名 | ✅ |
| Help / TF Tree / Image / Playback / Channels | 同名或扩展 | ✅ |
| Transformation | `TransformationPanel` + `TransformationManager` | ✅ |

## `autoviz/common` 模块索引

| 文件 | 对应 rviz_common |
|------|------------------|
| `visualization_manager.*` | `VisualizationManager` |
| `display_context.*` | `DisplayContext` |
| `frame_manager.*` | `FrameManager` |
| `view_manager.*` | `ViewManager` |
| `selection_manager.*` | `SelectionManager` |
| `view_controller_registry.*` | ViewController 工厂 + pluginlib |
| `view_controller_plugin.hpp` | ViewController 插件导出宏 |
| `transformation_manager.*` | `TransformationManager` |
| `frame_transformer.*` | `FrameTransformer` 接口 |
| `transformer_plugin.hpp` | Transformer 插件导出宏 |
| `config.*` | rviz_common `Config` |
| `yaml_config_reader.*` / `yaml_config_writer.*` | YAML IO |
| `config_session.*` | SessionConfig ↔ Config 桥接 |
| `selection_handler.*` | `SelectionHandler` + `HandlerManager` |
| `display_registry.*` / `display_factory.*` | `DisplayFactory` |
| `tool_registry.*` / `tool_manager.*` | `ToolManager` |
| `session_config.*` | `Config` + YAML reader/writer |
| `display_property.*` | Property 子集 |
| `plugin_info.hpp` | pluginlib 元数据 |

## 后续

1. ament index 自动插件发现
2. Ogre 路径下 Pick shader 与 Ogre GL context 兼容性实测（无头/CI）
3. Display 属性树与 rviz `Property` 子树完全对齐
