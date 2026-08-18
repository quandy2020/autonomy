# Autoviz ↔ RViz2 功能对齐清单

> 目标：Autolink 原生桌面 3D 可视化，概念与 RViz2 对齐。  
> 图例：✅ 已实现 · ⚠️ 部分 · ❌ 未实现

## Displays（默认插件）

| RViz2 Display | Autoviz | 状态 |
|---------------|------|------|
| Grid | Grid | ✅ |
| Axes | Axes | ✅ |
| TF | TF | ✅ Frames/Tree/Filter/Timeout 对齐 RViz2；轴长 = 0.2×Marker Scale |
| RobotModel | RobotModel | ✅ URDF 材质/纹理 PBR + 碰撞体线框 |
| LaserScan | LaserScan | ✅ Flat / Intensity 着色 |
| PointCloud2 | PointCloud2 | ✅ Flat / Intensity / RGB8 + Point Size |
| Map | Map | ✅ |
| Odometry | Odometry | ✅ |
| Path | Path | ✅ |
| Marker | Marker | ✅ 含 CUBE/SPHERE_LIST、TEXT 真实文字 billboard |
| MarkerArray | MarkerArray | ✅ |
| Pose | Pose | ✅ PoseStamped |
| PoseArray | PoseArray | ✅ |
| Image | Image + Image 面板 | ✅ rgb8/mono8/bgr8 |
| Camera | Camera | ✅ Image + 视锥 + 远平面纹理投影 |
| InteractiveMarkers | InteractiveMarkers | ✅ Update/Init + 控制手柄 + 菜单 |
| Wrench / Effort | Wrench + Effort | ✅ WrenchStamped + JointState 力矩 |
| GridCells | GridCells | ✅ nav_msgs/GridCells 方格线框 |
| PointStamped | PointStamped | ✅ 可配置半径十字标记 |
| Polygon | Polygon | ✅ PolygonStamped 闭合轮廓 |
| Range | Range | ✅ sensor_msgs/Range 扇形 |
| PoseWithCovariance | PoseWithCovariance | ✅ 位姿 + XY 协方差椭圆 |
| TwistStamped | TwistStamped | ✅ 线/角速度箭头 |
| CameraInfo | CameraInfo | ✅ 视锥线框 + 侧边射线 |
| DepthCloud | DepthCloud | ✅ depth + CameraInfo → 点云（32FC1/16UC1） |
| FluidPressure / Illuminance / Temperature / RelativeHumidity | 同名 | ✅ 标量伪点云着色 |
| Imu | Imu | ✅ 姿态轴 + 加速度/陀螺箭头 |
| Group | Group | ✅ 嵌套 Display + `.rviz`/`.autoviz` 层级持久化 |

## 面板

| RViz2 Panel | Autoviz | 状态 |
|-------------|------|------|
| Displays | Displays | ✅ Add 对话框、Duplicate/Rename、颜色/通道/enum/float/**path 文件选择** |
| Selection | Selection | ✅ Display 名 + 坐标，Ctrl 多选 |
| Tool Properties | Tool Properties | ✅ Topic 配置 + `.autoviz` 持久化 |
| Views | Views | ✅ 双列属性树、Zero（Z）、Focal Shape 拖拽可视化、交互实时同步 |
| Time | Time | ✅ 窗口底部全宽吸附 + Time/Wall 条 |
| Playback | —（Autoviz 扩展） | ✅ File → Open Record / 拖入 `.record` 即播（Foxglove MCAP 式） |
| TF Tree | TF Tree | ✅ 默认隐藏，Panels → Add Panel |
| Help | Help | ✅ 默认隐藏 |
| Image | Image | ✅ 默认隐藏 |
| Autolink Channels | Autolink Channels | ✅ 默认隐藏 |

## 布局 / 框架 UI

| RViz2 | Autoviz | 状态 |
|-------|------|------|
| 默认 5 面板（Displays / Selection / Tool Properties / Views / Time） | 同左，可选面板默认隐藏 | ✅ |
| 视口左右 Dock 折叠箭头 | 中央 QToolButton 折叠条 | ✅ |
| Displays → Global Options | Fixed Frame / Background / **Frame Rate** / Show Grid | ✅ |
| Displays → Status | 汇总 Ok / Warn / Error + 各行状态图标 | ✅ |
| Panels → Add New Panel | 全部内置面板（隐藏时可添加） | ✅ |
| Panels → Delete Panel | 关闭任意可见面板 | ✅ |
| Panels 各面板 toggle | 与 dock toggleViewAction 同步 | ✅ |
| Panels → 工具栏 toggle | ✅ |
| PanelDockWidget 折叠态 | 自定义标题栏 + `.autoviz` Panels 段 | ✅ |
| Hide Left/Right Dock 持久化 | `.autoviz` Window 段 | ✅ |

## 工具 Tools

| RViz2 Tool | Autoviz | 状态 |
|------------|------|------|
| Move Camera | Move Camera | ✅ |
| Select | Select | ✅ GPU 深度拾取（有硬件 GPU）+ CPU 回退 + Ctrl 多选 |
| Interact | Interact | ✅ 轴向/平面/旋转/6DOF + 菜单 + KEEP_ALIVE |
| Focus Camera | Focus Camera | ✅ 场景拾取 / 地面回退 |
| Measure | Measure | ✅ 场景拾取 / 地面回退 |
| 2D Pose Estimate | 2D Pose Estimate | ✅ 可配置 Topic |
| 2D Nav Goal | 2D Nav Goal | ✅ 可配置 Topic |
| Publish Point | Publish Point | ✅ 可配置 Topic |
| 工具栏 **+ / −** 增删 | 内置工具动态增删 + `.autoviz` Toolbar 段 | ✅ |

## ViewControllers

| RViz2 | Autoviz | 状态 |
|-------|------|------|
| Orbit | Orbit | ✅ |
| FPS | FPS | ✅ |
| TopDownOrtho | TopDownOrtho | ✅ |
| ThirdPersonFollow | ThirdPersonFollow | ✅ |
| XYOrbit | XYOrbit | ✅ 固定俯仰角透视轨道 |
| ViewControllerRegistry | 内置 6 种 + `.rviz` Class 映射 | ✅ |

## 渲染 / 框架

| 能力 | 状态 |
|------|------|
| OpenGL SceneOverlay | ✅ PBR（GGX）+ disc 点精灵（纹理 × 顶点色） |
| Ogre 可选后端 | ✅ 仅硬件 GPU 启用；llvmpipe/SwiftShader 等自动回退 OpenGL |
| GPU 深度拾取 | ✅ `glReadPixels` 深度反投影；无 GPU 时用 CPU 屏幕空间拾取 |
| RViz 风格图标 / 工具栏 | ✅ PNG 图标 + TextBesideIcon + **plus/minus** |
| 窗口布局持久化 | ✅ Dock/Geometry + HideLeft/RightDock 写入 `.autoviz` |
| 菜单栏（File / Panels / Help） | ✅ 对齐 RViz：Recent Configs、Delete Panel、Save Image、Ctrl+Q |
| 菜单/工具快捷键 | ✅ Ctrl+O/S/Shift+S/Q、F11、1–8 切换工具 |
| 插件动态加载 (pluginlib) | ⚠️ Display/Tool `AUTOVIZ_PLUGIN_PATH` + `.so` 注册 |
| Fixed Frame | ✅ Displays 树 Global Options |
| Background Color | ✅ Global Options + OpenGL/Ogre 视口 |
| .autoviz 配置持久化 | ✅ Displays + Views + **CurrentView** + Tools + Toolbar + Window |
| .rviz 配置导入 | ✅ Global / Displays / Views（含 Saved）/ Tools / Current 视角 / Panel 折叠态 |
| 截图 / 全屏 | ✅ |
| 内置回放 Seek 预览 | ✅ |
| automsgs 消息格式（无 libautonomy） | ✅ |
| 旧 autonomy.commsgs 类型名兼容 | ✅ Channel 列表归一化显示 |

## 后续里程碑（建议顺序）

1. **pluginlib** 完整插件生态（Display / Tool / ViewController）  
2. **完整 pluginlib**（ament index 自动发现）  
3. rviz_rendering **Pick ID 着色 Pass**（持久 PointCloud 对象）  
4. rviz_common **TransformationPanel** 对齐  

## 构建与运行

```bash
cmake -S src/autonomy -B build -DBUILD_AUTOVIZ=ON [-DAUTOVIZ_USE_OGRE=ON]
cmake --build build --target autoviz
./build/bin/autoviz --config autoviz/config/default.autoviz
```

依赖：`automsgs` + `autolink` + Qt6（**不链接** `libautonomy`）。

### Display 插件

设置 `AUTOVIZ_PLUGIN_PATH`（`:` 分隔目录），加载导出 `autoviz_register_displays(DisplayRegistry*)` 的 `.so`：

```cpp
AUTOVIZ_DISPLAY_PLUGIN_EXPORT(autoviz::common::DisplayRegistry* registry) {
  autoviz::common::RegisterDisplayPlugin(registry, "MyDisplay", creator, defaults);
}
```
