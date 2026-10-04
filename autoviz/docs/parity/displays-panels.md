# Display / Panel / Tool / View

## Displays

| RViz2 | Autoviz | 状态 |
|-------|---------|------|
| Grid | Grid | ✅ |
| Axes | Axes | ✅ |
| TF | TF | ✅ Frames / Tree / Filter / Timeout |
| RobotModel | RobotModel | ✅ URDF / 碰撞线框 |
| LaserScan | LaserScan | ✅ |
| PointCloud2 | PointCloud2 | ✅ Flat / Intensity / RGB |
| Map | Map | ✅ |
| Odometry | Odometry | ✅ |
| Path | Path | ✅ |
| Marker / MarkerArray | 同名 | ✅ |
| Pose / PoseArray | 同名 | ✅ |
| Image | Image + Image dock | ✅ |
| Camera | Camera | ✅ 视锥 + 投影 |
| InteractiveMarkers | InteractiveMarkers | ✅ |
| Wrench / Effort | 同名 | ✅ |
| GridCells / PointStamped / Polygon / Range | 同名 | ✅ |
| PoseWithCovariance / TwistStamped / AccelStamped | 同名 | ✅ |
| CameraInfo / DepthCloud | 同名 | ✅ |
| Imu / 标量传感器 | 同名 | ✅ |
| Group | Group | ✅ 嵌套与配置持久化 |

## 面板

| RViz2 | Autoviz | 状态 |
|-------|---------|------|
| Displays | Displays | ✅ |
| Views | Views | ✅ |
| Selection | Selection | ✅ |
| Tool Properties | 工具属性 | ✅ |
| Time | Time / Playback | ✅ `.record` |
| Transformation | Transformation | ✅ |

## 工具

| RViz2 | Autoviz | 状态 |
|-------|---------|------|
| Interact / MoveCamera / FocusCamera | 同名概念 | ✅ |
| Measure | Measure | ✅ |
| Set Goal / Set Initial Pose | NavGoal / 初值 | ✅ |
| Publish Point 等 | 视实现 | ⚠️ 按面板扩展 |

## 视图

| RViz2 | Autoviz | 状态 |
|-------|---------|------|
| Orbit / FPS / TopDownOrtho / XYOrbit | 同名或别名映射 | ✅ |

## 相关文档

- [框架对照](framework.md) · [使用](../guide/usage.md)
