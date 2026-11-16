# 输入输出与运行配置

## 自动启动链路

| 入口 | 配置 | 当前作用 |
|---|---|---|
| `autolink launch start perception.launch` | [perception.dag](../dag/perception.dag) → [perception.pb.txt](../conf/perception.pb.txt) 的 `engine` 段 | YOLO 人员轨迹与 MoGe 深度，语义默认关闭 |
| `autolink launch start behaviors.launch` | [behaviors.dag](../dag/behaviors.dag) → [follow.pb.txt](../conf/follow.pb.txt) | 订阅感知、相机内参、里程计并输出跟随结果 |
| `autonomy.perception_object <options.pb.txt>` | `ObjectInferenceOptions` | 独立 C++ 请求响应服务，服务名由配置指定 |
| `autonomy.perception` | [perception_exploration.pb.txt](../conf/perception_exploration.pb.txt) 等 | 现有探索入口 |

部署时确保 AutoLink 能解析 launch、DAG、配置与组件库，且模型路径指向真实导出。
更换 Follow 输入话题时同时更新 DAG reader 与 Follow 配置。

## AutoLink 话题

| 方向 | 默认话题 | AutoMsgs 类型或语义 |
|---|---|---|
| 感知输入 | `/camera/color/image_raw` | `sensor_msgs.Image`，`rgb8` 或 `bgr8`，含源帧头 |
| 感知输出 → Follow | `/perception/base/tracks` | `vision_msgs.Detection2DArray`，源图像像素框及人员 ID |
| 感知输出 → Follow | `/perception/base/depth` | `sensor_msgs.Image`，`32FC1` 米制深度，无效值 NaN |
| 可选感知输出 → Follow | `/perception/base/semantic` | `sensor_msgs.Image`，`mono8` 类别索引，255 表示未知 |
| Follow 输入 | `/camera/color/camera_info` | `sensor_msgs.CameraInfo`，图像几何和内参 |
| Follow 输入 | `/odom` | `nav_msgs.Odometry` |
| Follow 输入 | `/perception/follow/select` | `std_msgs.String`，指定轨迹 ID |
| Follow 输出 | `/perception/follow/target` | `geometry_msgs.PoseStamped` |
| Follow 输出 | `/perception/follow/path` | `nav_msgs.Path` |
| Follow 输出 | `/perception/follow/grid` | `map_msgs.GridMap`，含高度、障碍、可通行和语义障碍层 |

检测、分类、旋转框、人体关键点及实例分割按 task 的 `topic` 配置。
检测类框走 `Detection2DArray`，分类走 `Classification`。
启用 `keypoints_topic` 或 `instance_masks_topic` 时，补充消息按检测顺序对齐。

## 对象推理服务

权威定义见 [perception.proto](../proto/perception.proto)。请求与响应分别是
`ObjectInferenceRequest` 和 `ObjectInferenceResponse`。响应携带源消息头和错误字段。
模型阶段输入绑定到请求张量名或前序阶段的 `stage.tensor`。
张量字节连续排列，按运行主机字节序。尺寸和类型以已加载导出为准。

| 模式 | 请求输入 | 输出与限制 |
|---|---|---|
| 通用图推理 | 源帧头、已准备的命名张量 | 最后一阶段选定的输出张量。SAM 2 和 FlashOcc 当前使用此入口 |
| RRCE 原尺寸栅格 | `occupancy_grid`，单阶段 RRCE | 模型输入 `float32 [1,1,H,W]`，0–100 映射到 0–1，未知为 -1。返回地图系轮廓和矩形 |
| RRCE 预处理栅格 | 已处理张量与 `room_map`，包含六参数仿射、分辨率、原点和航向 | 末阶段提供 `vertices [V,2]` 和 int32 `vertex_counts [R]` |
| Boxer 解码 | 准备好的张量及与 `boxer_decode.frame_id` 相同的请求帧 | `Detection3DArray`。导出提供 `float32 [N,12]`，列为中心、尺寸、四元数、分数和类别索引 |
| Boxer 房间归属 | 同帧同地图版本的 `room_snapshot`、稳定 room ID 与原始轮廓 | `room_memberships` 以 `detections_3d` 中的零基索引关联 room ID。边界、重叠及房间外保持未归属 |

Boxer 三维框解码要求中心和尺寸以米表示，四元数顺序 `qx,qy,qz,qw`。
框已经处于声明的坐标系，服务按请求头核对坐标系。相机系输出须先转换到地图系才能用于房间归属。
