# Foxglove Bridge

`autonomy_foxglove_bridge` 是一个 C++ 原生可视化 bridge：

- 复用 `autolink` 的 channel 自动发现能力；
- 通过 registry 统一管理可视化消息、目标 schema、面板分类和转换策略；
- 对可直接兼容的消息做 protobuf schema / descriptor 规范化直通；
- 对必须转换的消息走 adapter（如 `MultiEchoLaserScan -> LaserScan`、`TransformStampeds -> TFMessage`、`PointCloud(2)` 重打包等）；
- 使用 `foxglove-sdk` 的本地 WebSocket server 对外暴露 topic/schema/live stream。

## 目录

```text
visualization/
├── foxglove_main.cpp
├── visualization_server.{hpp,cpp}
├── common/
│   ├── bridge_filter.hpp
│   ├── channel_snapshot.hpp
│   └── visualization_schema_registry.{hpp,cpp}
├── schema/
│   └── schema_compatibility.{hpp,cpp}
├── transport/
│   ├── autolink_discovery.{hpp,cpp}
│   ├── channel_forwarder.{hpp,cpp}
│   └── foxglove_bridge_server.{hpp,cpp}
└── adapters/
    └── message_adapter.{hpp,cpp}
```

- `foxglove_main.cpp`：命令行入口，对齐 `task_main.cpp`
- `visualization_server.*`：顶层装配与生命周期管理，对齐 `task_server.*`
- `common/`：共享模型、过滤规则、可视化支持矩阵
- `schema/`：protobuf descriptor 规范化
- `transport/`：autolink topic 发现、reader 转发、Foxglove server 发布
- `adapters/`：payload 转换（LaserScan、TFMessage、PointCloud2 等）

## 构建依赖

Bridge 依赖 `foxglove-sdk` 的 CMake package。

根目录 `CMakeLists.txt` 会执行：

```cmake
find_package(foxglove-sdk CONFIG QUIET)
```

如果未安装，bridge target 会被跳过，不影响其他模块构建。

## Foxglove 支持数据

registry 当前登记 **38** 种 autolink 消息类型。直接运行 `./bin/autonomy_foxglove_bridge`（不传 `--message_type_allowlist`）时，会转发下表全部已登记类型；仅当显式传入 allowlist 时才做前缀/精确过滤。

| 源消息（autolink `message_type` 后缀） | 目标 Schema | Foxglove 面板 | 策略 |
|---|---|---|---|
| `visualization_msgs.Marker` | `foxglove.SceneUpdate` | 3D | payload 转换 |
| `visualization_msgs.MarkerArray` | `foxglove.SceneUpdate` | 3D | payload 转换 |
| `geometry_msgs.PoseStamped` | `geometry_msgs.PoseStamped` | 3D | schema 直通 |
| `geometry_msgs.PoseArray` | `geometry_msgs.PoseArray` | 3D | schema 直通 |
| `geometry_msgs.PointStamped` | `geometry_msgs.PointStamped` | 3D | schema 直通 |
| `geometry_msgs.PolygonStamped` | `geometry_msgs.PolygonStamped` | 3D | schema 直通 |
| `geometry_msgs.TransformStamped` | `geometry_msgs.TransformStamped` | 3D / TF | schema 直通 |
| `geometry_msgs.TransformStampeds` | `foxglove.FrameTransforms` | 3D / TF | payload 转换 |
| `tf2_msgs.TFMessage` | `foxglove.FrameTransforms` | 3D / TF | payload 转换 |
| `planning_msgs.Path` | `nav_msgs.Path` | 3D | schema 直通 |
| `planning_msgs.Odometry` | `nav_msgs.Odometry` | 3D | schema 直通 |
| `sensor_msgs.LaserScan` | `foxglove.LaserScan` | 3D | payload 转换 |
| `sensor_msgs.MultiEchoLaserScan` | `foxglove.LaserScan` | 3D | payload 转换 |
| `sensor_msgs.PointCloud2` | `foxglove.PointCloud` | 3D | payload 转换 |
| `sensor_msgs.PointCloud` | `foxglove.PointCloud` | 3D | payload 转换 |
| `sensor_msgs.Range` | `sensor_msgs.Range` | 3D | schema 直通 |
| `vision_msgs.BoundingBox3D` | `vision_msgs.BoundingBox3D` | 3D | schema 直通 |
| `vision_msgs.BoundingBox3DArray` | `vision_msgs.BoundingBox3DArray` | 3D | schema 直通 |
| `vision_msgs.Detection3D` | `vision_msgs.Detection3D` | 3D | schema 直通 |
| `vision_msgs.Detection3DArray` | `vision_msgs.Detection3DArray` | 3D | schema 直通 |
| `map_msgs.GridCells` | `map_msgs.GridCells` | 3D | Raw/Plot |
| `map_msgs.VoxelGrid` | `map_msgs.VoxelGrid` | 3D | Raw/Plot |
| `sensor_msgs.Image` | `foxglove.RawImage` | Image | payload 转换 |
| `sensor_msgs.CompressedImage` | `sensor_msgs.CompressedImage` | Image | schema 直通 |
| `sensor_msgs.CameraInfo` | `sensor_msgs.CameraInfo` | Image | schema 直通 |
| `vision_msgs.BoundingBox2D` | `vision_msgs.BoundingBox2D` | Image | schema 直通 |
| `vision_msgs.BoundingBox2DArray` | `vision_msgs.BoundingBox2DArray` | Image | schema 直通 |
| `vision_msgs.Detection2D` | `vision_msgs.Detection2D` | Image | schema 直通 |
| `vision_msgs.Detection2DArray` | `vision_msgs.Detection2DArray` | Image | schema 直通 |
| `sensor_msgs.NavSatFix` | `sensor_msgs.NavSatFix` | Map | schema 直通 |
| `map_msgs.OccupancyGrid` | `map_msgs.OccupancyGrid` | Map | Raw/Plot |
| `map_msgs.OccupancyGridUpdate` | `map_msgs.OccupancyGridUpdate` | Map | Raw/Plot |
| `map_msgs.Costmap` | `map_msgs.Costmap` | Map | Raw/Plot |
| `map_msgs.CostmapUpdate` | `map_msgs.CostmapUpdate` | Map | Raw/Plot |
| `map_msgs.GridMap` | `map_msgs.GridMap` | Map | Raw/Plot |
| `sensor_msgs.Imu` | `sensor_msgs.Imu` | Raw / Plot | schema 直通 |
| `sensor_msgs.BatteryState` | `sensor_msgs.BatteryState` | Raw / Plot | schema 直通 |
| `diagnostic_msgs.DiagnosticArray` | `diagnostic_msgs.DiagnosticArray` | Raw / Plot | schema 直通 |
| `vehicle_msgs.RobotState` | `vehicle_msgs.RobotState` | Raw / Plot | schema 直通 |

说明：

- 完整源类型名为 `automsgs.msgs.<上表后缀>`，例如 `automsgs.msgs.sensor_msgs.LaserScan`。
- **schema 直通**：规范化 descriptor 后直接转发 payload。
- **payload 转换**：adapter 改写 payload 和/或目标 descriptor（如 `uint32[] data` → `bytes data`）。
- **Raw/Plot**：schema 可解析，但 Foxglove 3D/Image/Map 面板通常不能直接渲染，适合 Raw Messages / Plot 面板。
- 转发某一 proto 包下全部已登记类型：`--message_type_allowlist=automsgs.msgs.sensor_msgs`（前缀匹配）。

## Schema 兼容策略

bridge 使用 `encoding=protobuf`，因此 `schema.name` 必须和 descriptor 中的 protobuf 类型名一致，例如 `sensor_msgs.LaserScan`，不是 ROS 文本形式的 `sensor_msgs/LaserScan`。

直接兼容的类型会统一做下面几层规范化：

- `schema.name`
- `FileDescriptorProto.package`
- `field.type_name`
- `dependency` 文件名

当前显式映射：

- `automsgs.msgs.visualization_msgs.Marker` -> `visualization_msgs.Marker`
- `automsgs.msgs.visualization_msgs.MarkerArray` -> `visualization_msgs.MarkerArray`
- `automsgs.msgs.geometry_msgs.PoseStamped` -> `geometry_msgs.PoseStamped`
- `automsgs.msgs.planning_msgs.Path` -> `nav_msgs.Path`
- `automsgs.msgs.planning_msgs.Odometry` -> `nav_msgs.Odometry`
- `automsgs.msgs.sensor_msgs.MultiEchoLaserScan` -> `sensor_msgs.LaserScan`
- `automsgs.msgs.geometry_msgs.TransformStampeds` -> `foxglove.FrameTransforms`
- `automsgs.msgs.tf2_msgs.TFMessage` -> `foxglove.FrameTransforms`
- `automsgs.msgs.sensor_msgs.PointCloud2` -> `sensor_msgs.PointCloud2`
- `automsgs.msgs.sensor_msgs.PointCloud` -> `sensor_msgs.PointCloud2`

## LaserScan 转换策略

`MultiEchoLaserScan` 不是 Foxglove 3D 直接支持的 schema，因此 bridge 会在 reader 回调里转换成 `sensor_msgs.LaserScan`：

- `header`、`angle_*`、`time_increment`、`scan_time`、`range_*` 直接映射；
- `ranges[i].echoes` 取第一个 echo 作为 `LaserScan.ranges[i]`；
- `intensities[i].echoes` 取第一个 echo 作为 `LaserScan.intensities[i]`；
- 空 echo 写入 `NaN`，明确表示该 beam 没有可用单回波值。

## TF / FrameTransforms 转换策略

Foxglove 在 **protobuf** 编码下不识别 `tf2_msgs/TFMessage`，3D 面板只接受 **`foxglove.FrameTransforms`**。因此 bridge 会把 autolink 上的 TF 消息转换成 Foxglove 原生格式：

- 源类型：`TransformStampeds` 或 `tf2_msgs.TFMessage`
- 目标 schema：`foxglove.FrameTransforms`
- 每条 `TransformStamped` 映射为 `parent_frame_id` / `child_frame_id` / translation / rotation
- topic 仍使用 `/tf`（Foxglove 默认识别）

## PointCloud / PointCloud2 转换策略

Foxglove 3D 依赖标准 `sensor_msgs.PointCloud2`，其中 `data` 必须是 `bytes` 二进制布局：

- `PointCloud2`：保留 `fields/point_step/row_step` 等元数据，把 proto 里 `repeated uint32 data` 按单字节重打包成 `bytes data`；
- `PointCloud`：把 `PointXYZIT` 列表编码成 `x/y/z/intensity` 四个 float32 字段的标准 `PointCloud2`；
- `Range`：schema 与 ROS 兼容，3D 面板可直接显示测距弧。

## 3D 面板配置

Foxglove 在 **protobuf** 编码下，3D 面板只识别 Foxglove 原生 schema（与 TF 相同）。bridge 会将常用 3D 消息转换后再发布：

| autolink 源类型 | Foxglove schema |
|---|---|
| `tf2_msgs.TFMessage` | `foxglove.FrameTransforms` |
| `sensor_msgs.LaserScan` | `foxglove.LaserScan` |
| `visualization_msgs.Marker` | `foxglove.SceneUpdate` |
| `sensor_msgs.PointCloud2` | `foxglove.PointCloud` |

### 用 fakedata 快速验证

终端 1（bridge）：

```bash
./bin/autonomy_foxglove_bridge
```

终端 2（假数据，TF 默认在 `/tf`）：

```bash
./bin/autonomy_foxglove_fakedata
```

Foxglove 连接 `ws://127.0.0.1:8765` 后，在 **3D 面板** 中：

| 设置项 | 值 |
|---|---|
| Fixed frame | `map` |
| Topics → `/tf` | 开启 |
| Topics → `/fake/marker` | 开启（蓝色球 + 圆环路径） |
| Topics → `/fake/scan` | 开启（激光扇形） |
| Topics → `/fake/point_cloud2` | 可选（点云） |

应能看到：沿圆形轨迹移动的 Marker、LaserScan 扫描线、TF 树 `map → odom → base_link → laser`。

### 用 record 回放

`.record` 里若**没有 TF**（如 `echoes_1` + `imu`），需把 Fixed frame 设为消息里的 `frame_id`（例如 `horizontal_laser_link`），并只开 `/echoes_1` 等 laser topic。

## 运行

```bash
autonomy_foxglove_bridge \
  --foxglove_host=0.0.0.0 \
  --foxglove_port=8765 \
  --discovery_poll_interval_ms=1000 \
  --time_broadcast_interval_ms=100 \
  --channel_allowlist= \
  --message_type_allowlist=
```

说明：

- `--channel_allowlist` 为空时表示不过滤 channel 名；
- `--message_type_allowlist` 为空时表示转发 registry 内全部已登记类型；
- allowlist 每一项都支持“完整类型名”或“前缀”匹配。

## 数据流

1. bridge 进程启动 `autolink` node 与 Foxglove WebSocket server；
2. 周期扫描现有 writer channel；
3. 对每个候选 channel 读取 `message_type` 与 `proto_desc`；
4. 用 `SchemaCompatibility` 判断该类型是“直通”还是“需转换”；
5. 直通类型把 `ProtoDesc` 规范化后注册成目标 schema；
6. 需 payload 转换的类型由 `MessageAdapter` 改写后再发布（如 `MultiEchoLaserScan`、`TransformStampeds`、`PointCloud(2)`）。

## 验证路径

建议按下面顺序做端到端验证：

1. 运行 `autolink_recorder play -f <record>` 回放现有 `.record`；
2. 启动 `autonomy_foxglove_bridge`；
3. 在 Foxglove 中连接 `ws://127.0.0.1:8765`；
4. 先验证 `Marker`、`MarkerArray`、`Path`、`PoseStamped`、`Odometry` 是否出现在 3D topic 列表；
5. 再验证 `echoes_1` 是否以 `sensor_msgs.LaserScan` 身份出现；
6. 观察 writer 上下线时 topic 是否稳定保留 schema，并在再次上线后恢复数据流。

## 当前边界

- `planning_msgs.Path` / `planning_msgs.Odometry` 当前假设字段布局已与 `nav_msgs.*` 足够兼容，若后续发现 3D 解析异常，需要继续下沉到 payload 转换层；
- `MultiEchoLaserScan` 压缩成单回波会丢失额外 echo 信息；
- `OccupancyGrid`、`GridMap`、`VoxelGrid`、`Costmap` 等地图类消息当前优先作为 Raw/Plot 数据源桥接；若要在 3D/Map 面板里稳定渲染，需要继续实现 `foxglove.Grid` / `SceneUpdate` 级 payload 转换；
- `PointCloud` 当前只导出 `x/y/z/intensity` 四个字段，不会保留 `PointXYZIT.timestamp` 与 `channels` 附加通道；
