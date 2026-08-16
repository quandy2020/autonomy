# SensorService 设计 — 快照 / 参数 / 录制上传

日期：2026-08-14  
状态：已批准  
范围：`automsgs/proto/rpcs/sensor.proto`、`common.proto`（传感器码）、`rpcs/README.md`

## 1. 目标

第三方可：列举传感器、取最新样本、键值配置与机器人侧持久化、会话式录制/导出/上传。风格对齐 `navigation.proto`（unary 控制 + 单端流会话、`active`/`final`），避免按类型重复 Get*。

## 2. 已确认决策

| 项 | 选择 |
|---|---|
| 序列化 | 配置持久化 **+** 数据录制/导出/上传 |
| 采集形态 | 混合：unary `GetSample` + 会话 `Record` |
| 参数 | 键值 `name`/`value`（string） |
| 栅格 | **不在** Sensor；地图走 `mapping` |
| 类型化 Get* | **删除**（由 `GetSample` 的 oneof 替代） |

## 3. 服务面

```text
ListSensors                              → unary
GetSample                                → unary
GetParameters / SetParameters            → unary
SaveParameters / LoadParameters          → unary
Record                                   → stream RecordResponse
CancelRecord                             → unary
GetRecordStatus                          → unary（嵌入 RecordResponse）
```

无 Pause/Resume；无双端流。

## 4. 约定

- **GetSample：** `sensor_id` + 可选 `prefer_compressed`；载荷 `oneof`（Image / CompressedImage / LaserScan / PointCloud2 / Imu）。
- **Parameter：** `string name` + `string value`；`GetParameters.names` 空 = 全部。
- **Save/Load：** `sensor_id` 空 = 全部传感器配置。
- **Record：** `repeated sensor_ids`、目标 `uri`（本地路径或远程 URI）、可选时限/大小；流含 `state` / 进度 / `uri` / `active` / `final`。
- **关流：** 成功 / FAILED / Cancel → `final=true`；录制中 `final=false`。
- **忙：** 已有 Record → 新 Record 首帧 `CODE_SENSOR_BUSY` 并 `final=true`。

## 5. Code

保留 700–702；新增 `CODE_SENSOR_BUSY = 703`、`CODE_SENSOR_INVALID_PARAMETER = 704`、`CODE_SENSOR_SAVE_FAILED = 705`、`CODE_SENSOR_LOAD_FAILED = 706`、`CODE_SENSOR_RECORD_FAILED = 707`、`CODE_SENSOR_CANCELLED = 708`。

## 6. 非目标

实时多路订阅总线、按厂商强类型曝光参数、OccupancyGrid 快照、与 rosbag 格式绑定。
