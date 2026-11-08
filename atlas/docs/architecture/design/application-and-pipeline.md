# 应用与流水线编排

以下路径以 Atlas 工程根目录为基准；为规划名，不代表已落地。

| 文件 | 主类型 | 职责 |
|---|---|---|
| `app/mapping_service.hpp/.cpp` | `MappingService` | 启停、传感器输入、状态/地图快照和输出；保持应用层稳定入口 |
| `app/sensor_message_adapter.hpp/.cpp` | `SensorMessageAdapter` | 外部消息与 Atlas sensor 类型转换；不含 SLAM 算法 |
| `atlas/frontend/trajectory_builder_output.hpp` | `OdometryOutput`、`ProcessStatus` | 前端处理结果；当前单实现直接使用 `TrajectoryBuilder` |
| `atlas/frontend/trajectory_builder.hpp/.cpp` | `TrajectoryBuilder` | 前端阶段编排、有序输入处理、模块调用和结果发布 |
| `atlas/frontend/trajectory_builder_options.hpp` | `TrajectoryBuilderOptions` | 初始化、里程计和局部建图配置的组装入口 |
| `atlas/frontend/internal/trajectory_builder_state.hpp` | `TrajectoryBuilderState` | builder 当前阶段状态、输入序号和版本；跨阶段输出使用明确定义的结果值类型 |
| `atlas/frontend/internal/trajectory_lifecycle.hpp` | `TrajectoryLifecycle`、`TrajectoryLifecycleState` | 仅在状态转换有独立规则和测试价值时保留 |

`TrajectoryBuilder` 是前端稳定入口。目标职责是编排并发布状态，不负责几何计算或因子构造；目前它仍持有多阶段地图和后台状态，属于待拆分内容。初始化、惯性估计、里程计和局部建图实现当前分别位于 `frontend/initialization/`、`frontend/inertial/`、`frontend/odometry/` 和 `frontend/local_mapping/`。应用适配器负责输入形态，不负责轨迹生命周期。当前仅有一个 `TrajectoryBuilder` 实现，因而不保留额外接口层。
