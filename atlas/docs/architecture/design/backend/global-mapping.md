# Backend 多会话与全局建图

## 模块边界

目标设计中，`GlobalMapping` 协调跨 session 的历史地图装载、已有图关系恢复、会话结束后的全局 BA，以及 PGO/HBA 的异步任务生命周期。当前实现中它只持有 worker 生命周期、finish/drain/join 和错误状态；`TrajectoryBuilder` 仍构造快照、执行 HBA 阶段并提交结果。历史会话图约束由 backend 图类型承载，HBA 几何优化调用 `map/` 算法。

这一区分避免当前 `GlobalMapping` 仅仅包装 PoseGraph/GTSAM、而 `Odometry` 又持有 GBA 线程和缓冲的职责错位。目标类必须拥有真实状态/并发边界；否则保留函数级组织即可，不增加 worker facade。

## 推荐目录和文件名

```text
backend/global_mapping/
├── global_mapping.hpp/.cpp               # 全局服务 API、生命周期和任务接收
├── global_mapping_options.hpp             # 会话加载、窗口大小、全局优化策略
├── global_mapping_request.hpp              # map/session version + const 输入快照
├── global_mapping_result.hpp               # 完成状态、位姿更新和诊断
├── previous_session_loader.hpp/.cpp        # 历史会话发现、完整性检查和恢复
├── mapping_session.hpp                    # 会话身份、时间和生命周期状态
└── internal/
    ├── mapping_job.hpp/.cpp                # 单次作业执行与取消点
    ├── bundle_adjustment_request.hpp       # HBA 窗口/块快照和约束参数
    ├── bundle_adjustment_result.hpp        # HBA pose updates 和分层统计
    └── session_registrar.hpp/.cpp              # 计算并验证新旧会话的初始对齐
```

若 `GlobalMapping` 可直接将任务提交给 Atlas `common::ThreadPool`，不再创建常驻 `GlobalMappingWorker`。只有需要长期持有资源、独立 drain/cancel/join 或串行 session 状态时才创建 worker owner。`previous_session_loader` 只恢复数据，不生成回环；BTC descriptor index 由 `loop_closure` 管理。

## 主要职责和文件

| 文件/类型 | 职责 | 数据所有权与边界 |
|---|---|---|
| `global_mapping.*` / `GlobalMapping` | 接收 `FinalizeSession`、历史加载和全局优化请求；协调顺序、队列和服务停止 | 唯一拥有后台请求状态；不持有 frontend 可变容器 |
| `global_mapping_options.hpp` | 历史 session 清单、加载策略、窗口/重叠配置、失败策略 | 通过应用配置构造；不存 ROS 参数对象 |
| `global_mapping_request.hpp` | source session/map version、const keyframe/voxel snapshots、所需优化能力 | 请求提交后不可变；任务无权向输入快照写入 |
| `global_mapping_result.hpp` | source version、成功/失败/取消状态、pose updates、地图版本和统计 | 结果是值/共享只读数据；由唯一提交点应用 |
| `previous_session_loader.*` | 查找历史会话、校验文件版本/完整性、加载 poses/cloud/map/PGO edges | 加载失败不部分污染活动 session/图；先临时构造再原子发布 |
| `mapping_session.hpp` / `MappingSession` | session 稳定身份、原始名称、时间戳和坐标锚点 | 元数据跨重启稳定；名称字符串不作唯一 ID；文件格式字段归 `SessionFileHeader` |
| `mapping_job.*` / `MappingJob` | 封装一次 PGO/HBA/会话收尾任务、取消检查和错误收集 | 只在任务确有独立生命周期时拆出；不再开无限线程 |
| `session_registrar.*` / `SessionRegistrar` | 新会话初始对齐估计、可信度和图约束形成 | 输出候选测量，交给 PoseGraph 验证/接收 |

## 单次会话与多会话流程

```text
LoadPreviousSessions
  -> validate metadata and map snapshots
  -> restore session-local poses, keyframes, saved graph constraints
  -> publish immutable session snapshots to LoopClosure index
  -> start current session
  -> accept verified cross-session loop constraints in PoseGraph
  -> FinalizeSession creates GlobalMappingRequest with pose and map versions
  -> optional bottom-up HBA -> top-down graph/map correction
  -> validate and commit GlobalMappingResult
  -> persist one consistent final session/map version
```

前端不等待历史地图索引或全局 HBA；若首次会话对齐必须在融合前完成，由应用生命周期显式等待 `LoadPreviousSessions` 结果，而不是隐式阻塞每个 sensor callback。历史会话恢复需要原子性：目录缺项、版本不兼容或边文件损坏时返回结构化错误，不发布半加载 session。

## 后台并发和线程池

- 用现有 `common::ThreadPool` 复用线程；GlobalMapping 只定义 SLAM 任务和生命周期，不再实现第二套通用线程池。
- 当前 worker 用条件变量通知和关键帧游标表示待处理工作；不为同一串行阶段链再套一层请求队列。若未来出现可独立提交的约束处理、session finalize 或可取消优化作业，再按实际优先级和过载策略引入有界队列。
- HBA 可以在内部按独立块并行，但应传入任务 executor/并行策略，禁止每帧或每次调用无上限创建/join线程。
- 请求保存 `GraphVersion`、`MapVersion`、`SessionId`。完成时任何一个源版本过期都必须拒绝提交或按明确重基准流程重算。
- 局部 HBA、submap HBA 和 residual polish 的请求/结果都携带 pose correction 与 map state 双版本；结果必须回显两个版本，并在提交临界区同时匹配当前版本、请求 ID、会话和关键帧身份。两种版本不可互相替代：pose version 防止旧坐标解覆盖新解，map version 防止输入点云/约束变化后提交旧计算。TopDown PGO 仍使用专用多 session 快照路径；提交时也检查 pose/map 双版本、当前 session、session 容器形状、scan 有限性和关键帧 ID/session 身份。该路径暂未迁入公共 request/result 类型，因为通用窗口请求不能精确表达按 session 分组的 scan `v6` 与关键帧状态。
- 历史 session 由 `PreviousSessionLoader` 加载时持有 map mutex，成功发布后递增 map state version。该加载发生在前端接收新 scan 前；第一个后台任务因此能读取带有明确版本的历史地图状态。
- `PoseGraph::ApplyOptimizationResult` 校验图版本并提交优化位姿；提交逻辑直接归图状态 owner。地图几何校正由地图模块消费 PoseGraph 发布的修正结果。
- Shutdown 顺序固定：拒绝新请求 -> drain/cancel 已接收任务 -> 等待任务完成 -> 提交/保存最后一致版本 -> join 线程池所有权方。
- 不在任务执行期间持有 PoseGraph、VoxelMap 或活动窗口的写锁。

### 当前阶段状态与调度粒度

当前 backend 全局映射只有一个串行 worker，阶段顺序固定为局部窗口 HBA → submap HBA → residual polish → TopDown PGO；后续阶段依赖前一阶段提交的 pose/edge 结果。各体素块的独立计算由 HBA 内部线程池并行。`GlobalMappingStage` 对外报告当前阶段、drain、stop 或失败状态。`GlobalMappingResult::MatchesRequest` 负责在 backend 消息契约层校验 request identity、版本、关键帧索引、位姿有限性与 edge 结构；快照创建和具体目标状态写回仍由 `TrajectoryBuilder` 持有，属于尚未完成的 backend 所有权迁移项。

因此当前不实现通用 task DAG：它不会提高现有串行后端阶段的并行度，反而会额外引入依赖图生命周期、失败传播和取消语义。只有出现可独立执行的 backend 作业且有可测吞吐收益时，再把这些作业纳入有界队列/DAG；当前明确缺少的是把 request/result 和提交点从 `TrajectoryBuilder` 收敛至 `GlobalMapping`，不是缺少通用调度框架。

## 文件格式和持久化职责

文件编码格式属于 `mapping/serialization/`，session 加载次序和完整性策略属于 `GlobalMapping`。不把文本/二进制读写细节塞进 `global_mapping.cpp`，也不让 serializer 读取 ROS 参数。历史存储至少带 schema/version、session ID、坐标系约定、关键帧数和图约束版本；旧格式升级由明确 migration reader 完成，不靠文件名推断兼容性。

## 原始源码与 Atlas 当前实现

- 原始代码：`voxelslam.cpp` 中 `previous_map_names()`、`previous_map_read()`、`pgo_edges_io()`、`thd_loop_closure()` 的 session merge/finalize，以及 top-down HBA；`map/hierarchical_bundle_adjustment.*` 承载 HBA 算法。
- Atlas 当前代码：`backend/global_mapping/global_mapping.*`、`previous_session_loader.*`、`pgo_edges.*`、`pose_graph_optimization.*`，以及 `TrajectoryBuilder` 内 GBA worker/快照和 `map/hierarchical_bundle_adjustment.*`。
- 目标收敛：`PreviousSessionLoader` 返回结构化结果并由全局会话 owner 原子发布；PGO edge 类型/图连通状态迁给 `PoseGraph`；`GlobalMapping` 成为真实异步任务 owner；HBA 计算仍留在地图算法模块。

## 命名和不应新增的文件

- 使用 `previous_session_loader`，不再用含义宽泛的 `previous_map`；使用 `global_mapping_request/result` 区分跨线程消息。
- 不新增 `global_mapping_manager`、`gba_manager` 或同时存在的 `GlobalMappingWorker`/`GlobalMappingExecutor` 双 facade。
- 不把 `SessionRepository` 与 `PreviousSessionLoader` 重复实现为两个历史地图 owner；若持久化接口已覆盖 session 读写，由 loader 组合调用它。
- 不把 `GlobalBundleAdjustment` 数学类从 `mapping/` 搬到 backend；backend 只组装快照、调度并提交结果。
