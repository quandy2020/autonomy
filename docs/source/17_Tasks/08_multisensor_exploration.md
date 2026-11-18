# 多传感器室内探索任务

探索位于 `autonomy/task/exploration/`，是任务策略。全局路线由
`autonomy/planning/planner/vista/` 提供，局部控制接入 PolicyController。
保留 TARE 的全局区域访问顺序和局部覆盖思路，以
`HierarchicalCoverageStrategy` 表达职责，不创建 `tare/` 目录。

## 所有权与目录职责

`ExplorationTask` 负责原生任务生命周期及身份化输入。
`ExplorationSession` 是状态、观测账本、完成证据和异步阶段的唯一写入者。
策略工作线程对不可变输入进行选择/适配；迟到结果需通过请求身份和依赖
版本检查才能被 Session 接纳。

- `strategy/`：分层覆盖选择和可恢复的候选遍历。
- `frontier/`：边界提取、公平重访和暂时不可达记录。
- `coverage/`：覆盖账本、增益、局部覆盖路线及完成判据。
- `viewpoint/`、`observation/`：位置/yaw 候选及真实传感器可视模型。
- `region/`、`routing/`：区域管理、历史通行关系和访问顺序优化。
- `integration/`：读取地图、查询 Vista、目标位姿适配和导航派发。
- `plugins/`：原生 BT 动作、条件及阶段驱动。

地图所有权留在已有 map 模块；PlannerServer 持有一份增量 Vista 图。
BT 与自动驱动是可选择的 Session 驱动方式，不是两个同时发送运动目标的
执行者。NavigationTask 的 invocation gate 约束本次任务的动作派发。

## 完整数据流

配置与任务启动 → 原生 RGB-D/LiDAR 观测入口 → 采集时刻定位及原子数据包
→ 有限输入前缀处理 → 实测覆盖确认 → 原有地图融合回执与读取副本
→ Vista 增量更新 → 全局区域排序/局部覆盖选择 → 观察目标位姿适配
→ 原生导航派发 → 到达与实际视角确认 → 重选或完成复核 → 返回与地图保存。

`SelectObservationPose` 选择观察意图；`AdaptObservationPose` 调整位置和
yaw，同时保留该意图的必需观测。到达某个替代位置不等于观测意图已完成。
目标区域未知时应获取实际观测，不能因为信息缺失就宣称不可达或完成。
二维路线模型不能代替三维局部执行安全。

## BT 阶段

当前探索节点包括 `SensorReady`、`UpdateExploration`、
`SelectObservationPose`、`AdaptObservationPose`、`ObserveAtPose`、
`CheckExplorationCompletion`、`ReturnToStart` 和 `SaveExplorationMap`。
节点向 Session 请求阶段并轮询匹配的 `PhaseTicket`，不在 tick 中阻塞等待
服务。暂停、取消和重启优先处理，旧阶段不能影响新的任务世代。

原生 FollowPath 把路径内部点、帧和身份纳入参考判断。取消请求被接收、
远端动作终止，以及机器人实际停稳是三种不同状态。
运动派发 gate 用于避免取消/任务替换后继续发送新动作，它不制造物理
停止回执。

## 信息增益与效率

覆盖单元具有稳定 ID、种类、几何、面积/权重、目标质量和必需传感器集合。
预测质量先按目标质量归一化并在完成处饱和，然后计算相对于已实测质量和
本轮已预约观测的正向改善。相同单元和来源取最大值去重，不能把 LiDAR 与
RGB-D 对同一几何的重复观察直接相加。

不要求特定源的单元使用任一真实来源的质量改善；要求两种来源的单元分别
维护两份质量。覆盖增益结合类别尺度及面积/权重，选择再计入路线距离、
转向时间、观察时间及返回路径。相机/点云可视模型提供视场、量程、遮挡
和观测质量约束。DeFM tokens 不单独生成已观测覆盖证明。

效率来自全局区域排序、局部互补覆盖选择、共享路线查询及增量图更新。
公平重访与完整栅格/yaw 遍历避免低即时收益的区域一直被高收益区域挤出。
`maximum_examined` 限制连不可行候选也计算在内的单次枚举工作。

## 完成与无法完成

有限分辨率任务的完成要求：没有活跃边界、待观测单元、待重访和未解决
区域；完成相容的公平遍历及栅格复核；感知与定位健康；实际地图和图分别
覆盖同一个固定观测前缀；多轮新证据下结论稳定。

质量提高更新 measured revision，不无故重启候选遍历；新增覆盖要求、
几何变化、传感器能力变化和可达性起点变化会使旧遍历证明失效。
遍历证明还有明确寿命。预算不足、尚有无法观察区域、定位故障和取消都
有各自结果，不能统一返回完成。

完整性适用于指定任务边界、分辨率和传感器能力下可达且可观测的覆盖域。
关闭房间、永久遮挡、反光透明物体以及无法确认的区域保留未完成说明。
源码判据和单元测试不能证明整个真实建筑已被全部探索。

## 丢失恢复与保存

完成请求捕获有限观测前缀，后续正常输入不会造成无限尾部追赶。
发生丢失时记录最早/最近丢失序号和恢复世代，阻止错误完成。
重新观测恢复需要新的实际覆盖、全任务范围地图回执及对应图证据。
要求保留原始数据的任务必须核验原始重放，不能用重新拍摄替代。

完成、返回和地图保存分别记录。`request_map_refresh` 被接收不代表融合
结束，`save_map` 返回接收成功不代表数据已经落盘。
实际定位、地图融合/持久化和反馈执行还需与部署平台接通并验证。

## 实际应用组装的核查边界

2026-11-18 对远端源码的检查确认，`task_main.cpp` 当前只创建、配置和
启动 TaskServer，没有配置探索会话。远端 TaskServer 公开的
`exploration()` 可以取得实际任务，并通过该任务的 `ConfigureSession`
注入依赖；本地 TaskServer 头文件中的 `ConfigureExplorationSession`
便利包装尚未同步，不能将它当作远端已经存在的调用入口。

远端 TaskServer 当前探索桥接使用旧的 `SetSubmitNavigation` 和
`SetNavigationProbe`。原生 NavigationTask 新增的
`SubmitGoalWithMotionPermit` 提供协作许可准入，按身份结果和取消分别复用
`GetResultForTask` 和 `CancelForTask`。v2.42 的 `TaskScheduler::RegisterTask`
已在实际导航、探索任务上安装协作准入和按身份结果/取消绑定；注册顺序
不影响服务查找。生产应用仍需调用 `ConfigureSession` 配置实际会话，
接入定位、融合和保存者。当前 `MappingClient`
的 `LoadMap` 只检查地图名，`SetInitialPose` 只接受字段并写日志，二者
不能作为定位或建图成功的回执。MapServer 的静态地图注入/读取接口和
`costmap_2d::saveMapToFile` 可以复用，但文件写入本身不能证明观测前缀已融合。

Argus 的 VisualSlamSystem 有非惯性 RGB-D 跟踪入口和原生消息适配器；
VoxelSlamSystem 的点云模式依赖 IMU，不能直接称为无 IMU 的 LiDAR-only
定位实现。纯点云模式仍须选定并验证实际可用的定位链。上述核查没有修改
map、TaskServer 或替真实生产者制造融合、定位、持久化和停稳证据。

远端 Cartographer 的原生 `SensorBridge::HandlePointCloud2Message` 会将
点云送入 rangefinder 管线，`LocalTrajectoryBuilder2D::AddRangeData`
在 `use_imu_data=false` 时可以从点云时刻初始化 extrapolator。因此现有
2D builder 可以作为平面室内 LiDAR-only 定位的候选入口；仍需验证相机/
雷达外参、逐点时间、地面与高度过滤及实际定位质量。这是 2D 位姿估计
候选，不是已经验收的无 IMU 三维 SLAM，也不证明其融合保存已接入探索。

## 原生导航的协作许可

探索为一次导航请求产生不可复用的身份和运动许可，NavigationTask 的
`SubmitGoalWithMotionPermit(goal, permit)` 只接受有非空 task_id 的
START/REPLAN。使用原生 NavigationGoal，不新增消息；普通控制命令继续
走 SubmitGoal，结果与取消使用同一 task_id。

该入口与普通 SubmitGoal 共享 typed admission 的并发序号、待准入结果
屏障和接受后身份发布规则。许可绑定到本次 native MotionDispatchGate，
在准入、服务等待、旧 worker/动作清理之后、tick 前和动作发送提交时
重新检查。第一次 false 或异常会对这次调用锁存拒绝，恢复回调返回值、
暂停/恢复、换用普通同身份重试或换一个重试回调都不能解除它。
新的调用必须重新完成准入与前一动作终态清理。

回调须有界、线程安全、非阻塞，不得重入任务控制或准入。查询不持有
native gate 或任务生命周期锁；串行替换/恢复期间可能持有任务控制锁。
运行中撤销通过真实 runner 清理、UUID 定向取消和 terminal drain 结束。
许可拒绝、取消 RPC 接受和动作终态是三个不同事件；这不是底盘停稳证明。
最后一个节点返回成功后也会核验许可，失效时不发布任务成功。
tick 前发现撤销时 runner 返回 canceled；动作节点已进入执行后发现撤销
可能返回 FAILURE，使任务终态为 failed。两条路径都必须定向取消并等待
动作终态，不能将节点失败强行重写为成功或清除已有 transport fault。

这是协作接口；v2.42 由 Scheduler 注册钩子把它接入探索父任务的运动所有权。

## Scheduler 的探索父子所有权

探索和普通导航都在领域准入期间申请 Scheduler 的根权限。
TaskServer 即使看到 NavigationTask 已 active 而跳过 RequestActivation，
普通 START/REPLAN 仍会经过该准入工厂。探索派发给同一个 NavigationTask
的请求携带子权限，保留探索根，不把该子请求当作另一个独占任务。
即使关闭全局 navigation-class 独占，探索与普通导航仍须互斥使用同一个
原生 NavigationTask 引擎。

跨根接管先不可逆撤销父子权限，再在 Scheduler 锁外取消任务，等待待准入
请求和原生 dispatcher/worker 清理。两秒内无法确认清理、取消异常或仍有
运动 fault 时，拒绝新接管并保留旧根屏障。Scheduler 的查询回调只读取
原子权限，不调用任务 getter；准入完成回调只更新计数。普通结果与取消
继续使用现有 automsgs task_id，不新增消息。

RequestActivation 的初始预约跨越 TaskServer Request/Submit 间隙；
reaper 不因原生任务暂时 idle 就释放它。领域工厂消耗预约并登记待准入。
回收在锁外核验真实领域清理，再核对同一根、版本和待准入数，防止旧快照
释放新工作。旧权限在恢复、再初始化及重启后仍保持撤销。暂停的探索会
撤销旧子请求，恢复后派发新的身份和子权限。

NavigationTask 与 ExplorationTask 提供 `MotionDrained`，涵盖领域待准入、
实际运行和所持动作清理。导航还检查真实 runner 的 `IsRunning`：终态
发布早于取消回调和树销毁，这段时间仍保留所有权。父探索 dispatcher 与
Scheduler 同时取消同一个子导航时，重复取消返回不代表另一线程已清理；
版本化取消回调在锁外有界等待对应 worker 的清理，不因此制造永久 fault。
其他旧插件仍只有 TaskInterface 的取消/生命周期
契约；这些软件状态不能作为底盘已经停稳的物理回执。完整生产闭环仍需
实际定位、地图融合/持久化、模型配置和底盘执行验证。
TaskServer 对已 active 的 NavigationTask 可能跳过 RequestActivation。
普通导航的领域准入仍经过 Scheduler，因此探索子导航占用同一个引擎时，
外部导航会按抢占配置被拒绝，或在父子权限退役并确认软件清理后接管。
该路径已在 Scheduler 和领域任务中接入；TaskServer 保留用户版本。
这项所有权接入仍需与真实建图会话、定位和模型配置一起完成生产组装。

原生构建与证据见 [检查方法](09_native_exploration_validation.md)。
