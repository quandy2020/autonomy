# 多传感器室内探索任务

探索位于 `autonomy/task/exploration/`，是任务策略。全局路线由
`autonomy/planning/planner/vista/` 提供，局部控制接入 PolicyController。
保留 TARE 的全局区域访问顺序和局部覆盖思路，以
`HierarchicalCoverageStrategy` 表达职责，不创建 `tare/` 目录。

## 所有权与目录职责

`ExplorationTask` 负责原生任务生命周期及身份化输入。
应用通过 `ConfigureNativeExploration()` 同时安装原生 Session 工厂、传感器
配置和采集时刻定位查询。既有 `ConfigureSession()` 与
`ConfigureObservationSource()` 保留为独立接入接口。安装后，task owner
自动管理预 START、START 替换和终态的
原生 Reader。暂停/恢复保留序号；替换必须等待旧 Reader 成功退役。
退役超时或失败会锁存输入错误并拒绝新 START。
输入清理 future 与机器人停止/地图落盘回执分别审计，详细契约见
[多传感器观测入口](../10_Perception/13_multisensor_observation.md)。
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

## 跨进程路线查询

任务进程通过 `integration/native_vista_path_provider.hpp/.cpp` 复用 PlannerServer
的 `/get_plan`。调用方向是探索 worker → 原生 Autolink GetPlan → 服务器已配置的
VistaPlanner → 同一张持久化增量图 → automsgs Path 与实际图证据 → worker 结果。
同进程嵌入使用 `VistaPathProvider`；两个实现都满足现有 `PathProvider` 接口。

构造时传入已有 task 通信 Node，配置明确的 planner id、frame、响应超时和
路径点上限；再将同一个 provider 传给策略、GoalPoseAdapter 和 ExplorationSession。
跨进程组装不能另外实例化 VistaRoutePlanner，也不能在 `update_route_map` 中维护
一份任务私有图。服务器通过自己的地图读取源完成更新。

每次路径查询保持精确起终点与观测朝向，路径随自身证据返回；`Evidence()`
则通过独立只读请求刷新 graph owner 的前缀。单客户端对请求进行串行化，
超时与取消保留未完成 future，迟到结果不参与下一目标。无服务、旧协议和
非法响应产生查询错误，进入原有 worker 的错误/重试路径；它们不能提供区域
不可达证明，也不能满足完成判据。

在原始响应预算内，`retry_interval` 允许同一只读 GetPlan 请求重传，仍使用
同一个原生 sequence/promise 和业务 token。它解决晚发现接收器时的首包丢失，
不扩张客户端 pending 队列；暂停/取消或超时后不会继续重传。客户端销毁关闭
回调入口后才释放 transport，服务销毁关闭入口、清理待执行队列并等待
实际 worker 退出后再退出拓扑并释放传输端点，以保留已进入请求的回复
通道。异步历史发送持有独立快照，不引用已经销毁的发送器。
历史接收同样使用拥有独立状态的静态任务；到达标志是共享原子变量，
不引用已经返回的函数栈。所有接收传输的分发器监听器复制基类的回调
状态，析构先关闭转发并等待实际调用退出，晚到的副本不再调用业务层。
历史任务通过协程友好的短轮询观察退役，并释放临时通道接收器。

该适配器及原生 Session 工厂已实现；平台仍需提供实际定位、地图融合和
前缀回执。接口和测试可用不等于整机探索部署已完成。

## 原生 Session 工厂与平台接入

`integration/native_session_factory.hpp/.cpp` 提供
`ConfigureNativeExploration(task, node, configuration, platform)`。它直接使用
远端 TaskServer 现有的 `exploration()` 和 `node()`，不需要修改 TaskServer。
在 TaskServer 已配置、任务空闲时安装；配置完成后可按原有顺序启动服务器。

```cpp
// server 为已完成 Configure 的原生 TaskServer。
// configuration 和 platform 由实际设备/定位器/地图后端提供。
const bool installed = autonomy::task::exploration::ConfigureNativeExploration(
    server.exploration(), server.node(), configuration, platform);
```

`NativeExplorationConfiguration` 汇集 Session、覆盖选择、输入源和路线参数。
三个位置的 `SensorMode` 必须一致；RGB-D 模式只配置实际相机，LiDAR 模式
只配置实际束方向和外参，联合模式配置二者。执行模式必须在
`validated_controller_modes` 中明确列出；工厂不会自动声明模型已经适配。

工厂在调用线程核验话题、范围、模式、内参、外参和工作预算，然后一次性
安装 Session 工厂与源配置。源和工厂共享一个配置修订，活动或暂停任务、
不同 Node、输入/运动故障均拒绝替换。旧 Reader 必须由 owner 实际退役后
才能按新配置创建，配置成功不代表新订阅已就绪。

实际 owner 创建 Session 时，先取得本世代 `NavigationServices` 的协作许可
派发器，再调用平台 `create_mapping_services`。融合必须提供
`request_map_refresh`、`poll_map_refresh` 和 `cancel_mapping`；要求保存时还
必须提供 `save_map`、`poll_map_save`。平台不得覆盖导航回调或
`update_route_map`。随后创建一个 `NativeVistaPathProvider`，共享给覆盖策略、
目标位姿适配器和 Session，服务器继续持有唯一的增量图。工厂返回尚未启动的
BT Session；异常由任务 owner 关闭派发器和会话，报告失败。

`NativeExplorationPlatform` 的 `localization` 和 `clock` 必须来自实际定位链。
每个采集时刻的证明需匹配路线坐标系、实际标定修订，以及
`world_from_base * base_from_sensor == world_from_sensor`。RGB-D 的实际
CameraInfo 必须匹配覆盖模型内参；不一致锁存标定错误，不能让预测视域
使用另一套相机参数。`prepare_mapping` 在目标明确要求启用建图时调用。
这些检查只核验供应者契约，不把请求接收、Reader 创建或文件存在视为融合/
保存成功。平台必须通过版本化实际回执确认观测前缀和持久化完成。

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
`exploration()` 和 `node()` 可直接用于上面的原生工厂；本地 TaskServer
头文件中的 `ConfigureExplorationSession`
便利包装尚未同步，不能将它当作远端已经存在的调用入口。

远端 TaskServer 当前探索桥接使用旧的 `SetSubmitNavigation` 和
`SetNavigationProbe`。原生 NavigationTask 新增的
`SubmitGoalWithMotionPermit` 提供协作许可准入，按身份结果和取消分别复用
`GetResultForTask` 和 `CancelForTask`。v2.42 的 `TaskScheduler::RegisterTask`
已在实际导航、探索任务上安装协作准入和按身份结果/取消绑定；注册顺序
不影响服务查找。生产应用仍需调用 `ConfigureNativeExploration` 并提供实际
定位、融合和保存者。当前 `MappingClient`
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

## 实际部署与独立地图评估

生产链按已有职责组装：RGB-D / LiDAR 原生输入提供采集时刻的定位与
有效观测；实际建图会话通过有限输入前缀返回独立融合回执；探索 owner
按真实覆盖与地图快照选择目标，使用 PlannerServer 的共享 Vista 图
提交路线；SanD 的样条统一调用 `common/math/bspline.hpp`，通过已有
NMPC/MPPI 和 automsgs `Path` / `TwistStamped` 完成跟踪。map 源码继续
保持不变，应用配置需要显式接通这些已有入口。

2026-11-18 的真实 AutoSim 结果目前只覆盖独立 RGB-D 环境启动及原生
消息传输，生产 Task/Planning/Control 应用也已编译。探索会话、实际
RGB-D 定位/地图融合保存、DeFM 条件 SanD 和执行安全供应者的完整组合
仍未验收。当前 SanD ONNX 是原始深度条件，不能据此声明已经适配 DeFM。

真值 OccupancyGrid 只进入离线 `scripts/compare_exploration_maps.py`。
它采用固定的起点连通真值自由域，分别比较算法地图的已知程度、正确
free 覆盖、错误占用及剩余未知区域；算法预测信息增益、目标到达或消息
数量不能替代真实覆盖。地图只比较已明确对齐的 frame、resolution 和
原点。具体报告、使用方式及验收缺口见
[原生探索验证](09_native_exploration_validation.md)。

原生构建与证据见 [检查方法](09_native_exploration_validation.md)。
