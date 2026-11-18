# Autonomy 多传感器室内自主探索与建图方案

版本：v2.15 最终方案审查稿（正式 v2.12 原生 CTest 11/11；本地派发隔离验证单独记录）  
日期：2026-11-18  
状态：ExplorationTask owner、原生 BT、typed-task 与 NavigationTask 领域接受/worker 生命周期已正式同步和验证；SpaceHero planning/task/control 与十一个相关测试目标构建成功，CTest 11/11 通过，2.67 秒。v2.15 派发门控已本地接入并通过隔离编译及原有 21 类导航场景；新增派发竞态与正式部署尚待验证。父子调度、真实取消、三模式观测、融合/保存、学习控制与机器人任务验收仍未完成。当前边界见 [集中审查稿 §15](multisensor_exploration_final_plan.md#15-当前实现与验证边界)；§20 各版本记录保留为历史。  
适配目标：Autonomy 原生 C++、Autolink、Protobuf；目标设备 Jetson Orin。

审查入口：[最终方案总览与八项审定决策](multisensor_exploration_review.md)。该总览按架构、传感器、增量图、收益、Opt、执行、BT 和完成协议组织，本文保留完整接口和历史证据。

集中审查稿见 [最终设计](multisensor_exploration_final_plan.md)，先核对任务行为与模块契约，再查看本文件的算法细节和历史实施记录。

审查结论：采用 FAR 增量可见图与 TARE 分层覆盖框架，统一为支持三种传感器配置的探索闭环。规划插件名为 VistaPlanner，探索归属 task，地图保持现有实现，Opt 负责保持观测意图的位姿调整，冻结 DeFM 与经训练的 SanD 适配负责局部轨迹。

建议按以下顺序审查：先看第 17.1 节的八项默认政策；第 2–3 节看模块所有权和数据流，第 4–5 节看三模式及地图边界，第 6–9 节看图、覆盖收益和 Opt，第 11–13 节看执行平滑、BT 与完成协议，第 15–16 节看目录及验收，第 19–20 节看异常情况和实施缺口。第 12.6–12.7 节明确父任务调度、初始化与暂停恢复契约；第 20.13 节保留隔离验证历史，第 20.14 节保留 owner 部署历史，第 20.17 节记录 NavigationTask 正式验证与当前审查。目录是目标布局；频率是设计预算；专项测试不能证明当前完整任务或远端全部工作树已经通过。

三个主要验收依据：完整性由实际覆盖账本、公平扫尾与可撤销完成协议证明；效率在相同覆盖标准下比较路程和时间；平滑性在实际切换点验证位置、速度、加速度与 jerk。它们均需测试与任务实验，不能由所采用算法的名称直接推出。

本稿提交审查的默认决策：

1. 任务是单层室内几何探索建图；范围和覆盖要求在启动时固定，没有真值边界时保留未确认清单。
2. 三种配置共用探索策略、Vista 图和观测目标契约；每种配置必须有独立可工作的定位链。
3. 默认 GEOMETRY_ANY_SENSOR，一份几何覆盖按稳定单元去重；指定双来源或多视角质量时分别验收。
4. 探索放在 task/exploration，路线插件放在 planning/planner/vista；map 的所有既有实现、接口和消息保持不变。
5. Opt 只调整观测位姿，适配后重新验证原任务的可见性、required_views、覆盖关联和最低收益。
6. 冻结 DeFM，训练一套共享模态条件融合与 B 样条扩散头；执行端另外验证时间、切换边界、扫掠体与停止能力。

历史 v2.1 证据：逐文件核验的同步清单包含 120 个文件，该次更新 10 项并备份既有内容。SpaceHero 中 autonomy_planning、autonomy_task、autonomy_control 与六个测试程序编译成功；Vista、服务器、覆盖、Session、阶段与 BT 六项 CTest 6/6 通过，总耗时 1.93 秒。本地四项相关回归 4/4 通过，2.81 秒。map 的 471 个文件 SHA256 与保护基线一致，零新增/删除/内容变化。日志、同步清单和核验结果见第 20.6 节；第 20.5 节保留 v2.0 历史记录，当前派发器部署记录见第 20.11 节。

v2.1 四项本地专项和六项 Docker 专项分别记录。覆盖、会话、阶段与 BT 测试使用真实原生库，导航/建图回执仍由测试注入。八个 BT 节点已进入真实 task 编译；最新 owner 正式部署与八项 CTest 见 §20.14，十三类隔离专项见 §20.13。真实导航/建图服务、父子调度和整机任务尚未验收。注入回执不能代替真实服务。第 8 节的质量正改善收益已在 v2.1 实现并通过本地相关回归；第 13.2 节的有限观测截面、真实融合/保存、三模态策略与三维执行链均保留为待实施或待专项验收。

本版明确共享图的进程边界：PlannerServer 是图所有者；task 通过路径提供接口使用它。GetPlanner 只证明同进程共享能力；独立任务进程须使用 Autolink 规划服务，不能解引用另一个进程的插件指针。图复用、任务唯一提交者、可审计完成与轨迹切换约束是审查重点。

2026-11-17 v2.2 已保存的只读核验：远端 HEAD 为 7b251c5e5b5a2deeace5b7558a750d6917bac23f；六个任务/规划入口文件与此前保存的逐文件快照一致，最近部署清单的 120 个文件内容未变化，map 的 471 个文件仍与保护基线一致。该记录覆盖此前部署的文件，不覆盖之后的新本地任务代码。本次审查没有新增业务 build、CTest 或机器人运行记录；v2.1 通过记录不能延伸为当前本地代码或新 HEAD 的全工程验收。

## 1. 核心方案与约束

采用 TARE 的全局粗粒度、局部细粒度覆盖规划框架，使用 FAR 的增量可见图路线规划机制，封装为 VistaPlanner。探索作为 task 下的一种策略行为。DeFM 提供冻结的深度特征基座，SanD 风格的条件 B 样条扩散生成局部轨迹，独立几何校验与高频跟踪完成执行。

支持 lidar_only、rgbd_only、lidar_rgbd 三种模式。输入模式可以配置；运行中的有效传感器集合由健康状态决定。

本版固定四个设计选择：地图由既有建图链管理；导航只由 ExplorationSession 的统一入口提交；覆盖要求在任务开始时确定，传感器降级保留尚未兑现的要求；探索完成由真实观测及地图同步证据确认。八个 BT 节点分别请求选择、适配、执行和收尾阶段，使行为树实际编排这些操作。

固定约束：

- autonomy/map 的目录、代码、接口和消息定义保持不变，继续使用现有 GridMap、Costmap 和地图读写入口。局部 3D 安全证据由控制侧消费者维护。
- 地图读取适配、版本差分、覆盖记录和传感器融合证据放在 map 外部。
- VistaPlanner 位于 autonomy/planning/planner/vista；插件实例名 vista_planner。
- 探索策略位于 autonomy/task/exploration；不设置 tare 中间目录。
- 目标位姿适配复用 Opt 的几何核心，限定为对探索观测目标的补充。
- 无 ROS 耦合；不指定底盘型号，使用平台提供的足迹、运动约束和执行接口。
- 初期覆盖单层室内、与起点连通的可通行空间；跨楼层、电梯和操作开门另行扩展。
- 本文是设计审查基线；代码与验证进度单独记录于 implementation_validation.md，不将设计要求视为已通过的验收。

源代码核对：2026-11-17 的 v2.1 原生构建仍以其保存的日志及逐文件指纹为准；v2.2 核验 HEAD 7b251c5 是历史快照，本次远端检查见第 20.9 节。工作树内容以逐文件 SHA256 为准，不能仅凭 HEAD 判断实现状态。现有规划目录包含 navfn、theta_star、dijkstra；控制位于 autonomy/control/controller；探索已有 exploration.hpp/.cpp 入口。GlobalPlanner::CreatePlan、原生 BT 基类与 MapServer::GetRawStaticMap 拷贝接口已核对。以下目录是最终目标布局；Opt 中存在对应算法文件不代表远端任务生命周期或三模态模型已经接入。

本版的三维任务含义：地面机器人在既有地图能表达的单层通行空间内选位置和朝向，三维射线/表面用于观测收益、遮挡、地面连续性与垂直净空。路线图、历史覆盖账本和当前执行几何分别维护；2D 路线不能代替三维观测与安全证据。跨楼层、悬空自由飞行及操作开门需要另行定义运动空间和任务能力。

## 2. 模块边界和所有权

ExplorationTask：拥有探索会话、预算、策略、任务状态和当前导航请求。通过既有任务调度通道提交导航、返航和建图请求。

HierarchicalCoverageStrategy：维护全局区域进度、局部观测候选和重访队列，输出 ObservationGoal；不直接发送底盘命令。

VistaPlanner：由 PlannerServer 持有，拥有长期增量可见图；实现现有 common::GlobalPlanner 的 CreatePlan(start, goal, path, cancel_checker) 接口。普通导航与探索的路线请求到达同一图所有者。

GoalPoseAdapter：将 ObservationGoal 交给 Opt 做有限位置/朝向调整，再由 ViewpointEvaluator 重新验证收益。不能将观测目标退化为任意可停靠点。

PolicyController/SanD：消费局部目标、传感器条件和运动反馈，生成带时间的 B 样条候选；不维护探索覆盖账本，不决定全局访问顺序。

Safety：验证实际机器人扫掠体、净空、未知区域和动态障碍；拥有否决权。安全约束不进入“高收益抵消风险”的加权评分。

TrajectoryTracker：跟踪已经验证的轨迹，执行过期保护、参考衔接和反馈控制。

MappingTask/上游定位建图：拥有地图更新与保存。探索可以请求启动、刷新和保存，但不另建一个 map owner。

共享地图消费者契约放在 perception/data/map_reader.hpp，由 planning 与 task 共同读取。Vista 不 include task 的实现或状态；task 中原先的 map_reader 仅保留迁移兼容别名。原生 Costmap 读取桥位于 planning/planner/vista/native_costmap_reader，不给 autonomy/map 增加任何入口。

路径提供接口隔离进程部署：同进程组合时，显式注入 PlannerServer::GetPlanner 返回的既有 Vista 实例；独立进程时，task 使用原生 Autolink 规划客户端，服务端调用同一 Vista 实例并返回真实 Path。初版可从返回路径计算路线长度；后续批量距离查询也只在该图所有者处执行。跨进程请求需关联 request_id、frame_id、map_epoch 和 graph_revision；缺少相容版本时拒绝沿用旧路线，不另在 task 构建一张并行全局可见图。

关键所有权规则：

- 一个探索会话只有一个导航提交者。感知模块、策略、Opt 和 BT 子节点都不能绕过它再发送 waypoint。
- 回调只向会话队列提交输入；会话状态在拥有者线程更新。
- Vista 图由单一更新线程修改；路径查询使用只读版本或受锁保护的读视图。
- 每次异步结果必须携带会话、请求、目标和地图对齐版本；取消、重置和回环后丢弃不再适用的结果。
- ExplorationTask 管原生任务接口，ExplorationSession 管算法会话与事件处理；BT 节点通过会话请求动作和读取结果，不分别拥有另一套生命周期。
- 探索专用 BT 节点位于 task/exploration/plugins/action 与 plugins/condition，继承 task/behavior_tree/plugins/bt_node_base.hpp。它们沿用现有 navigation/plugins 的按任务分组方式；通用 Control/Decorator 继续复用公共行为树目录。

## 3. 总体数据流

    LiDAR 点云 / RGB-D 帧
             |
             v
    传感器适配 + 时间/外参校验 + 健康评估
             |
             +--> 上游定位/建图 --> 现有 map
             |                        |
             |                        v
             |                 MapReader 安全读取
             |                 自有版本和分区差分
             |                        |
             +--> 几何观测缓冲 --------+----------------------+
             |                        |                      |
             v                        v                      v
    深度视图适配 + DeFM       Vista 增量可见图        实际覆盖更新
             |                        |                      |
             |                        +--> 路线距离查询 <----+
             |                                               |
             |                         分层覆盖 + viewpoint 选择
             |                                               |
             |                         Opt 目标适配 + 收益复核
             |                                               |
             |                         ExplorationTask 提交导航
             |                                               |
             +--------------------------> 局部目标 / 路径引导
                                             |
                                             v
                                    SanD B 样条候选
                                             |
                                  边界约束 + 时间参数化
                                             |
                                  Safety 扫掠体/动态校验
                                             |
                                    高频轨迹跟踪/执行
                                             |
                            位姿反馈 + 实际观测 + 覆盖兑现

地图几何、学习特征和覆盖证据有独立用途。DeFM 的特征新颖度不能直接标记空间已覆盖；轨迹生成成功也不能代替几何安全验证。

## 4. 三种传感器模式

### 4.1 lidar_only

- 输入实际 3D LiDAR 扫描，使用真实束方向、视野、有效量程和回波状态。
- 位姿来源必须是可独立工作的 LiDAR 里程计/SLAM；不默认存在相机或 IMU。
- 可见性依据束模型与遮挡，不默认水平 360°，也不默认垂直视野完整。
- 原始点云和扫描射线用于覆盖、安全和路线几何。
- 学习支路将点云投影为所需的虚拟透视深度视图，使用正确深度定义与有效性掩码。
- 点稀疏、扫描盲区、丢包、无效回波都不能被填成已知 free。
- 若尚无经验证的 LiDAR 条件模型，只能启用明确配置且经验证的几何控制后端；不能静默复用 RGB-D 权重并宣称兼容。

### 4.2 rgbd_only

- 输入标定后的深度、RGB、相机内参和外参；深度使用米。
- 位姿来源为独立可用的 RGB-D 里程计/SLAM。
- 视锥由相机模型计算，候选包含机器人位置与 yaw；固定相机 pitch 由安装外参提供。
- 深度缺失、反光、透明、量程外区域保留未知，不通过特征推测自由空间。
- 使用原生透视深度视图进入 DeFM；真实几何观测用于覆盖和安全。

### 4.3 lidar_rgbd

- 上游输出统一位姿基准；两种观测按各自采样时间变换到共同坐标系。
- LiDAR 扫描运动补偿使用可用的连续位姿插值，不能假定整帧点在同一时刻采集。
- 不要求把不同频率的输入强行拼成“同一时刻”；记录年龄和时间差，超过有效窗口的输入不参与当前执行决策。
- 几何覆盖按稳定覆盖单元去重，学习特征按来源、视角、时间和有效性区分。
- 对同时有效的 occupied/free 冲突记录证据与时间；任务侧不做无依据的清除。执行校验保守处理仍然有效的障碍证据，并触发重新观测。
- RGB 纹理和类别语义只作为单独的可选任务收益，不作为几何覆盖成功条件。

### 4.4 配置模式与有效模式

configured_mode 表示任务允许的传感器配置；active_sensors 表示本周期仍然有效的传感器。

健康判据包括输入年龄、有效比例、变换是否存在、时间同步质量、扫描完整度和定位可信状态。模式退出和恢复使用独立持续窗口，避免每帧来回切换。

退出滞回只用于稳定模式状态，不能延长几何证据的执行有效期。当前帧已经超龄时，即使模式尚在退出窗口内，也不能据它批准新运动。

联合模式允许降级到单传感器的条件：剩余观测有效、剩余定位链有效、控制模型支持剩余模态、当前安全校验可完成。任一条件不足则停止新的平移执行并进入等待/恢复。

仅传感器消息仍在发布，不足以证明定位链仍可工作。

### 4.5 任务覆盖政策

任务开始时固定 coverage_policy 与 coverage_requirement_revision：

- GEOMETRY_ANY_SENSOR：允许任一配置传感器兑现同一几何单元，联合模式按单元去重。
- GEOMETRY_PER_SENSOR：要求指定来源分别达到质量目标，按单元、来源、必要方向分别保存证据。
- RGB 纹理或类别要求通过独立的可选质量目标表达；lidar_only 任务不配置无法提供的 RGB 要求。

configured_mode 决定任务允许使用的来源；active_sensors 决定当前能够兑现哪些要求。理论可观测范围由任务边界、机器人可达空间和配置传感器模型定义，不能随当前孔洞、传感器掉线或候选采样失败而缩小。运行降级后，暂时无法兑现的要求保留在账本。显式改变任务要求生成新 revision，并分别保留改变前后的结果。

## 5. 地图适配：map 保持不变

MapReader 复用现有安全读取入口。已核对 MapServer::GetRawStaticMap 提供地图拷贝入口；不直接跨线程持有 GridMapWrapper 返回的可变对象，也不将复用的发布消息指针视为不可变快照。

共享 MapReadView 的所有者是消费者。原生 Costmap 读取在地图 mutex 内复制几何和占据数组，释放锁后做版本比较与历史维护。读取动作不等于新观测，因此没有真实观测时间时 source_stamp 保持 0；没有真实融合确认时 mapping_synchronized 保持 false。2D Costmap 转换明确标记 has_height_evidence=false，不用默认地面/顶棚数值制造三维净空证据。

GetRawStaticMap 的存在只证明静态地图可以安全读取，不证明该返回值就是在线 mapper 的最新结果。接入时必须核验规划使用的在线 GridMap/Costmap、上游建图输出与保存结果之间的真实数据来源和坐标关系；不得把未更新的静态快照当成增量建图源。若现有在线数据只通过发布通道可读，消费者立即深拷贝并记录真实来源；仍不改 map 目录或消息。

适配层生成自有 MapReadView：

    MapReadView
      map_epoch          坐标基准/全局对齐代数
      geometry_revision  地图几何或占据内容版本
      frame_id
      source_stamp
      observation_watermark
      grid_geometry      原点、分辨率、尺寸
      immutable_data     本次读取拥有的不可变拷贝
      dirty_regions      由适配层比较得到的变化区域

这些字段是消费者内部契约，不要求加入 map/proto。

observation_watermark 是每个观测来源的序列截面，至少包含 LiDAR 和 RGB-D 各自已处理序号；不能把两个独立序号取最大值后当作所有输入都已处理。完成确认还要记录地图差分、可见图、覆盖账本对该截面的处理结果。

该字段只证明消费者处理到了哪个截面，不自动证明上游 map 已融合这些观测。地图刷新/保存确认的真实语义必须从既有建图任务入口核验；如果缺少有意义的同步依据，不能自行填写一个序号制造“map 已处理完”的证明。严格完成确认需报告这项同步状态，无法确认时保持未完成状态，不修改 map 消息来掩盖限制。

规则：

- 发布时间变化不等于地图几何变化。
- 首次输入、分辨率/原点变化、回环对齐变化分别处理，不能只比较时间戳。
- rolling window 移出当前范围不等于历史障碍消失。
- 局部窗口之外的覆盖状态和已访问连通记录应长期保存，不能跟着窗口一起删除。
- 初版可比较分区内容得到 dirty regions；若仍需复制整张地图，必须计入端侧预算，不能宣称端到端已经完全增量。
- 2D GridMap 路线规划需要原始观测的 3D 扫掠体与垂直净空补充；单高度地图不能自动识别所有悬空结构和负障碍。

历史记录与当前安全证据分开：历史覆盖账本用于避免重复探索；执行几何记录单元的观测时间、来源、有效性和坐标版本。地图发布的新时间戳不能刷新旧单元的证据年龄。滚动窗口合并保留的旧 free、occupied 和净空不能自动成为当前可执行空间。

未知覆盖范围不能由数组边界推断。显式任务边界之外可按配置排除；没有建筑边界时，地图外缘的未知邻域保留为扩展/待确认任务。资源上限、候选上限或窗口裁剪不能把这类任务变成“已完成”。

## 6. VistaPlanner：增量 visibility graph

结构延续 FAR 的局部几何更新、全局可见图保留机制。

原生接入需闭合类注册、实例名解析、带真实 options/Costmap 的初始化和共享图注入四个环节。仅注册 VistaPlanner 类、或工厂创建未配置对象，不等于 planner_server 已能使用 vista_planner。规划返回真实 Protobuf Path，失败、取消或回调异常先清空输出；保留请求的首末位姿及最终观测朝向。原生规划成功仍不赋予 3D 安全或执行许可。

### 6.1 节点和边

节点来源：障碍轮廓拐点、通行连接点、起终点临时连接点和必要的已访问连接节点。

节点保存稳定 ID、世界位置、几何来源、最后验证版本和状态。边保存端点 ID、长度、净空、已知/待验证状态、影响区域和验证版本。

碰撞膨胀只应用一次。不能从已膨胀轮廓提取图后又重复按机器人半径膨胀，导致窄门被错误封死。

### 6.2 增量更新流程

1. 读取 dirty regions，更新相关障碍轮廓。
2. 对节点进行匹配，保留未改变部分的稳定 ID。
3. 更新邻接关系，检查受影响边。
4. 使用边空间索引查找所有穿过变化区域的长边，不能只检查端点附近。
5. 新障碍导致边失效；障碍移走后，根据轮廓变化和阻挡依赖重新连接候选边。
6. 发布新图版本，重新校验当前剩余路径。

地图整体对齐发生变化时，使用新 epoch 处理已有图和覆盖账本；有可靠锚点变换则重投影后复核，否则失效重建。旧 epoch 的路径不能直接继续执行。

### 6.3 查询与备用搜索

VistaPathProvider 的同进程实现复用既有 VistaPlanner 路线能力；独立进程部署由 Autolink 路径客户端实现同一 PathProvider 契约。GetPlanner/SharedRoutePlanner 不跨进程共享内存。当前服务器测试证明同进程共享、配置与真实 Path 输出，跨进程探索调用尚未接入。

可见图失败时，使用已知空间的栅格连通搜索作备用检查。只有图失败不能证明区域不可达。

FAR 式未知区域尝试能力可用于高层路线估计；带未知段的路径必须标记未验证。当前执行区段只能进入有有效几何证据且通过 Safety 的区域。

图上的最短路径不等于整个连续空间的全局最短路径。完备性声明也必须明确限定到实际地图与候选分辨率。

## 7. 探索策略：保留 TARE 分层机制

### 7.1 全局层

RegionManager 管理粗粒度区域及其待观测、已覆盖、暂时受阻、无法确认状态。

VisitOrderOptimizer 使用 Vista 图上实际路程，计入转向、观测时间、覆盖重叠和返航代价。保留 TARE 全局访问次序优化思路，TSP 求解细节封装在模块内。

TraversalGraph 保存实际访问位姿及可通行连接，是路线连通记录，不承担 SLAM 图优化。

### 7.2 局部层

在当前局部规划范围：

- 提取有效 frontier 和待补全表面。
- 在已知安全空间内采样候选位置，并按相机/扫描模型采样朝向。
- 先做足迹、垂直净空、坡度/台阶和可达性硬筛选。
- 计算边际覆盖收益，选取有限 viewpoint 集。
- 优化局部访问次序，与全局区域路线衔接。

初版保留 TARE 的分层覆盖、候选贪心边际收益更新和访问次序优化；不要将其降成只选一个最近 frontier 的系统。

### 7.3 不遗漏机制

收益优先调度与公平重访分开：

- 新增收益较大的候选优先。
- 低收益 frontier 进入扫尾队列，不能因固定收益阈值永久丢弃。
- 暂时受阻设置重访时刻，地图变化后提前重启。
- 永久不可达必须保存判定依据、版本和可重新打开条件。
- 高收益候选反复预测失败时修正其质量权重，切换观测方向，不能无限重复同一目标。

区域和候选数量超过当轮计算预算时，未评估部分留在队列，记录 budget_exhausted；不将前若干个区域当作全部区域。扫尾采用可恢复的分页/游标，跨轮次检查完整候选集合。

frontier 的身份由空间位置、方向和当前几何关联确定。地图变化影响某项观测任务时，重新打开相关项并撤销受影响的覆盖结论；纯发布时间变化不重新产生任务。暂时看不见、退出局部窗口或稀疏图断连，不足以删除未完成项。

收益驱动的候选采样之外，扫尾回退按配置分辨率枚举已知可达位置、允许观测朝向和未完成覆盖关联，保存跨轮次进度与每项否决依据。地图变化只重新评估受影响项；当前版本下未检查完的项继续待处理。无观测收益的结果需区分已覆盖、视角不足、证据失效与配置模型不可观测。完成声明针对这套明确的离散位置/方向模型，不能从有限启发式候选为空推出连续空间不存在有效观测位姿。

## 8. 信息增益、融合与实际覆盖

第一版采用新增有效覆盖的代理评分，不称为严格 Shannon 信息增益。

严格概率信息增益需要地图后验、传感器似然和未来观测期望。当前消费者契约没有经过校准的概率模型，因此采用物理可解释的几何覆盖与质量权重。map 零改动本身不禁止消费者维护独立后验；后续若引入概率收益，必须补齐模型和标定证据，不能将当前代理改名为熵下降。

覆盖单元分为：

- 未知空间边界：用于推进探索。
- 已发现但观测不足的表面单元：用于地图补全。
- 必要的观测方向单元：用于多视角要求。

不同类型分量使用固定参考尺度分别归一化后加权。原始点数不能直接代表面积；点云密度变化不能改变同一表面的收益。

对候选 q 和类型 k：

$$
G_k(q)=\sum_{i\in\mathcal I_k}A_iW_i
\left[\max_{s\in\mathcal A}\{V_{i,s}(q)\widehat C_{i,s}(q)\}-B_i(S)\right]_+.
$$

A_i 是面积或标准化覆盖单位；W_i 是任务权重；C_i 是真实观测质量相对于任务所需质量归一化后的完成程度，限制在 [0,1]；V 是可见性；预计质量必须按与 C_i 相同的规则归一化为预测完成程度 \widehat C，限制在 [0,1]。B_i(S) 为真实完成程度 C_i 与已选集合 S 的预测完成程度的最大值；S 为空时 B_i(S)=C_i。[x]_+=max(0,x)。当前有效传感器集合 active_sensors 用于式中的来源下标 s，避免与面积 A_i 混淆。

质量预测考虑距离、量程、视角、有效深度/回波比例、表面入射角、扫描密度、历史失败和定位/运动状态。未经标定的预测是选择代理，不是观测成功概率。

上述公式是用于选择候选的预计边际质量改善代理，不是实际提高量的概率期望。第一版实际质量采用同一单元、同一来源的最大有效观测记录；重复较差观测不提高完成程度。预测质量不超过已获得质量时，质量改善收益为零；仅用 (1-C_i) 乘预测质量会错误奖励这类重复观测。predicted_gain 与 realized_gain 必须分开记录；若某视角反复无法提高质量，降低其预测权重、换视角或进入有界重访，不能通过累加重复观测伪造覆盖完成。独立帧数、必要方向和停留要求保留为单独的待兑现项；质量改善收益为零不意味着这些条件已经完成。

v2.1 实现保留原始质量 q∈[0,1]，归一化仅在评分时使用 min(1,q/required_quality)。例如要求 0.8、实际质量 0.6：候选质量 0.4 的改善为 0；候选 0.7 为 0.125；候选 1.0 饱和为 0.25。已选 0.7 后，新候选 0.65 的边际改善仍为 0。双来源分别要求时，每个来源按自己的实际/预测基线计算，再按必要来源数平均，不能将已完成 LiDAR 的质量用于填补 RGB-D 缺口。贡献、预选缓存和实际账本的类型/质量都先校验；预测与候选选择不修改真实 revision。

计算顺序固定为硬约束筛选、可见性预测、质量加权、覆盖去重、边际更新、路线排序。定位不可信、足迹不满足、无有效地面证据或目标不可观测的候选不能通过提高信息收益抵消硬约束。

联合模式采用 max 去重，作为保守初版策略；它不能表达全部互补测量的概率收益，也不假定 LiDAR 与 RGB-D 误差独立。

若任务只要求一份几何覆盖，联合观测在同一单元上去重。若任务要求 LiDAR 和 RGB-D 分别达标，则保留 C_i,lidar 与 C_i,rgbd 两本质量进度，分别验收；融合后的 max 不能替代这两个要求。RGB 纹理/类别覆盖也是独立可选目标，不由 DeFM 深度特征自动兑现。

分别要求传感器质量时，对每个必要来源分别计算 [V_i,s(q) \widehat C_i,s(q)-B_i,s(S)]_+，B_i,s(S) 为该来源实际与已选预测完成程度的最大值；按必要来源数平均作为该单元质量改善，并逐来源判断是否达标；不得先用融合后的 C_i 把另一来源仍未兑现的收益归零。多视角要求按覆盖单元与方向组合记录，实际到达一个方向不能完成其他 required_views。选择候选集合时，暂存预计质量的逐单元最大值，再计算新增候选的边际提高量；重复覆盖不相加。实际账本只接受真实观测，候选选择的暂存质量不能写入已完成状态。

选取多个候选时，对已选集合 S 计算边际增益 G(q|S)。使用预计覆盖的并集更新剩余集合，不能直接相加多个 viewpoint 的原始收益。

未知遮挡后方不直接计入确定收益。第一版沿射线统计最先遇到的未知边界；潜在门口/通道拓展价值属于高层调度优先级，不计入已经能够直接观测到的面积。

路线效率评分：

$$
U(q)=G(q)/(T_{\mathrm{travel}}+T_{\mathrm{turn}}+T_{\mathrm{observe}}+\epsilon).
$$

安全与可达性先筛选；等待时间、区域公平性、切换滞回作为调度规则，不混入“地图信息已经增加”的定义。

到达后依据真实观测更新 C。盲区、未转到所需朝向、深度无效和实际遮挡都不能标记完成。Viewpoint/Session 记录 predicted_gain，CoverageEvaluator 记录 realized_gain 与证据原因。当前默认参考尺度均为 1；若按类别使用不同尺度，预测与实际比较必须采用同一尺度，不能混比。

有效、经过校准的最大量程无回波可以确认其射线通过的自由空间，但不能确认表面；缺失回波、丢束、无效深度或投影空像素仍是未知。既有点云如果没有原始束/无回波元数据，就不凭空推断这些射线。

## 9. Opt：观测目标自适应调整

ObservationGoal 至少包含：

    goal_id / goal_version
    original_pose
    region_id / frontier_ids
    sensor_mode / required_views
    allowed_translation / allowed_yaw_change
    minimum_gain / required_coverage_units
    map_epoch / coverage_revision
    completion_policy = OBSERVATION_CONFIRMED

Opt 只在允许范围内修正位姿，保证目标所在区域关联和可观测方向。

调整结果必须重新执行：

1. 已知空间可达性与足迹/净空筛选。
2. 相机或 LiDAR 可见性检查。
3. 对 required_coverage_units 的覆盖与最低收益检查。
4. 路线和目标切换代价评估。

如果安全可停靠但看不到所需 frontier，则该适配无效。

UNKNOWN_APPROACH 在探索中的含义是“到达已知安全的观测位姿获取信息”，不能直接将未知原始点作为已经验证的执行目标。

路径临时受阻保持探索目标与区域意图，先重新规划或恢复；只有目标自身的几何/观测条件失效才重新选择目标。到达适配位置也不能替代实际观测确认。

点到点任务与探索共用目标适配几何，完成政策不同。点到点任务在已确认障碍占用时，可按用户允许的区域替代政策搜索邻近位姿；探索任务必须保留 required_views、frontier 关联和最低收益。单个 free 栅格也不证明整个位姿足迹安全、全程可达或任务语义有效。

## 10. DeFM、SanD 与三模态训练

DeFM 为共享冻结深度编码器，每个新视图只编码一次；缓存历史特征、时间戳、采样位姿和有效掩码。

- RGB-D 输入原生透视深度。
- LiDAR 输入由点云生成的必要虚拟透视视图。
- 联合模式融合当前执行需要的视图，并带 sensor_id、view_id、modality_mask 和时间信息。
- 点云投影保留有效掩码，禁止将未命中的投影像素解释为 free。
- 相机轴向 Z 与 LiDAR 射线距离按对应模型转换；不能对量程图简单拉伸后宣称与相机深度同分布。
- 几何覆盖和 Safety 始终保留原始观测，不能只依赖投影图或 DeFM tokens。

几何连通区域、地面和净空来自真实深度/点云证据。DeFM 是特征编码器，不能直接当作“桌子、床、危险区域”等类别识别服务；任务需要类别语义时另设经过训练与验证的解析头/外部服务。第一版完整性仅针对已定义的几何覆盖任务。

SanD 原论文使用历史深度、目标和上一规划条件。本方案更换为 DeFM，并新增 LiDAR/联合条件，属于需要训练和验证的适配，不能直接加载原策略权重后宣称兼容。

第一版采用一个共享的模态感知条件融合网络与去噪头，使用统一动作、时间和掩码契约进行三模态训练；不要求维护三套完整策略。是否增设轻量模态投影或分头，由单模态消融结果决定。LiDAR 虚拟视图的内参、外参、视图朝向与有效区域随 tokens 一起进入条件，不能将不同投影约定的数据混在同一个无标识条件中。

共享头的数据契约包括：当前与历史视图及其真实年龄、内外参和来源掩码，局部路径引导与观测目标，预计切换状态，以及上一条实际执行参考的剩余段。上一规划 token 是历史条件；当前位置、速度和加速度约束来自经校验的执行反馈/预测。扩散的 v-prediction 与机器人速度使用不同字段，避免符号相同造成接口混用。

训练原则：

- DeFM 冻结。
- VistaPlanner、覆盖策略和 Opt 几何核心不需要学习训练。
- 训练轻量特征投影、条件融合和 B 样条去噪头；深度投影及官方预处理按固定几何/度量规则执行，不作为学习模块。
- 数据同时覆盖三种模式，包含 LiDAR 稀疏投影、RGB-D 孔洞、异步观测和模态缺失。
- 使用模态随机缺失训练，训练与部署保持一致的掩码和投影规则。
- 对三种模式分别验证，不能用联合模式表现替代单传感器验收。
- 若配置明确允许几何控制备用后端，后端切换仍遵守同样轨迹与安全契约。

训练与部署一并锁定 DeFM 权重校验值、官方预处理版本、视图投影配置、token 维度、控制点及结点配置、时间单位和条件字段。先缓存冻结 DeFM 特征训练投影/融合及扩散头，再做包含闭环偏差、掉帧与切换状态的数据补充。专家轨迹按同一物理时间定义拟合为 B 样条，保存拟合误差与不可行样本的处理依据；不能从缺少时间戳的路径直接生成速度监督。训练可以加入动力学或连续性损失，执行端仍做独立硬约束检查。

## 11. 平滑执行与独立 Safety

### 11.1 三层平滑约束

全局目标层：目标保持与切换滞回，减少无效来回转向，但不阻止真实地图变化触发改道。

轨迹层：固定数量 clamped cubic B 样条控制点，显式时间参数化。重规划切换时匹配反馈预测位置、速度和加速度：

$$
p_{\mathrm{new}}(0)=p_s,\quad
\dot p_{\mathrm{new}}(0)=v_s,\quad
\ddot p_{\mathrm{new}}(0)=a_s.
$$

跟踪层：使用连续 odom 坐标系执行，地图修正通过目标/路径变换处理。平台足迹、非完整约束、速度、角速度、加速度和 jerk 上限由抽象平台配置提供。

几何参数 u 不等于时间。热启动提高一致性但不是边界约束。三次样条内部 C2 连续不代表 jerk 连续；要求 jerk 连续时需要更高阶轨迹与连接约束。

三次样条默认采用内部简单结点；重复内部结点会降低连续阶数。第一版可配置 N=8 个控制点作为起步参数，该数值是工程选择，需按通道复杂度与端侧预算验证。沿用相同结点与基函数时，控制点扰动对曲线位置的凸组合上界只限制表示误差，不能证明碰撞安全、动力学可行或真实跟踪误差。

控制顺序固定为：确定执行切换时刻与候选时间尺度，构造/投影起始位置、速度、加速度约束，检查平台运动学和导数上界，最后验证整个候选。如果之后改变时间尺度或控制点，重新检查边界和 Safety，不能在验证后再随意拉伸轨迹。

边界约束使用实际结点向量与时间尺度的样条导数矩阵，求解 A_boundary Q = b_switch。N=8 的 clamped cubic B 样条一般不等于单段三次 Bézier，不能直接套用单段 Bézier 控制点公式。轨迹交接绑定 execution_switch_time 和 reference_version；若实际交接时间变化，则重新预测切换状态、投影和验证。平滑验收区分参考导数连续性与实际底盘反馈的速度、加速度和 jerk。

模型输出位置轨迹不自动满足底盘非完整约束。差速平台需验证曲率、线速度/角速度与转向可执行性；轨迹跟踪器按平台接口实现反馈控制。策略目标还包括正常停车的连续衔接；安全停车和急停的优先级高于平滑指标。

### 11.2 Safety 验证

执行候选必须通过：

- 沿轨迹的机器人扫掠体检查，而不只是机器人中心 clearance。
- 垂直净空、地面连续性和可用数据范围检查。
- 与观测年龄、定位误差、地图分辨率相关的安全裕度。
- 对动态障碍的执行时间/短期预测检查；静态 ESDF 本身不足以提供动态安全。
- 时间参数化、边界投影和候选选优之后的最终复核。

离散轨迹采样不能自动成为连续碰撞证明。使用自适应细分或距离场变化上界保留采样间隔裕度，并计入地图误差；若无法满足则否决。

轨迹过期、新传感器模式缺少有效条件、所有候选不安全或定位失效时，不执行未经验证的新参考。依照仍有效的安全空间进行受控停车；急停允许优先牺牲平滑性。原地旋转也要验证旋转扫掠体，不能在全盲状态无条件旋转。

Safety 的硬约束是对输入证据和误差假设的拒绝/放行契约，不等同于真实世界绝对无碰撞保证。透明结构、未观测地面、传感器盲区或未知动态行为无法从不足的数据中证明安全；应停止未验证运动并报告无法确认。

执行速度还需受已验证可停车空间约束。可用简化制动模型审查 D_verified >= |v| tau_total + v^2/(2 a_brake) + d_buffer；a_brake 必须是平台在当前条件下可兑现的保守制动能力，tau_total 包括观测、推理、通信和执行延迟。转弯、反向或模型不适用时直接验证完整停车轨迹扫掠体。传感器视野不足以验证停车区域时降低速度或停止；不能把学习输出的最高速度当作可执行速度。

## 12. BT 编排与状态机

行为树使用 autonomy/task/behavior_tree/plugins/bt_node_base.hpp 的既有 native 生命周期及取消机制。探索专用 Action 与 Condition 放入 task/exploration/plugins，沿用 task/navigation/plugins 的职责分组；通用 Control 与 Decorator 复用公共目录。

建议新增的语义节点：

- SensorReady：有效观测、定位和所需控制后端是否齐备。
- UpdateExploration：消费地图差分、观测和实际覆盖结果。
- SelectObservationPose：调用策略选择目标，不发底盘命令。
- AdaptObservationPose：调用 Opt 并复核收益。
- ObserveAtPose：满足朝向、停留和有效观测要求后才返回 SUCCESS。
- CheckExplorationCompletion：执行完成确认协议。
- ReturnToStart：通过同一导航提交通道返航。
- SaveExplorationMap：通过现有建图任务入口保存并等待完成。

原生节点生命周期规定：SensorReady、CheckExplorationCompletion 使用 BtCondition，只读取已确定状态，返回 SUCCESS/FAILURE；UpdateExploration 使用 BtSyncAction，只做有界队列消费；选位姿、适配、观测、返航、保存使用 BtStatefulAction，耗时操作在工作线程执行，等待期间返回 RUNNING，OnHalted 取消该版本请求。采用现有 BT_REGISTER_NODES 静态注册入口，不引入 ROS action client。

### 12.1 阶段请求与唯一提交入口

原生任务使用 BT 驱动模式。UpdateExploration 消费输入并更新状态，SelectObservationPose 只完成策略选择，AdaptObservationPose 只完成 Opt 调整和复核，ObserveAtPose 才请求导航/观测执行。Session 不在选择完成后自动跨越适配和导航阶段。独立算法测试可以保留明确配置的自动驱动模式；同一会话的两种模式互斥。

每个 Stateful 节点持有 Session 返回的阶段请求凭据，至少包含 session_id、request_id、phase、goal_version 和取消所有权。重试初始化时复用本次请求；正常重复 tick 查询其结果；OnHalted 只取消该凭据。结果区分 PENDING、SUCCEEDED、RETRYABLE、INVALIDATED、FAILED、CANCELED。候选耗尽、分轮扫尾未结束和临时受阻分别进入其对应会话状态，避免由一项 FAILURE 直接误报不可达。

原生 BtStatefulAction 的 onStart 首次返回 RUNNING，后续 onRunning 才调用 OnFirstTick；实现和测试以真实基类为准。耗时查询在工作线程完成，回调只投递事件。UpdateExploration 有单轮处理上限，原生任务还持续推进会话监测，确保导航运行、暂停和收尾期间的健康与取消都得到处理。

| BT 节点 | 所请求的操作 | 成功依据 |
|---|---|---|
| SensorReady | 读取健康、定位与控制后端状态 | 当前必需链路满足运行条件 |
| UpdateExploration | 有界消费地图/观测/执行事件 | 本轮消费正常，剩余输入保留 |
| SelectObservationPose | 异步策略选择 | 有版本化候选与观测要求 |
| AdaptObservationPose | Opt 适配及重新评估 | 安全、可达、必要覆盖与最低收益通过 |
| ObserveAtPose | 通过 Session 提交导航并兑现观测 | 必要朝向、有效独立观测和停留条件满足 |
| CheckExplorationCompletion | 读取会话完成确认结果 | 完整扫尾、正常输入与同步截面均通过 |
| ReturnToStart | 经同一导航入口返航 | 收到对应请求的实际到达确认 |
| SaveExplorationMap | 既有建图入口刷新/保存 | 收到可追溯保存结果，且更晚证据未撤销完成 |

### 12.2 生命周期与恢复

BT 名称保留 SensorReady、UpdateExploration 等任务语义；C++ 类与源文件沿用 SensorReadyCondition / sensor_ready_condition.cpp、SelectObservationPoseAction / select_observation_pose_action.cpp 的现有风格。节点命名空间为 autonomy::task::plugins::exploration；策略核心仍为 autonomy::task::exploration。

导航动作复用既有 NavigationTask 与 Autolink 调用，由 ExplorationSession 的统一提交入口派发。八个探索节点是新增的核心语义节点集合，并不表示所有导航、健康监测、预算、恢复和控制节点都需要重写。BT 不能一边调用导航节点直接提交、一边让 Session 再提交相同目标。

CheckExplorationCompletion 的 FAILURE 只表示当前不能确认完成，不表示应该把任务报错；VERIFYING_COMPLETION 的多轮确认由会话推进。SelectObservationPose 的候选为空也不直接等于完成，应区分扫尾待处理、临时受阻、预算不足和无法观测。

导航运行时监测、预算检查、目标变化和传感器降级在执行期间持续 tick，不能只放进 FollowPath 失败后的 recovery 分支。

语义流程：

    初始化 / 建图就绪
      -> 更新证据
      -> 完成或预算检查
      -> 选观测位姿 -> Opt 适配 -> 提交既有导航
      -> 导航中持续监测 / 重规划 / 有界恢复
      -> 到达 -> 实际观测确认 -> 下一轮
      -> 无剩余任务 -> 完成确认
      -> 返航 -> 最终地图刷新 -> 保存确认 -> 输出结果

内部状态：

IDLE、INITIALIZING、EXPLORING、OBSERVING、REPLANNING、RECOVERING、WAITING_FOR_SENSOR、PAUSED、VERIFYING_COMPLETION、FINISHING、RETURNING、SAVING、FINISHED。FINISHING 表示已进入收尾候选，等待 BT 显式请求返航/保存；不等于任务成功。

取消首先取消当前导航与控制执行，不自动发起新的恢复运动。暂停与恢复保留覆盖记录，但恢复必须重新检查位姿、地图和传感器状态。

暂停保存当前阶段及其请求版本。OBSERVING 恢复后继续验证所需观测，SAVING 恢复后核验原保存请求或重新发起有版本的保存；不能统一恢复为 RETURNING 并产生额外运动。取消旧节点只能取消它所持有的请求，不能误取消已被新节点接管的导航。

参考中的恢复逻辑需要纠正：恢复不作为正常 Sequence 的最后一个必执行子节点；接近重评估不只挂在 FollowPath 的失败分支。导航运行时的目标状态变化和预算/感知检查必须持续执行，成功到达后不触发无理由的 Spin/BackUp。

### 12.3 当前原生树与任务结果契约

ExplorationTree 已使用以下 XML 编排八个节点。它是当前可核验的原生树结构；导航、健康、超时和有界恢复由 Session 阶段及其服务入口推进。owner 生命周期有 §20.13 的隔离专项与 §20.14 的正式同步、原生构建及 CTest 证据；真实依赖配置、跨进程导航/建图服务和完整任务测试仍待完成。

```xml
<root BTCPP_format="4" main_tree_to_execute="Explore">
  <BehaviorTree ID="Explore">
    <ReactiveSequence>
      <UpdateExploration/>
      <Fallback>
        <Sequence>
          <ScriptCondition code="exploration_finishing"/>
          <Fallback>
            <ReturnToStart/>
            <AlwaysSuccess/>
          </Fallback>
          <SaveExplorationMap/>
        </Sequence>
        <Sequence>
          <SensorReady/>
          <Fallback>
            <CheckExplorationCompletion/>
            <Sequence>
              <SelectObservationPose/>
              <AdaptObservationPose/>
              <ObserveAtPose/>
            </Sequence>
          </Fallback>
        </Sequence>
      </Fallback>
    </ReactiveSequence>
  </BehaviorTree>
</root>
```

UpdateExploration 在每次根 tick 先更新输入，保证执行与保存期间仍处理有效事件。ReturnToStart 的失败可以进入保存当前部分地图的分支；该失败必须保留在 return_outcome，AlwaysSuccess 不得将它改为返航成功。根节点 SUCCESS/FAILURE 只表示本轮树的状态，ExplorationTask 必须读取 Session 的 exploration_outcome、return_outcome、map_save_outcome 及配置要求决定任务结果。

本地测试已核验真实基类的首次 tick、独立阶段、重复 tick 不重复提交、实际观测/停留、取消所有权、过期位姿、旧节点 halt 不取消保存以及整树收尾。持续流入的多传感器数据、全部阶段暂停/恢复、真实导航失败和保存服务故障仍属任务级专项验收，不能从上述六组场景推断已经覆盖所有组合。

### 12.4 真实 ExplorationTask 接管与旧入口收敛

最终接入保留 TypedTaskAppBase 的公开任务 API。ExplorationTask 拥有一条 Session/ExplorationTree 工作线程；Start、Tick 和阶段请求在该拥有者线程执行。传感器、地图、任务命令及服务回调仅入队，禁止在回调线程推进 Session。较重的路线/候选计算如需异步，工作线程只处理带版本的不可变输入，结果回到拥有者线程核验后提交；BT 节点不得另起一套会话状态。

TaskServer 负责原生任务注册、分发与依赖注入。ExplorationTask 与 TaskServer 中原有的 exploration waypoint 直提交路径须统一到 Session；旧 finished Bool 只能作为兼容事件，不能直接设置探索成功。选择与 Opt 适配阶段不提交导航；ObserveAtPose 和 ReturnToStart 经同一个提交入口执行，并分别持有版本凭据。任何导航结果必须与本次请求关联，不能仅凭 navigation_active 从 true 变为 false 判断到达。

Pause、Resume、Cancel、Shutdown 均通过命令队列和明确的请求取消推进。任务反馈及结果读取一致状态快照；不得持生命周期 mutex 等待工作线程 join，也不得在取消过程中接受晚到的旧导航成功。Shutdown 先停止新请求、取消当前版本、唤醒并结束工作线程，再释放树和服务句柄。暂停不兑现观测停留，恢复前复核地图、定位、健康与原阶段。

现有 SAVE_MAP 向 MappingTask 转发 MAP_CMD_SWITCH 的行为需要接到真实保存能力；加载/切换名字也不证明地图已载入或融合。新增请求/回执语义限定在任务协议及上游建图适配，autonomy/map 代码、接口和消息保持不变。上游未提供真实融合或保存依据时，任务返回未确认/失败结果，并保留有效部分地图，不由客户端生成成功 ACK。

真实任务接入的验收包括：单一导航提交者、START 抢占与晚到回执、各阶段暂停恢复、取消/shutdown、真实观测到达确认、跨进程路线调用、融合/保存失败及完成结论撤销。当前八节点的原生专项回归是这项集成的基础证据。本地接管原型已有代码，尚有第 12.5 节的阻断项，完整任务接管仍待修复和验收。

### 12.5 任务接管的执行门控与不可阻塞契约

以下是最终方案必须满足的约束；当前本地接管原型尚未全部满足。

- **取消与抢占**：generation 相同只说明身份仍匹配，不能证明任务仍允许运动。factory 返回、Session::Start、阶段提交及导航派发前均需同时核验任务身份、运行意图和 motion permit。取消后晚到的构造/计算结果只能丢弃，不能再启动运动。新 START 清除旧 pause/resume 的延迟命令及旧观测暂存。
- **暂停**：暂停立即撤销新运动派发资格；暂停期间的新观测可以按有效性保留，停留计时不能继续兑现。恢复重新校验地图、位姿、传感器、阶段及参考版本。
- **异步导航**：原生导航 Submit 可能等待 readiness，禁止让探索 owner 同步等待。使用独立、有界、可取消的派发入口，区分排队、原生接受、实际到达与观测兑现。派发资格与原生接受需有明确的原子交接或接受后按 ID 取消的兜底；只在调用前检查一次不能消除竞态。
- **对象关闭**：线程对象的创建/读取/join 在统一的同步规则下处理；用 inbox 保护的 worker_started 和已缓存 owner ID 判断状态。禁止持生命周期锁 join，禁止依赖回调与外部 join 相互等待。Tick/Halt/取消抛异常后仍须在 owner 上释放 Tree/Session，并保留失败结果。
- **证据队列**：入队成功不等于覆盖兑现。主队列、延迟队列和 Session 队列各有容量、拒绝语义及 accepted/processed/discarded 统计。暂存满时不能静默丢掉已接受的覆盖要求；批次有缺失或处理失败时禁止完成确认。
- **依赖安装**：SetNode、SetSubmit、版本化结果/取消服务仅在 idle/terminal 且无在途派发时安装；注释中的 idle-only 必须落实为代码检查。缺少真实 bindings 时 START 返回明确拒绝或初始化失败，不生成模拟观测。
- **跨重启身份**：请求身份包括进程启动身份和 task/session/request 代数。GetResultForTask 在同一生命周期锁内检查 task ID 与终态；CancelForTask 只能取消匹配的原生请求。任务替换与结果发布也需同一身份规则，不能仅修复读取端。
- **范围及保存命令**：最终契约区分任意允许多边形、全量覆盖与显式部分覆盖政策。手动 SAVE_MAP 是不终止探索的版本化 checkpoint；最终保存绑定完成截面。当前原型只支持轴对齐矩形、拒绝部分覆盖与手动 checkpoint，这些属于未实现能力，不能标为兼容完成。

接管专项须验证：factory 阻塞时 cancel；排队到接受之间的 cancel/preempt；START 抢占后的迟到到达；所有阶段的暂停恢复；依赖回调重入 shutdown；延迟队列饱和；任务/进程重启旧回执；没有真实融合或保存服务时绝不成功。只有源码存在或旧六项专项通过均不满足这些条件。

### 12.6 父任务调度、原生接受与关闭顺序

探索占用原生任务调度器的运动资格，导航与返航是该探索会话委托的子执行。子执行继承父任务身份及可撤销运动许可，由调度器显式识别为委托请求，不再申请一个与父任务竞争的独立运动槽。普通外部 Navigate START 仍按既有抢占政策处理：撤销父任务资格、取消其匹配子请求并确认交接，再允许新的运动任务。不能直接绕过调度器调用导航，也不能让子导航自我抢占父探索。该规则需要适配真实 TaskScheduler 的激活及取消语义，当前尚无这项接入的验证记录。

原生接受契约区分 QUEUED、SUBMITTING、ACCEPTED、TERMINAL 和撤销意图。运动许可在 readiness 等待期间、任务身份提交时、树启动前和实际执行入口持续检查；task_id 只有真正接受后才与新任务状态共同提交。同位姿的新 task_id 仍是新的请求，不能因位置相同就读取旧任务结果。

异步派发器在 Submit 返回后，必须在同一队列锁保护的状态交接中重新检查请求 live、当前 pending 身份、stopping/failed 与运动许可，并明确原生取消责任。若取消发生在 Submit 期间，提交线程负责接受后按 ID 取消；若接受已经公布，取消入口接管该责任。实际 native Cancel 在锁外执行，失败则永久禁止该代新的运动。不得先在锁外缓存 revoked=false，再在锁内公布 accepted=true，因为这会漏掉二者之间发生的取消。仅增加异步线程不能满足这项契约。

依赖关闭顺序是：停止各外部入口接收新请求，撤销所有相关派发资格，排空匹配的原生取消并停止提交者，结束探索 owner 并在其线程销毁 Tree/Session，然后关闭被调用的导航及建图服务。若某提交者正在 readiness 等待，通过其可撤销许可及停止令牌打断；不能先销毁 NavigationTask 再等待仍持有它的探索派发器退出。回调中的 Shutdown 只请求关闭，线程销毁与 join 由外部拥有者完成，不能在自己的 worker 上销毁仍执行的 Impl。

验收须包含：父探索委托导航不会被自身抢占；外部导航替换期间无双重运动；接受前、接受交接窗口和接受后取消均覆盖；暂停恢复不复活旧 lease；原生接受后抛异常仍取消；取消失败后无新 START；callback 重入关闭与多个外部关闭调用不死锁。最终方案固定以上行为，源码存在和旧专项结果不算这项验收通过。

### 12.7 初始化、暂停恢复与专项证据

建图准备是当前任务代数的初始化前置条件。暂停可以延后尚未开始的准备，但恢复后必须先补齐准备再启动 Session，不能因为旧 Begin 分支被跳过而永久绕过它。记录准备结果及其绑定的 generation；取消、抢占和过期返回使旧结果失效。回调在锁外执行，返回后复核当前身份和运行意图。

建图准备拒绝、种子地图/位姿超时、factory 异常或不合法返回必须进入一致的失败清理：撤销许可、取消并确认在途原生工作、在 owner 释放候选及 Session/Tree、清空本代暂存，完成后释放 runtime_busy。若取消或停止无法确认，保留明确的 motion fault 并拒绝后续运动。仅清空 waiting 或发布 FAILED 不代表清理完成。

Session 的 Start/Tick 与树生命周期由 task owner 管理；策略计算及其 Reset 可以由 Session 专属规划工作线程管理，这两类线程证据不能混用。当前 Session 在首个对应版本的规划工作中调用 strategy Reset，该回调不证明 Session::Start 被调用，也不应被测试误判为 owner 线程违规。测试需分别验证 Session/Tree 的生命周期、策略工作线程的生命周期和实际运动接受；通过受控回调确认时序，禁止以猜测的线程延迟或错误计数器替代证据。

2026-11-18 owner 测试已修正上述启动/线程证据，并扩展到十三类：factory 暂停/取消/抢占、建图准备恢复与失败、种子超时、cooperative 服务缺失、最终时钟回调期间暂停/取消、暂停初始化预算与覆盖监测、提交晚接受及回调重入关闭。新 owner、Session 和测试先在隔离 staging 编译、链接真实原生库，十三类通过；之后经过逐文件冲突核验、备份和正式同步，纳入原生构建及八项 CTest，8/8 通过。旧失败与修复后的日志分别保留。原生 NavigationTask 的实际门控、全部业务阶段暂停恢复及父子任务调度不在这项通过范围内。详见 [owner 生命周期说明](exploration_task_owner.md) 和 §20.14。

### 12.8 原生执行前检查：已实现范围与派发边界

BtRunnerHooks 新增 BeforeTick；BtTaskApp 将其转发到派生任务 BeforeTreeTick。每次未暂停的 tree.tickOnce 之前检查，返回后重新检查取消/暂停。拒绝执行使该轮真实 Script/Action 效果为零，worker 负责 halt。默认任务继续获得兼容的 true 返回值。

取消、暂停和恢复通过条件变量及在等待 mutex 下维护的代数唤醒循环，避免长周期等待及边界丢通知。worker 回调重入 Cancel 只撤销，不 self-join；Configure/Run 拒绝重入。setup、检查、节点、监测、日志及 halt 的异常有明确失败结果；取消不能掩盖已发生的异常。IsRunning=false 在树、logger 清理及最后 host 回调之后发布。回调仍必须有界，任意阻塞的第三方接口不能靠通知强制中断。

[BT 生命周期说明](native_bt_admission.md) 与 [隔离 native 专项](validation/2026-11-18-v2.10-bt-stage.json) 记录十一类真实 XML/原生节点效果场景。新增虚函数必须重编相关 native task ABI。正式部署后的 planning/task/control 与相关测试已重编，九项相关 CTest 9/9；见第 20.15 节证据。

这不是实际运动资格接入完成：Pause 只暂停树，不自动停止已运行的异步控制。NavigationTask 的 readiness/接受，以及 ActionSession 实际 dispatch 仍要使用同一可撤销 lease，处理检查与派发之间的竞争、晚接受及取消确认。父探索/委托导航运动槽及依赖关闭仍按第 12.6 节接入；不得用 BeforeTick 专项替代这些链路。

## 13. 完成协议、预算与结果

探索范围定义为：所选分辨率、允许区域和配置传感器模式下，与起点连通且机器人可到达、传感器可观测的空间。

完成确认必须同时满足：

1. 没有仍然可达、可观测的待探索 frontier。
2. 没有尚未兑现的观测任务或到期重访任务。
3. 地图差分、图更新和观测队列处理到一致 watermark。
4. 连续多个确认周期没有新增 frontier，且输入不是因感知失效而为空。
5. 已对稀疏图失败和小收益遗漏执行栅格备用检查及扫尾。
6. 无法确认、不可达和模式盲区形成独立清单。

完成是“已证明配置范围内的可达/可观测工作已处理”。仍有可能存在可达空间但因未知、定位问题、未评估区域或感知失效无法确认时，使用 INCOMPLETE_OBSERVATION，不使用 COMPLETED。与配置范围明确无关或有充分依据不可达的区域可列入排除清单；排除理由必须可审计。

覆盖结果与任务收尾结果分开：

    exploration_outcome:
      COMPLETED
      PARTIAL_BUDGET
      INCOMPLETE_OBSERVATION
      NO_REACHABLE_VIEW
      CANCELED
      FAILED_LOCALIZATION

    return_outcome:
      NOT_REQUESTED / RETURNED / RETURN_FAILED

    map_save_outcome:
      NOT_REQUESTED / SAVED / SAVE_FAILED

任务 success 只在配置要求的覆盖、返航和保存条件全部满足时成立。部分地图可以保存，但不能伪装为完全探索成功。

如果 lidar_rgbd 配置要求两个传感器各自的质量目标，降级后可能只完成几何目标，未完成 RGB-D 质量目标必须保留为部分完成。不能通过降低目标定义让失败模式突然“完成”。

使用时间/距离预算时持续预留返航成本和配置裕度；无电池遥测时不宣称保证电量充足。取消是否保存当前图由明确任务策略决定，不产生额外隐式运动。

返航成本通过同一 Vista 路线查询得到实际可通行路径长度，必要时用 TraversalGraph 与已知空间栅格复核，不使用到起点的欧氏直线距离替代。下一项任务必须满足 T_remaining >= T_next + T_return_from_next + T_margin，并满足对应距离预算。地图变化使返航路线缓存失效时先重算；返航连通性无法确认时停止新增向外探索。历史路线支持远程估算，真正执行的近端区段仍需要新鲜安全证据。

完成确认是可撤销的阶段状态。返航、最终刷新或保存结束之前若出现新的有效 frontier、必须观测任务或地图对齐变化，撤销旧完成结论；有预算时恢复探索，否则保留 PARTIAL_BUDGET 或 INCOMPLETE_OBSERVATION。最终记录包含覆盖截面、地图保存版本与各子结果，保存 ACK 不能覆盖更晚出现的未完成证据。

### 13.1 可审计完成凭据

确认轮次必须处理新的有效观测，并覆盖当前要求的传感器来源；重复 tick、同一帧重放或全盲期间的空队列不增加稳定轮数。扫尾结果绑定 geometry_revision 与 coverage_requirement_revision。持续地图变化时保持分页进度并重验受影响项，完成之前证明当前版本下没有漏评估项。

保存阶段继续消费健康、地图和覆盖事件。Session 在同一拥有者线程核验保存 ACK 与最新输入后提交最终结果，检查期间失去定位/必要感知、新 frontier 或 epoch 改变均撤销旧完成候选。ACK 只能证明对应保存请求及地图来源，不能覆盖更新的未处理工作。

任务结果附带消费者侧 CompletionEvidence：配置边界与覆盖要求版本、地图 epoch/几何版本、各来源已接受/已处理/已丢弃截面、可见图及覆盖处理截面、完整扫尾编号、稳定观测轮数、上游融合同步依据、未完成/排除清单、返航结果及保存来源。丢弃观测单独记录原因；它不等于已获得覆盖证据。缺少上游融合确认时保留 INCOMPLETE_OBSERVATION，不自行生成不存在的 map ACK。

返航路径丢失时先复核已访问连接与已知空间栅格路线；仍不可达则返回 RETURN_FAILED，并按配置保存当前部分地图。保存只有现有建图入口返回成功确认才算 SAVED；提交请求和最终地图刷新开始都不算完成。

只有在有限、最终稳定环境、可靠定位、足够预算、覆盖充分的观测采样和可达搜索等条件下，才能论证分辨率意义上的完成。没有已知建筑边界或真值地图时，不输出无依据的整栋建筑 100% 覆盖率。

### 13.2 持续输入下的有限截面与融合确认

完成确认不能要求所有模块每次都等于不断增长的 latest_accepted 序号。RGB-D 与 LiDAR 持续输入时，这会使检查永远追不上输入。采用以下消费者侧协议：

1. 进入 VERIFYING_COMPLETION 时，拥有者锁定 completion_cut_id，以及此刻各来源已接受的序号向量 cut={lidar:n_lidar,rgbd:n_rgbd}。未配置的来源不要求参与；已配置但失效的必要来源不能被静默移除。
2. 覆盖、地图读取差分与 Vista 更新分别报告这个有限输入前缀的处理结果。使用逐来源前缀的覆盖关系，不比较两个来源的序号大小，也不强求持续变化的最新序号完全相等。序号空洞与丢弃有独立记录，不能当作覆盖已兑现。
3. 通过 map 外的建图任务适配请求真实上游融合屏障。回执至少能关联建图会话、坐标版本、来源截面和所确认的地图版本。适配层只转译真实上游能力；单纯延时、队列为空、消息已发布或静态地图读取成功均不能代替融合确认。
4. 对屏障返回的地图版本完成差分、图更新及当前覆盖要求下的完整扫尾。稳定确认轮需来自新鲜、独立且必要来源齐备的有效观测；每轮锁定自己的有限截面。仅图或覆盖消费者追上截面，仍不等于上游融合完成。
5. 截面之后的观测继续处理。重复已确认几何且没有新增要求的正常帧不自动重启扫尾；出现新 frontier、未完成质量/方向要求、相关几何变化或 epoch 改变，则撤销受影响的完成候选并推进新的截面。处理预算不能吞掉这类变化。
6. 保存请求绑定已确认的建图会话及地图版本；保存回执在拥有者线程与最新已接受证据复核后提交。最终结果说明覆盖与保存各自对应的截面；不承诺已经融合提交之后无限到来的未来观测。

融合屏障与实际保存能力是严格完成的接入前提。若上游能力尚不可用，保留 INCOMPLETE_OBSERVATION / SAVE_FAILED 或相应未请求状态；不能由客户端生成一个“已融合/已保存”的成功回执。有限截面解决处理活性，不提供本来不存在的上游能力，也不保证持续动态环境最终终止。

专项验收需包含：双来源不同频率持续输入仍可在稳定几何下完成；低频必要来源缺失不能完成；序号空洞与超龄丢弃不伪造覆盖；屏障之后出现新工作撤销结论；新健康帧不使完整扫尾无限重置；迟到/旧会话融合与保存回执被拒绝。这些契约尚无本次任务级通过记录。

## 14. 并发、版本与实时性

异步结果元数据至少包括 task_boot_id、task_generation、session_id、request_id、goal_version、map_epoch、geometry_revision、sensor_epoch、coverage_requirement_revision 和 valid_until。task_boot_id 区分进程重启；进程内递增计数不能独自防止跨重启旧回执与新任务 ID 重合。字段位于任务/消费者协议，不扩展 map 消息。

收到结果时验证会话、目标和对齐版本。地图内容更新不必一律丢弃所有路径，但必须复核剩余路径与新版本是否相容；坐标对齐 epoch 变化必须重新变换/规划。

初始调度目标，均为设计预算，尚未在 Orin 实测：

- 传感器适配：各自输入频率；每个新帧/扫描一次。
- 覆盖更新：随有效观测和地图变化。
- 全局区域访问排序：低频、事件触发。
- Vista 几何/图更新：随 dirty regions，独立于查询。
- 局部候选和轨迹生成：以 8–10 Hz 为初始目标。
- 执行跟踪：以 30–50 Hz 为初始目标，最终取决于平台能力。

到期保护依据实际 valid_until 与最坏延迟，而不只看平均值或 P95。错过截止时间时使用仍然有效的参考或停车。

仅编码新视图并缓存历史；增量重算受影响 viewpoint；控制学习支路优先编码当前执行相关视图。降采样不能永久丢弃窄通道和小 frontier，必要时自适应细化。

需要随平台冻结的参数包括：足迹/高度与制动能力；地图和覆盖分辨率；标定、真实视野与有效量程；逐来源最大观测年龄和模式滞回；候选采样密度、规划预算及扫尾分页大小；minimum_gain、每方向质量/独立帧数/停留要求；返航裕度、阶段超时和重访规则；样条结点、时间范围、导数限制、Safety 细分误差与参考期限。已有 N=8、8–10 Hz、30–50 Hz 只是起步配置，最终依据设备、数据和闭环验证冻结。无设备规格时不编造统一净空、固定 20% 膨胀或可兑现的制动距离。

## 15. 拟实现目录树

下列树仅列新增或重点扩展的位置。现有 map 不列入改动。

    autonomy/
    ├── common/
    │   └── navigation/
    │       ├── CMakeLists.txt        轻量几何/观测支撑 target
    │       └── 几何核心实现           Opt 内部复用，待统一原生命名
    ├── perception/
    │   ├── data/
    │   │   ├── sensor_observation.hpp
    │   │   ├── observation_buffer.hpp/.cpp
    │   │   └── map_reader.hpp/.cpp
    │   ├── observation/
    │   │   ├── lidar_observation_adapter.hpp/.cpp
    │   │   └── rgbd_observation_adapter.hpp/.cpp
    │   └── inference/
    │       └── defm/
    │           ├── defm_encoder.hpp/.cpp
    │           └── depth_view_adapter.hpp/.cpp
    │
    ├── planning/
    │   ├── planner_server.hpp/.cpp   持有并配置 Vista 实例
    │   ├── planner_autolink_actions.hpp/.cpp  跨进程规划入口
    │   ├── proto/
    │   │   └── vista_planner.proto
    │   └── planner/
    │       └── vista/
    │           ├── vista_planner.hpp/.cpp
    │           ├── native_costmap_reader.hpp/.cpp
    │           ├── visibility_graph.hpp/.cpp
    │           ├── graph_updater.hpp/.cpp
    │           ├── obstacle_contour.hpp/.cpp
    │           ├── edge_validator.hpp/.cpp
    │           ├── route_planner.hpp/.cpp
    │           └── spatial_index.hpp/.cpp
    │
    ├── task/
    │   ├── exploration/
    │   │   ├── exploration.hpp/.cpp
    │   │   ├── exploration_session.hpp/.cpp
    │   │   ├── exploration_context.hpp
    │   │   ├── strategy/
    │   │   │   ├── exploration_strategy.hpp
    │   │   │   └── hierarchical_coverage_strategy.hpp/.cpp
    │   │   ├── coverage/
    │   │   │   ├── coverage_state.hpp
    │   │   │   ├── coverage_evaluator.hpp/.cpp
    │   │   │   ├── surface_extractor.hpp/.cpp
    │   │   │   ├── coverage_fusion.hpp/.cpp
    │   │   │   ├── completion_evaluator.hpp/.cpp
    │   │   │   └── local_coverage_planner.hpp/.cpp
    │   │   ├── frontier/
    │   │   │   ├── frontier_extractor.hpp/.cpp
    │   │   │   └── frontier_queue.hpp/.cpp
    │   │   ├── viewpoint/
    │   │   │   ├── viewpoint.hpp
    │   │   │   ├── viewpoint_sampler.hpp/.cpp
    │   │   │   ├── viewpoint_evaluator.hpp/.cpp
    │   │   │   └── viewpoint_manager.hpp/.cpp
    │   │   ├── region/
    │   │   │   ├── exploration_region.hpp
    │   │   │   └── region_manager.hpp/.cpp
    │   │   ├── routing/
    │   │   │   ├── exploration_route.hpp
    │   │   │   ├── visit_order_optimizer.hpp/.cpp
    │   │   │   └── traversal_graph.hpp/.cpp
    │   │   ├── observation/
    │   │   │   ├── sensor_visibility_model.hpp
    │   │   │   ├── lidar_visibility_model.hpp/.cpp
    │   │   │   └── camera_visibility_model.hpp/.cpp
    │   │   ├── plugins/
    │   │   │   ├── action/
    │   │   │   │   ├── update_exploration_action.cpp
    │   │   │   │   ├── select_observation_pose_action.cpp
    │   │   │   │   ├── adapt_observation_pose_action.cpp
    │   │   │   │   ├── observe_at_pose_action.cpp
    │   │   │   │   ├── return_to_start_action.cpp
    │   │   │   │   └── save_exploration_map_action.cpp
    │   │   │   └── condition/
    │   │   │       ├── sensor_ready_condition.cpp
    │   │   │       └── check_exploration_completion_condition.cpp
    │   │   └── integration/
    │   │       ├── navigation_dispatcher.hpp/.cpp  运动许可与版本化异步派发
    │   │       ├── path_provider.hpp
    │   │       ├── vista_path_provider.hpp/.cpp
    │   │       ├── vista_route_client.hpp/.cpp  拟接入原生规划服务
    │   │       └── goal_pose_adapter.hpp/.cpp
    │   └── behavior_tree/
    │       └── plugins/
    │           ├── bt_node_base.hpp   复用既有基类
    │           ├── control/           复用通用控制节点
    │           └── decorator/         复用通用装饰节点
    │
    └── control/
        └── controller/
            └── policy_controller/
                └── sand_planner/
                    ├── sensor_condition_encoder.hpp/.cpp
                    ├── diffusion_bspline_head.hpp/.cpp
                    ├── trajectory_time_scaler.hpp/.cpp
                    ├── trajectory_validator.hpp/.cpp
                    ├── trajectory_tracker.hpp/.cpp
                    ├── local_esdf.hpp/.cpp
                    └── critic.hpp/.cpp

raw observation 适配与缓存放在 perception；探索中的 observation 只包含对候选位姿的可见性预测，避免混淆职责。

命名遵循已有 snake_case 路径、.hpp/.cpp 和嵌套命名空间。VistaPlanner 的命名空间为 autonomy::planning::planner::vista。TARE/FAR 来源记录在说明和代码注释中，公开模块名描述职责。

需要新增公开状态或请求版本时，扩展任务/控制侧现有 Protobuf 定义；不扩展 map/proto，也不平行建立一套脱离任务调度的控制消息。

### 15.1 原生编译依赖与接入边界

几何和数据契约须形成无环依赖。当前原生接入在 common/navigation 构建 autonomy_navigation_support，编译 Opt 几何、观测适配与轻量 MapReader，导出 perception/data 公共头，只链接 Threads，不依赖 task、DeFM 或 map 实现。真实 NativeCostmapReader 留在 planning 内，由原生 planning 的既有依赖获得 Costmap。planning 链接支撑库；当前 task 的同进程路径适配链接 planning 与同一支撑库。最终独立进程路径适配通过 Autolink 调用规划入口；control 按需链接数据与推理。链接库不意味着可以跨进程使用其对象实例。

当前 Opt 核心仍依赖 opt::navigation 的几何与导航公共头。正式同步前必须把相应源文件、include 路径和链接依赖一起接入原生内部支撑库，或完成等价的原生类型迁移；只复制 autonomy/ 子目录不算完成集成。独立 Opt 的 autonomy/CMakeLists.txt 不能当作远端原生入口覆盖过去。

原生 domain 的递归源文件收集需要显式排除已经进入独立支撑 target 的实现，避免重复链接；perception 原有 inference 排除规则需要为 DeFM 单独接入。探索 BT 实现进入原生 task target 并链接真实 BehaviorTree 基类，独立算法构建排除这些插件源。各 domain 的既有选项、接口及用户修改保留，仅合并必要依赖。

Vista 接入必须同时验证类注册、vista_planner 实例配置、Costmap 初始化、共享图实例和 Protobuf 路径输出。ExplorationTask 保留既有任务提交接口，由 Session 统一提交导航，关闭与新会话竞争的旧 waypoint 自动提交路径。保存适配只使用具有真实处理语义的既有建图结果。

## 16. 落地顺序与验收

阶段 A：统一观测、地图读取、健康与覆盖账本。用记录回放检查三种模式的视野、单位、坐标、重叠去重和真实观测确认。

阶段 B：实现 Vista 的图维护与路线查询。检查窄门、长边穿越新增障碍、障碍移走重连、滚动窗口和回环更新；与全量重建及栅格参考路径比较。

阶段 C：移植 TARE 分层覆盖、局部候选和访问次序优化。接入 Opt 和既有导航通道，先用已验证的控制后端完成任务闭环。

阶段 D：训练 DeFM 条件的三模态 B 样条策略，接入时间参数化、边界约束、独立 Safety 和真实跟踪。

阶段 E：端侧实测、调度与故障切换，完成覆盖/返航/保存协议验收。

计划验证矩阵：

- 每种传感器单独完成任务；联合模式、重叠观测、互补视野。
- 时间错位、外参误差、输入停止、定位停止、恢复和模式抖动。
- 遮挡房间、细小 frontier、低增益角落、窄门、悬空结构、深度失效。
- 新障碍、门打开、路径临时受阻、长期受阻、图连接缺失。
- 回环、坐标变化、滚动窗口、旧请求返回、取消和暂停恢复。
- 预算耗尽、返航失败、地图保存失败。
- 换目标、换传感器、重规划、截止时间错过、正常停车和急停。

主要验收指标：

覆盖：真值支持时的可达/可观测覆盖率、遗漏 frontier、误完成率；无真值时报告终止证据与未确认清单。

效率：相同覆盖目标下的总时间、总路程、重复行走比例、新增有效覆盖/米和/秒；不把提前停止算成效率提升。

增量图：更新影响范围、边正确性、错误断连/漏失效率、更新与查询延迟及图内存。

平滑：轨迹切换处位置/速度/加速度差，实际线/角速度跳变，实际 jerk，以及动态约束违例。

安全与实时：碰撞、净空违例、未验证区域执行、拒绝率、参考过期、P95/P99 延迟与最坏停机响应。

这些都是待实施的验证要求；当前没有实验数据支持性能承诺。

对照实验固定相同地图输入、任务边界、传感器模式、专家监督、覆盖达标条件和 Safety 设置。分别比较完整分层策略与局部 frontier 基线、Vista 增量更新与全量重建、Opt 开关、历史帧数、三模态训练/缺失模态训练、直接回归与扩散样条、边界约束开关。图路径对照只在相同可通行与未知策略下解释；平滑对照同时报告速度、任务用时和覆盖，防止仅靠慢速获得较小 jerk。最终要求本地单元/会话测试、真实原生节点与插件测试、新增代码 Docker 构建、回放/仿真任务和端侧实验各自有记录。

当前实施顺序：独立阶段 API、八个原生 BT 节点、Vista 配置/别名、第 8 节质量正改善收益与 owner 初始化修复已有实现及对应记录；正式 owner 部署后 SpaceHero 八项相关 CTest 8/8。下一步接入第 12.5–12.6 节真实导航接受/执行门控、父子调度与 Autolink 路径提供接口，处理 TaskServer 和旧 waypoint 的竞争提交；实现第 13.2 节有限截面，并验证真实观测、定位、融合/保存、覆盖补全与公平扫尾。随后完成三模态训练、时间样条、三维执行链、任务及端侧验收。任何一层的编译或少量测试通过都不能代替其后一层的证据。

## 17. 审查要点

1. map 的零改动边界是否满足要求。
2. 单传感器定位链是否确实不依赖缺失的传感器。
3. 联合几何收益是否去重，质量权重是否被错误当作概率。
4. TARE 全局/局部分层与访问次序优化是否保留。
5. Vista 的图查询是否复用同一实例，长边和消失障碍是否正确更新。
6. Opt 是否保持观测意图并对适配后的收益重新验证。
7. “到达”“观测确认”“探索完成”“返航成功”“保存成功”是否分开。
8. LiDAR 深度投影与模态适配是否有独立训练和验收。
9. 跨重规划平滑是否由时间与边界约束保证。
10. 感知退化、预算限制和无法确认区域是否有明确结果。
11. 历史覆盖与执行安全证据是否分开，整图时间戳是否错误刷新旧空间。
12. 地图窗口边界、候选/区域计算上限是否被错误当作任务结束。
13. BT 返回值是否遵守生命周期，恢复是否仅在有原因时触发。
14. 设计频率、局部测试、原生 Docker 编译和实机指标是否分别记录。

### 17.1 本稿需要逐项审定的设计决策

本稿提交的是以下默认政策；允许修改参数，但修改任务范围、覆盖要求或模式契约时必须生成新配置版本。

- **范围与结果**：单层、与起点连通的允许空间；按配置分辨率验收几何覆盖。无建筑边界或充分观测时保留未确认区域，不把预算结束当成完全探索。
- **三模式契约**：三种配置共用策略与动作表示；每种模式需独立可用的定位、几何观测及控制后端。配置传感器与本轮有效传感器分别记录。
- **覆盖政策**：默认 GEOMETRY_ANY_SENSOR；同一稳定几何单元去重。双来源质量或多朝向要求分别记账，降级不能自动删除要求。
- **地图边界**：继续使用原生 GridMap/Costmap 与既有建图入口；消费者快照、版本、差分、覆盖账本和局部执行几何放在 map 外。若上游不能提供可信融合确认，严格完成条件保持不满足。
- **增量图与策略**：一个共享 VistaPlanner 图实例，结合 TARE 式全局区域访问顺序与局部边际覆盖选择。长边失效、移障重连和低收益公平扫尾是必需机制。
- **位姿适配**：Opt 只在观测意图允许范围内调整位置/朝向，重新检查 required_views、必要覆盖、最低收益、足迹和可达性；不能仅凭安全停靠判适配成功。
- **学习与执行**：冻结 DeFM，训练一套共享模态条件融合/扩散头，三模式分别验收。使用物理时间样条、切换导数约束、完整扫掠体校验与过期保护。
- **任务与 BT**：Session 是唯一执行提交者；BT 显式编排选择、适配、观测、返航和保存。完成、返航和保存结果分别报告，保存 ACK 需与最终证据版本相容。

以上政策确定后，再依据真实传感器及底盘冻结量程/视野、足迹/高度、制动能力、观测年龄、分辨率、独立帧数、停留时间、返航裕度、阶段超时与轨迹导数上限。本稿不预设缺少设备数据支持的统一数值。

## 18. 主要依据

- FAR Planner: Fast, Attemptable Route Planner using Dynamic Visibility Update，arXiv:2110.09460；MichaelFYang/far_planner。
- TARE: A Hierarchical Framework for Efficiently Exploring Complex 3D Environments，RSS 2021；caochao39/tare_planner。官方实现以全局稀疏、局部密集的分层覆盖为基础。
- arXiv:2110.14573 的标题是 Autonomous Exploration Development Environment and the Planning Algorithms，介绍开发环境以及 FAR/TARE 的集成；不把它误写成原始 TARE 算法论文。
- DeFM: Learning Foundation Representations from Depth for Robotics，arXiv:2601.18923；leggedrobotics/defm。使用官方度量深度预处理；跨传感器特征泛化不等于本系统三模态控制策略免训练。
- SanD-Planner: Sample-Efficient Diffusion Planner in B-Spline Space for Robust Local Navigation，arXiv:2602.00923，重点参考 §IV-A 至 §IV-D；历史深度条件、B 样条扩散与几何候选筛选。

2026-11-17 v2.4 通过 Python urllib 重新读取四篇 arXiv 页面和 FAR/TARE 官方 README，六个来源均返回 HTTP 200；标题、原始来源和 SHA256 保存在 validation/reference_sources，当前核验清单见 validation/2026-11-17-v2.4-review-sources.json。本次网页工具未返回内容，采用上述原始 HTTP 读取作为来源核验。早期访问失败记录作为历史保留；本稿采用锁定的参考基线，不据页面可访问性宣称所有代码或模型已经发布/验证。FAR/TARE 的几何框架与 SanD 的局部表示提供基础；本文的模式切换、完成协议、地图消费者版本、Opt 观测约束和跨轨迹执行约束为本系统的设计补充。

v2.5 再次读取上述六个原始 HTTP 来源，均返回 200；本轮来源记录为 validation/2026-11-17-v2.5-review-sources.json。网页工具仍无可用内容，不制造网页引用或将元数据读取作为全文复现、模型训练及运行证明。

文献版本记录：此前来源记录为 FAR arXiv:2110.09460v3、开发环境论文 2110.14573v1、SanD 2602.00923v1，以及 DeFM 2601.18923v3、修订日期 2026-10-27。此处保留已核对的快照信息，不作本轮最新版本声明。实施时锁定实际论文版本、仓库提交、权重 SHA256 与预处理版本，论文版本不自动决定模型权重版本。

## 19. 关键情况与预期行为

这些情况使用系统可观察的证据触发，物理真值只用于仿真验收。机器人不能读取“目标其实在墙里”等真值标签。

| 情况 | 触发证据 | 规定行为与结果 |
|---|---|---|
| 已知 free、正常观测 | 位姿足迹/净空和已知路径通过 | 选最佳观测朝向、执行、真实观测确认；不能仅按目标栅格完成 |
| 历史 free、当前不可见 | 历史记录可用但局部证据年龄不足 | 历史用于路线与覆盖；新执行区段重新取得有效几何，不刷新旧证据年龄 |
| 目标 occupied、附近有有效替代 | 已观测占用，允许的观测区域内有可见且可达位姿 | REGION_FALLBACK；Opt 复核必要覆盖与最低收益 |
| 目标 occupied、无有效替代 | 足迹/净空/观测约束不能同时满足 | 记录 NO_REACHABLE_VIEW 与证据，尝试有界恢复；不得静默替换到看不见目标的位置 |
| 目标 unknown、可以接近 | 有已知安全观测位姿，未知区域尚未确认 | UNKNOWN_APPROACH；执行安全位姿获取信息，未知路线段只作未验证估计 |
| 目标 unknown、长期遮挡 | 多个有效视角仍不能确认，预算/候选尚可用 | 调度替代视角、入口或公平重访；仍无法确认时 INCOMPLETE_OBSERVATION，不伪称已证明不可达 |
| unknown 变 free | 新有效观测确认目标区 | 重新评估足迹、路径、朝向与收益，切入 POINT_GOAL 或对应观测任务 |
| unknown 变 occupied | 新有效观测确认占用 | 重新搜索允许区域内的观测位姿；必要时 REGION_FALLBACK |
| occupied 变 free | 有效清除证据，几何版本改变 | 重新打开相关任务，复核原目标；按目标保持政策切换，避免无意义抖动 |
| free 变 occupied | 新障碍或相互矛盾的有效观测 | 失效受影响图边和剩余执行段，重新规划/选择；Safety 可否决 |
| 2D free 但悬空净空不足 | 3D 点云/深度确认机器人高度内障碍 | 目标或轨迹否决；搜索侧方有效观测位姿，不能仅依靠 2D costmap 放行 |
| 透明/反光/局部深度缺失 | 有效性不足，无法建立需要的自由空间证据 | 保留未知、拒绝未验证区段；不假设固定增加 20% 裕度就足够安全 |
| 全部观测失效或定位失效 | 过期、无变换、定位不可信 | 停止新平移，在仍可验证条件下停车；无条件旋转也被禁止 |
| 路径暂时受阻、目标仍有效 | 执行受阻但目标观测条件未失效 | 保留原意图，重规划、等待、有界恢复；不直接退化目标 |
| 稀疏图无路 | 可见图查询失败 | 已知空间栅格复核；图连接遗漏不能作为不可达证明 |
| 栅格也无路但边界仍未知 | 当前已知空间不连通，存在潜在未知入口 | 记录待确认、换视角或重访；有预算时探索，不能直接宣称永久不可达 |
| 到达但没有新增有效观测 | 朝向、停留、遮挡或质量要求未兑现 | ObserveAtPose 保持 RUNNING 或返回明确失败；重新选择视角，不增加真实覆盖 |
| 联合传感器覆盖重叠 | 两种观测落到同一稳定几何单元 | 几何去重；分别要求质量时单独记账和验收 |
| 单传感器退出联合模式 | 剩余观测、定位、控制模型、安全同时有效 | 允许配置内降级；原任务质量目标保留，不能自动宣布双模态目标完成 |
| 窗口移动、回环、地图重置 | 窗口/坐标版本改变 | 保留合法历史；可靠变换后复核或新 epoch 重建，拒绝旧坐标请求结果 |
| 小 frontier / 候选超过预算 | 收益小、区域多、当轮评估截断 | 公平扫尾、分页继续；截断项维持未评估，不作为完成证据 |
| 动态门口、暂时封闭房间 | 门/障碍变化或入口不可确认 | 保留重访条件；未知房间不能靠门口停靠算作房间已覆盖 |
| 时间/距离不足 | 剩余预算不满足探索加返航预留 | PARTIAL_BUDGET；执行配置的返航/保存，覆盖结果保持部分完成 |
| 返航或地图保存失败 | 导航失败/保存 ACK 失败 | 分别报告 RETURN_FAILED / SAVE_FAILED；成功保存部分地图不改变覆盖结果 |
| 取消、暂停或旧请求返回 | 会话/请求/目标版本不匹配 | 取消对应运动、保留合法账本、丢弃旧结果；恢复前重验健康与版本 |
| 几何可达但类别/任务语义不允许 | 经过验证的任务语义来源给出限制 | 独立报语义不满足；纯几何/DeFM 特征不得伪造类别判定 |

## 20. 实现边界与审查结论

当前可检查的代码位于 Opt/autonomy：统一观测与适配、MapReader、Vista 图核心、覆盖/完成评估、分层策略、Opt 观测目标适配及 ExplorationSession 初版已有文件。它们需要分别核查正确性与验证记录。

VistaPlanner 继承真实 GlobalPlanner，NativeCostmapReader 与六组 native 接口检查已在本次 SpaceHero 原生构建/专项回归通过。PlannerServer 的别名、optional 配置构造和同进程共享也已有通过记录。共享地图契约在 perception，图核心拒绝旧 epoch 回滚。完整 ExplorationTask/Tree 接入、跨进程路径提供、本方案的 DeFM 三模态条件与 SanD 执行适配、模型训练和端侧验收仍待完成；远端已有 SanD 代码不代表新增契约已经满足。

验收记录分层保留：旧全量 CTest 13/13、v1.8 本地 4/4、v2.0 原生 Vista 2/2、v2.1 本地四项 4/4 与原生六项 6/6 均只对应各自历史版本。最新正式 owner 部署后相关原生八项 CTest 8/8，2.13 秒，记录见 §20.14。基础覆盖、阶段和 BT 场景仍包含注入服务回执，不能作为真实任务验收。§20.3–20.13 保留历史过程，不能将其中当时的“尚未同步”状态当作当前状态；当前以 §20.14 及部署后指纹为准。

最终采用的默认政策与平台配置项：

1. 范围限定单层、与起点连通的允许空间。有任务边界时以该边界为准；未给边界时持续处理地图外缘的潜在可达未知区域，结论注明当前可达/可观测范围，不宣称整栋建筑 100%。
2. 默认目标是一份几何覆盖，联合模式去重。需要各传感器质量、RGB 纹理或类别目标时，在任务提交时显式开启；运行中不降低原要求。
3. 位姿适配局限于原观测区域及其观测约束。允许偏移/朝向、必须覆盖单元和最低收益是任务参数，未提供有效约束时不执行任意区域退化。
4. 初期以局部规划 8–10 Hz、跟踪 30–50 Hz 作为调度目标；感知年龄、定位质量、扫尾确认轮数、返航裕度和可执行控制频率通过平台数据标定，不采用论文数字代替本机验收。
5. 先接入已验证的几何执行闭环验证覆盖策略，再验收 DeFM/SanD 三模态模型；两阶段采用相同任务完成与安全契约。最终学习执行后端的三种模式必须分别通过任务验收。

### 20.1 审查必须明确的接入条件

- 定位链：三个模式均需能独立提供可用位姿与坐标变换。只有传感器消息适配完成不足以证明三模式任务可工作。
- 建图输入与收尾：复用既有 mapper 接受的输入，不增加 map 模块接口。需要核验 MappingTask / MappingClient 的结果能否证明所需观测已融合并保存；普通发布和提交成功不足以证明这一点。
- 原生协议缺口（2026-11-17 的只读协议记录）：automsgs/proto/task/mapping.proto 的 MappingCommand 只有 LOAD、SWITCH、SET_INITIAL_POSE、CLEAR_COSTMAP、KEEP_OUT 与 CANCEL，没有 SAVE 或融合屏障命令。此前任务入口曾将 EXPLORATION_CMD_SAVE_MAP 转发 MAP_CMD_SWITCH；当前正式同步的 owner 明确拒绝未接入 checkpoint 的手动保存请求，并未实现真实保存。切换或拒绝请求均不能作为保存成功证明。实施需核验并适配 map 外的任务协议和 MappingTask/MappingClient，调用真实上游保存及处理完成能力并返回可追溯结果；若上游没有该能力，严格收尾保留为未满足，不能只增加枚举便宣布完成。
- 在线地图来源：静态地图拷贝接口不自动等同在线建图输出。需确认规划地图持续更新，并与最终保存地图属于同一建图会话和可追溯坐标版本。
- 完成同步：若既有入口无法给出可信融合截面，MapReader 的 mapping_synchronized 保持 false，严格完成验收不能通过。可在 map 外的建图任务适配层使用真实处理完成依据；没有该依据时必须报告限制，不能用计时器或空队列伪造。
- 控制条件：三模式共用动作空间，但投影、历史、时间、模态掩码和反馈状态必须与训练契约一致。备用控制后端必须显式配置且单独验证。
- 任务范围：需要明确允许区域、表面质量、多视角与返航/保存政策；配置传感器看不到但任务仍要求的内容是未完成，不能自动剔除。
- 平台能力：足迹、高度、可通行台阶、线/角速度、加速度、制动能力与延迟来自平台配置。缺少必要证据时不能假设某个底盘参数。

### 20.2 当前实现必须继续闭合的缺口

1. 当前 ExplorationSession 的六组会话场景、六组独立阶段场景及六组原生 BT 场景已在本地回归。规划/导航超时、持续流入的双来源数据及地图融合截面、所有阶段暂停恢复和任务级故障组合仍需专项验证。
2. 真实路线返航预算、阶段保留和完成撤销已有初版逻辑；绕行预算、保存阶段暂停/恢复及保存前出现新任务的完整闭环仍待验证。
3. 覆盖单元稳定 Upsert、按来源质量正改善收益与多视角记录已有实现。v2.1 已修复 CoverageFusion::MarginalGain：相对真实/已选质量取正改善，统一按 required_quality 归一化并饱和；ANY_SENSOR 去重，PER_SENSOR 分别计量后平均。新回归证明劣质重复观测为零、缺失来源不能由另一来源替代、预测不写入真实账本及非法输入不产生部分更新。表面提取、无证据删除保护与队列幽灵项清理仍需专项验收。
4. 分页保持和取消传递已有实现；持续 revision 更新下的低收益后页公平性及访问优化取消仍待证明。
5. 分传感器新证据与有界队列进度已有回归；有限截面协议、上游融合确认与地图保存 ACK 仍需实现及真实服务接入。2026-11-18 部署后核验 map 471 文件，新增、删除、内容变化均为 0；最新指纹见 validation/2026-11-18-v2.9-review-native-postdeploy-check.json，不以旧 HEAD 代表当前工作树。
6. Vista 原生继承、注册、别名、配置构造、真实 Costmap 与同进程共享实例在 SpaceHero 编译和专项通过；八个探索节点进入真实 autonomy_task 编译。ExplorationTask owner/Session 已正式同步、构建及专项验证。TaskServer 的旧 waypoint/finished 驱动收敛仍需合并用户修改，真实 NavigationTask 领域/worker 生命周期已在 v2.12 验证，实际 ActionSession 派发许可、父子调度与依赖关闭仍有阻断项；跨进程路线、DeFM/SanD 三模式训练与端侧执行验收仍未完成。
7. 当前 has_height_evidence 标记不能代替原始观测的三维安全链。Vista 锁外地图/取消回调、提交前版本复核和读取串行化已有本次 native_reentrant_callbacks 与并发专项通过记录。覆盖结论仍需真实上游融合和保存证据；有限用例通过不证明所有并发/超时组合已覆盖。

上述是实施缺口，不改变本稿的目标架构，也不能在审查版中改标为已通过。

### 20.3 七项修复的历史状态与验证边界

本节区分当前源码、本地专项回归与原生 Docker 记录。四个本地相关目标通过日志为 validation/2026-11-17-v1.8-local-bt-tests.log；故障回归为 validation/2026-11-17-exploration-fault-regression.log。正式同步、原生模块编译与两个 Vista 专项测试见第 20.5 节；它们不替代真实任务和模型运行。

1. **完成确认新证据：已修复，本地回归通过。** CompletionEvaluator 同时检查轮次与各必需来源的有效观测截面；缺失来源、旧帧及只更新轮次不能推进稳定确认。单 watermark 兼容路径也要求推进。真实上游融合同步仍待接入，不能由本地测试代替。
2. **保存最终复核：已修复，本地回归通过。** SAVING 期间先更新健康与定位状态，消费 ACK 时检查最新输入及版本。定位过期可保留有效部分地图的 SAVED，但不保留完整探索成功；更晚的新工作使旧保存请求失效。用例显式给定已观测房间边界，未将地图外未知空间偷偷算作完成。
3. **Vista 锁外回调：已修复，本次原生专项通过。** 锁内复制拥有生命周期的配置和地图源，锁外执行地图/取消回调及搜索；提交时复核配置、请求与图版本。包含 native_reentrant_callbacks 的六组 native 检查本次在 SpaceHero 通过，连同真实 PlannerServer 测试形成 2/2 CTest 记录。未由有限用例推断全部并发情况正确。
4. **策略重启：已修复，本地回归通过。** 工作线程按新 session 调用 Reset，重置策略队列、游标与历史。同 epoch 重启重新选择目标，旧会话导航返回不能满足新请求。
5. **异常边界：已修复，本地回归通过。** 工作线程与拥有者服务捕获非标准异常；取消回调不越过析构边界。取消异常置执行故障，阻止额外隐式运动。真实原生服务和其他故障组合仍需验证。
6. **有界处理：已修复，本地回归通过。** 默认每 tick 最多 32 个事件、8 份观测，未处理项留队；命令与取消优先。accepted/processed/discarded 截面分开，有积压不能确认完成。处理名额也包含被丢弃的过期输入；仍需端侧测量最坏 tick 时间与停车响应。
7. **独立 BT 阶段与八个节点：本地编译和专项回归通过。** Session 的 RequestPhase/PhaseState/CancelPhase、自动/BT 互斥驱动、版本化 PhaseTicket 和 Select/Adapt/Observe/Return/Save 五类阶段已有六组阶段检查。task/exploration/plugins 使用真实 BtCondition、BtSyncAction、BtStatefulAction 与 BT_REGISTER_NODES；test/native_exploration_bt_test.cc 的六组检查核验首次 tick、独立阶段及实际观测、取消所有权、过期位姿/缺失上下文、旧 halt 与保存 ACK，以及两次观测/显式返航/保存的整树流程。本轮修正 Blackboard 的 const 接口和 nodiscard 编译问题，并纠正整树测试的相机视野及独立采集时序；最终四项 CTest 4/4 通过。服务使用注入回执，完整 ExplorationTask/SpaceHero 接入仍待验证。

质量正改善收益已在 v2.1 对齐实现；接下来优先闭合 ExplorationTask 的真实会话/BT 生命周期和跨进程规划调用，再实现有限截面及真实融合/保存。三种传感器的定位/观测、公平扫尾、三模态模型和三维执行链继续保留为独立验收项。原生模块构建阶段已通过，完整实施目标仍未完成。

### 20.4 v1.9 历史审查交付的范围

本次更新设计正文及审查视图，重新只读核对真实 MappingCommand 与旧探索保存转发。修订质量改善收益代理，增加有限观测截面协议并显式列出原生保存/融合缺口；未修改业务代码、autonomy/map 或远端业务工作树，未新增 build/CTest/实机结果。现有 v1.8 的 4/4 是其对应本地专项日志；不得将 v1.9 新增设计要求标为已经实现或通过。

### 20.5 v2.0 当前原生集成与验证证据

此前实施阶段已将 115 个文件同步到真实远端工作树，逐文件核验基线 SHA256 并备份已有内容，没有整目录覆盖或删除用户文件。同步清单为 validation/2026-11-17-native-sync-manifest.json；合并后的原生 CMake 位于 integration/native，未把 Opt 独立根入口当作原生根入口覆盖。

SpaceHero 原生配置与构建成功：autonomy_planning、autonomy_task、autonomy_control、autonomy_native_vista_test、autonomy_vista_server_test。使用 AUTONOMY_BUILD_EXPLORATION_TESTS=ON，仅开启新增原生回归；其他可选模块的全套构建未据此验收。

两项 CTest 通过：autonomy.native.vista 为 0.15 秒，autonomy.planning.vista_server 为 1.37 秒，共 1.52 秒。前者包含快照、真实规划接口、门关闭/重开及 epoch、工厂、并发读取、回调重入六组检查；后者检查真实 options 的 optional 零值与非法值、别名/id、同进程共享、真实 Path 与终点朝向。

对应记录：validation/2026-11-17-native-configure.log、validation/2026-11-17-native-build.log、validation/2026-11-17-native-tests.log。构建后 map 的 SHA256 核验仍为 471 文件、零新增/删除/内容变化，记录为 validation/2026-11-17-native-map-check.json。

当前原生 ExplorationTask 仍使用旧 waypoint 和 finished 通道，TaskServer 也保留探索 waypoint 提交路径；SAVE_MAP 仍转发 MAP_CMD_SWITCH。节点进入 task target 只证明可以编译，尚未成为真实探索任务的驱动入口。GetPlanner 的服务器测试证明同进程共享能力，规划与任务的独立 main 入口要求跨进程调用适配，不能据链接依赖推断实例共享。

本次原生模块通过不包括：三种模式的真实定位/观测、真实探索闭环、真实融合屏障和保存、三模态 DeFM/SanD 训练、时间样条及三维执行安全、回放/仿真/Orin 实测。审查材料更新不改变这些待完成状态。


### 20.6 v2.1 收益对齐与原生专项回归

本轮先复现旧算法的错误：实际质量 0.6、要求 0.8、预测质量 0.4 仍获得正收益；修复后按真实与已选质量的最大基线计算正改善。补齐覆盖状态/融合的文件、类、函数和成员 Doxygen，校验来源、类别、质量与固定归一化尺度；不使用非法枚举索引数组，不在错误输入批次中部分更新预选缓存。

旧重复去重用例由 0.18 修正为 0.2，因为 raw=0.9 已超过 required=0.8，其 normalized completion=1；这是与实际覆盖相同的计量口径，并非放宽阈值。新增质量/来源用例同时验证较差观测为零、质量饱和、预选重叠为零、PER_SENSOR 独立需求，以及预测不改变实际 revision。

原生测试增加覆盖、Session、Phase 和 BT 四个程序，链接真实 autonomy_task/planning/navigation_support，复用单一原生 BT 注册表，不编译第二份 registry。测试源码以 _test.cpp 后缀从 task 库递归收集中排除。六项 Docker CTest 6/6 通过，共 1.93 秒；本地四项 4/4，共 2.81 秒。导航、保存及融合仍通过测试服务注入，不声称完整任务已由真实 ExplorationTask 驱动。

逐文件同步 120 项清单中本次 10 项更新，基于上次部署 SHA256 校验冲突，已有内容备份至 /tmp/opt-native-1e993d25bad3/backup。完成后再次核验 map 471 文件，新增、删除和内容变化均为 0。未使用 reset/clean 或整目录覆盖。

记录：validation/2026-11-17-v2.1-native-validation.json、2026-11-17-v2.1-native-sync-manifest.json、2026-11-17-v2.1-native-build.log、2026-11-17-v2.1-native-tests.log、2026-11-17-v2.1-local-tests.log、2026-11-17-v2.1-native-map-check.json。完整实施目标仍为 active；真实任务/跨进程路径、融合保存、有限截面、三模态模型、三维安全与实机验收尚待完成。

### 20.7 v2.2 最终方案审查与当前核验

本版交付完整方案、离线审查页和交互审查画布。正文保留架构、三模式、信息增益、地图边界、增量图、Opt、训练、平滑、安全、BT、完成协议、目录及 26 类异常情况。新增第 12.4 节真实任务接管契约；修正实施顺序中已完成质量收益及六项原生专项的旧描述。

2026-11-17 只读远端核验：HEAD 为 7b251c5e5b5a2deeace5b7558a750d6917bac23f；六个任务/规划入口文件与此前读取的 SHA256 快照一致。最近部署清单的 120 个文件均与其部署 SHA256 一致；map 的 471 个文件与保护基线比较，新增、删除、内容变化均为 0。核验仅涵盖这些文件，不能证明其余工作树未变化。

该状态下八个原生探索节点已实现并进入对应的 task 构建及专项回归，真实 ExplorationTask 尚未接管 Session/Tree。旧 waypoint 竞争提交、finished Bool 成功路径、MAP_CMD_SWITCH 代替保存及真实上游同步，是正式任务接入必须继续修复的事项。v2.1 的六项 Docker CTest 6/6、本地四项 4/4 保留为对应实现的历史运行证据；本次未重新进行业务构建、CTest 或机器人运行。

来源核验：四篇 arXiv 页面及 FAR/TARE 官方 README 已重新读取，六个来源 HTTP 200。页面可访问与元数据核验不代表模型可直接复用，也不代表三模式或实机性能已经验收。

本版新增证据：validation/2026-11-17-v2.2-review-native-check.json、2026-11-17-v2.2-review-map-and-deployment-check.json、2026-11-17-v2.2-review-sources.json；审查材料结构和 TypeScript 检查另记 validation/2026-11-17-v2.2-review-artifacts.json。审查视图的静态检查不等于浏览器渲染或机器人运行验证。

完整实施目标继续保持 active。下一实施阶段按第 12.4 节接通真实任务生命周期与单一提交入口，随后闭合跨进程规划、真实观测/融合/保存与有限截面，再完成三模式模型、三维执行及任务/端侧验收。

### 20.8 v2.3 最终审查与新本地任务代码的边界

本次以实际 Opt 文件及保存的 v2.1/v2.2 证据核对最终方案。当前本地已写入 ExplorationTask 的 owner 工作线程、BT-driven Session 工厂、有界输入和任务命令队列、版本化导航服务；TaskServer 删除第二条探索 waypoint/finished 驱动路径，Shutdown 先停 exploration 再停 navigation。TypedTaskAppBase 新增身份关联结果读取，NavigationTask 新增按任务 ID 取消，Session 改为进程内统一的 session 计数。这些新代码尚未编译、测试或同步，不能称为真实任务已接通。

本次源码审查定位以下阻断项：Current 仅检查 generation/stopping，尚不能阻止 cancel 后 factory 返回引发晚启动；waiting_control 未在所有切换路径清空；thread 的 get_id/joinable 与 join 同步不统一；StopSession 的异常可能跳过 owner 清理；延迟队列满时可能静默丢弃已接受的要求；若干 idle-only setter 未执行限制；导航 Submit 仍为同步调用，可能阻塞 owner 的取消和健康处理。第 12.5 节明确修复契约；这里只报告源码审查发现，没有新增故障测试通过记录。

最终方案还要求跨进程启动身份、真实生产者 bindings、有限逐来源截面、真实融合/保存、完整表面/多视角与公平扫尾、三模式模型训练、时间样条/3D Safety/跟踪及回放/仿真/Orin 任务验收。轴对齐矩形之外的任务区域、显式部分覆盖政策和手动 checkpoint 当前未实现。完成凭据及反馈只使用真实统计，不能把终态 progress=1 当作过程中实测覆盖率。

v2.1 本地 4/4 与 Docker 6/6 保留为对应旧代码的运行证据。v2.2 的 120 文件一致和 map 471 文件零变化属于其已保存的远端核验，本轮未重新扫描远端、构建业务模块或运行机器人。当前本地代码指纹和审查材料结构检查另记 validation/2026-11-17-v2.3-review-artifacts.json；视图 TypeScript 及 HTML 结构检查不等于浏览器渲染或业务验证。

执行优先级：先修复并专项验证真实任务接管的取消/派发/关闭与身份关联，再以逐文件 SHA256 冲突检查同步并重编原生目标；随后接通真实观测、跨进程路线及融合/保存/有限截面；完成覆盖扫尾与三模式轨迹执行验收。完整实施目标保持 active，最终方案交付不等于实施目标完成。

### 20.9 v2.4 最终审查与实时源码核验

本版补齐第 12.6 节父探索与委托导航的调度、原生接受/取消责任及依赖关闭顺序，目标目录加入 navigation_dispatcher.hpp/.cpp。20 个章节、26 类情况、八个原生节点和三组核心公式仍保留；交互视图提供 30 项审阅标记。审阅标记仅表示用户核对过该条，不能当作实现或验收完成。

2026-11-17 再次只读 SSH 核验，远端 HEAD 为 bdfd9f0b8a2949f5b7fae08c5f5ec8e5c5243498，SpaceHero 正在运行。受保护 autonomy/map 共 471 个文件，与保护基线相比新增、删除、内容变化均为 0。检查的七个既有任务文件中，task_server.hpp/.cpp 已不同于 v2.2 逐文件快照；其余五个保持该快照内容。两个 navigation_dispatcher 文件仅在本地存在，远端尚无。不能将远端新 SHA 直接改成 expected 后覆盖文件，后续同步必须先读取并合并用户的新修改，再检查、备份及逐文件同步。

当前本地 MotionGate/NavigationDispatcher 提供可撤销 lease 与异步工作队列，但未被 NavigationServices 使用，后者仍通过 NavigationBridge 同步调用 bridge->submit。导航入口尚未提供贯穿 readiness 等待与执行启动的 guarded acceptance。这是接口原型状态，不是取消/暂停契约已经实现。

源码审查发现派发交接竞态：Submit 返回后先在锁外读 revoked；CancelRequest 在此后、accepted 公布前取消时，看到 accepted=false 而不入 native cancel 队列；提交线程随后仍可能公布 accepted=true，导致已被原生接受的请求漏取消。第 12.6 节规定提交线程与取消入口在同一受保护交接点确定取消责任。本次仅定位问题，未修复业务代码，未新增竞态测试通过记录。

此外，Unbind 仍在探索提交者关闭前调用 navigation_->Shutdown；20.8 的关闭描述不能替代对这条实际路径的修复。父任务/委托导航的 TaskScheduler 激活语义、回调重入关闭、异常后的 owner 清理、队列饱和与跨重启身份均需接入专项。旧 v2.1 六项原生和四项本地通过记录不覆盖这些新代码。

本次仅更新审查 Markdown、离线 HTML、交互画布和核验记录，无业务 build/CTest、远端写入或机器人运行。资料结构和画布 TypeScript 的验证独立记录，不等同于浏览器渲染或业务验证。当前证据：validation/2026-11-17-v2.4-review-native-check.json、2026-11-17-v2.4-review-sources.json 与 2026-11-17-v2.4-review-artifacts.json。完整实现目标仍未完成。

### 20.10 v2.5 早期总览与派发器本地专项的历史边界

本轮新增最终审查总览，完整设计继续保留八项默认政策、26 类情况、八个原生节点和 30 项审查清单。总览明确三模式定位/控制条件、质量改善收益、Opt 观测意图、有限来源截面、图/覆盖/执行几何的分离及待验收项，不新增虚构性能数据。

在此前修改的派发器上，最新源码重新 configure、构建 opt_navigation_dispatcher_tests 并运行对应 CTest，1/1 通过，测试时间 1.50 秒。八类场景覆盖 lease 暂停恢复和永久退役、阻塞提交的晚接受取消、5000 次接受/取消交接、身份结果与拒绝、Submit/Result 异常、取消失败禁止新运动、Shutdown 重入及提交中的暂停恢复。当前代码的取消责任在同一队列 mutex 内交接，native Cancel 锁外执行；失败状态供 owner 读取。这是有限场景的模块回归，不能证明所有线程交错、完整探索任务或原生 NavigationTask 的门控通过。

旧实现的交接压力回归已保存失败日志，表明接受窗口可能漏取消；较早的 CV 唤醒用例未复现，后续采用紧贴接受回调返回的 atomic 事件复现。旧实现失败记录与最新源码通过记录分开，不改写既有证据。当前源码 SHA256、命令和日志见 validation/2026-11-17-v2.5-dispatcher-local.json、2026-11-17-v2.5-dispatcher-local.log；旧实现记录为 2026-11-17-v2.5-dispatcher-before-regression.log。

2026-11-17 本轮只读 SSH 核验 HEAD 为 bdfd9f0b8a2949f5b7fae08c5f5ec8e5c5243498，SpaceHero running，map 471 文件与保护基线相比零新增、删除或内容变化。远端 TaskServer 两文件仍不同于旧快照，派发器两个文件仍未部署。本轮没有远端业务写入或 Docker 构建；后续同步须合并用户修改并检查上次部署指纹，禁止目录覆盖。

派发器尚未接入真实 ExplorationTask，原生 guarded acceptance、pre-tick/实际执行门控、父探索/委托导航调度、owner 异常清理和关闭顺序仍需接入与专项。真实观测/定位、跨进程路径、外层 backlog 的有限截面、融合/保存、公平扫尾、三模态训练、三维执行和 Orin 任务验收仍未完成。审查视图静态检查单独记录，完整实现目标保持未完成。

### 20.11 v2.6 最终审查：实际派发器部署与新任务接入

本节替代前述历史段落作为当前证据入口；不改写当时尚未部署的事实。本次只整理本地审查材料、核对主来源和已保存的验证指纹，没有新业务构建、CTest、远端同步或机器人运行。

**已经保存的派发器部署与原生专项。** v2.5 的五项白名单为 navigation_dispatcher.hpp/.cpp、navigation_dispatcher_test.cpp、ExplorationTests.cmake 和 navigation_dispatcher.md。同步脚本核验 expected SHA256，逐文件备份和替换，记录为 validation/2026-11-17-v2.5-dispatcher-sync-manifest.json。SpaceHero 的 planning/task/control 与相关测试程序 configure/build 均返回 0；七项相关 CTest 7/7 通过，总耗时 2.00 秒。记录为 validation/2026-11-17-v2.5-dispatcher-native-validation.json，以及同前缀 configure/build/tests 日志。该记录核验 map 471 文件零新增/删除/内容变化。

本次重新核对五项本地文件和三份日志 SHA256，均与原生验证记录相符。本地派发器的八类场景专项 1/1、5000 次接受/取消交接及 1.50 秒记录保持独立，不能将它与七项原生测试相加为完整探索验收。原生编译日志仍有测试 lambda 隐式捕获 this 的 C++20 弃用警告；不影响该次通过，但修复后要另存新源码与验证记录。

**新的 ExplorationTask 接入仅写入本地。** 当前 NavigationServices 使用每代 NavigationDispatcher；增加 GuardedSubmitNavigation、可撤销 MotionGate、进程启动身份及 request/session/generation 关联。Pause 保留 factory 返回的未启动候选，Resume 使用新许可；Cancel/Shutdown 撤销旧许可；回调重入关闭不自行 join。加入 owner 线程身份、idle-only 配置、输入转交失败和异常清理的检查。这些源码尚未编译、测试或同步，列为实施中。

新接入必须首先通过以下真实原生故障专项：factory/submit 阻塞时暂停、取消和新 START 抢占；factory 返回与 Session::Start 的提交窗口；owner 每次树 tick 前的许可复核；全部阶段暂停恢复；原生回调重入关闭；输入队列满、已接受输入转交失败与初始化过期/超时清理。原生 NavigationTask 仍需持续核验许可的接受和执行入口，TaskServer 仍需父探索/委托导航的运动槽、取消确认及依赖关闭顺序，不能仅靠派发前一次检查或 tick 后 hook。

**严格完成与保存仍为接入条件。** 外层输入 backlog 的有限完成截面、真实上游融合屏障和保存回执尚未闭合；任意允许多边形、部分覆盖政策及手动 checkpoint 也未完成。三模式生产者与独立定位、跨进程路线、公平扫尾全任务、DeFM/SanD 三模态训练、物理时间轨迹/3D Safety/反馈跟踪及仿真/实机/Orin 均保留待验收状态。

本次八个主来源请求均返回 HTTP 200，包括四个 arXiv 元数据、FAR/TARE 官方 README、DeFM 与 SanD v1 全文 HTML。本文仅把其机制与输入定义用于设计，不沿用论文硬件时延、候选数量或零样本表现作为本系统承诺。来源内容和指纹见 validation/2026-11-17-v2.6-review-sources.json。网页工具未提供可用 source ID，未制造网页引用。

审查页保持 20 节、26 类情况与八个核心 BT 节点；完整性、效率、平滑三项均按 §16 的任务级方法验收。当前 review 的源文件、离线页和画布静态检查另记 validation/2026-11-17-v2.6-review-artifacts.json；静态检查不等于浏览器渲染或业务测试通过。

### 20.12 v2.7 历史审查：只读核验与 owner 草稿

2026-11-17 再次只读核验真实远端：HEAD bdfd9f0b8a2949f5b7fae08c5f5ec8e5c5243498，SpaceHero 为 running。autonomy/map 的 471 个文件与原始保护基线完全一致，零新增/删除/内容变化。五项派发器文件的本地、远端指纹均匹配 v2.5 原生证据，三份保存的 configure/build/CTest 日志指纹也一致。这里只核验这些文件与旧日志，不代表重新构建或远端全工作树通过。记录见 validation/2026-11-17-v2.7-review-native-check.json 和同前缀 map-and-deployment-check.json。

本地新 exploration_owner_test.cpp 已有九类受控回调测试草稿，尚未编译、执行或同步。源码核对发现其把策略 Reset 当作 Session::Start、并要求策略复位在 task owner 上执行的假设不正确：当前 Reset 位于 Session 规划工作线程。该计数不能作为已启动/晚启动或线程一致性的证明，必须修正后再运行。此次没有修改该草稿或业务实现。

当前 ExplorationTask 的 Begin 建图准备分支仅在 Running 时执行；暂停发生于准备之前时，Resume 进入的 StartReady 没有补齐准备。建图准备失败和种子超时分支仅发布失败、清空 waiting，未经过完整 StopSession 清理，可能保留 runtime_busy。上述两项是基于当前源码的发现，尚未通过受控故障专项复现；第 12.7 节定义了所需修复及验收契约。factory 返回后的启动许可窗口、每次树 tick 前的最新意图复核、原生接受/执行门控及父子任务调度继续作为阻断项。

八个原始 HTTP 来源本次全部为 200，包括 DeFM 与 SanD v1 全文；内容及指纹见 validation/2026-11-17-v2.7-review-sources.json。明确 arXiv:2110.14573 的开发环境/集成角色，三模式投影、训练、覆盖收益、完成与切换约束均为本系统适配设计。未把论文参数或性能作为本系统已经达标的依据。

本次交付总览、详细设计、离线审查页和交互清单，未新增业务代码、运行业务构建/CTest 或同步远端。静态与 TypeScript 检查另记 validation/2026-11-17-v2.7-review-artifacts.json，不等于浏览器渲染。真实观测/定位、跨进程路线、覆盖扫尾/有限截面、融合保存、三模态训练、时间轨迹/三维 Safety/跟踪及机器人/Orin 仍未验收；完整实施目标保持 active。

### 20.13 v2.8 历史审查：owner 实际修复与隔离原生验证

2026-11-18，新的 ExplorationTask、exploration_session.cpp 与 exploration_owner_test.cpp 已在 SpaceHero 的独立 staging 中编译并链接现有真实原生共享库，十三类 owner 场景全部通过。编译、链接、运行返回码均为 0，源码和日志 SHA256 保存于 [专项记录](validation/2026-11-18-v2.8-owner-final-owner.json)。记录明确 native_checkout_modified=false；它不是正式原生目录同步、全目标构建或正式 CTest。

专项使用实际 Session 的非零 session_id 判断已经 Start，核验真实构造/销毁线程，策略 Reset 不再被当作启动证据。受控时钟和条件变量有有限等待期限；回调只控制外部服务/时序，核心使用真实 Session、树、策略和原生库。

旧实现实测失败：建图准备前暂停后恢复跳过准备，准备拒绝和种子超时未释放运行占用，最终 freshness/clock 检查期间取消或暂停仍发生晚启动。分别保存 baseline 的 6 通过/3 失败、增加启动窗口后的 6 通过/5 失败记录。修复初始化准备按代执行、统一异常清理、Start 的最终许可交接及每次树推进前最新意图复核后，十一类通过。

随后扩展暂停初始化计时和暂停期间覆盖监测，十二类通过/一类失败；失败是暂停消费新覆盖要求后，发布的 pending_units 未从真实账本刷新。补齐 Publish 计数刷新后，最终十三类全部通过。测试没有降低对实际覆盖反馈的要求。

当前十三类：factory_cancel、factory_pause_resume、mapping_pause_prepare、mapping_refusal_cleanup、seed_timeout_cleanup、factory_preemption、cooperative_bindings、factory_clock_cancel、factory_clock_pause_resume、paused_initialization_budget、paused_monitoring、native_submit_cancel、native_shutdown_reentry。生命周期、接口与失败清理详见 [exploration_task_owner.md](exploration_task_owner.md)。

既有 task_interface.hpp:89 的宏调用产生 extra semicolon 的 -Wpedantic 警告，未将本次编译描述为零警告。新增 owner 测试登记改变本地 ExplorationTests.cmake，这是有意修改，不应把与 v2.5 指纹不同解释为意外远端漂移。

本次另做只读远端核验：HEAD bdfd9f0b8a2949f5b7fae08c5f5ec8e5c5243498，SpaceHero running；map 471 文件与原保护基线相比零新增、删除、内容变化。读取 owner、Session、TaskServer 和 CMake 的当前远端内容并保存 SHA256，未写远端。记录见 validation/2026-11-18-v2.8-review-native-check.json。八项主来源也重新读取，均 HTTP 200，包含 DeFM/SanD v1 全文；记录为 validation/2026-11-18-v2.8-review-sources.json。

后续先逐文件读取冲突、备份并同步 owner/Session 修复，完成正式原生构建和包含 owner 的相关 CTest；继续实现 NavigationTask readiness/接受/pre-tick/实际执行的可撤销许可、父探索与委托导航调度，以及依赖关闭顺序。TaskServer 用户修改必须合并，不能覆盖。

完整目标仍未完成：真实三模式观测与独立定位、跨进程 Vista 调用、外层 backlog 有限截面、真实融合/保存、公平扫尾全任务、任意多边形/部分覆盖/checkpoint、三模态训练、物理时间轨迹/3D Safety/反馈跟踪、仿真/实机/Orin 验收均有后续工作。审查稿、离线页和画布仅展示这些明确的契约及当前证据，不将方案交付标为实施完成。

### 20.14 v2.9 历史审查：正式 owner 部署、八项原生 CTest 与剩余链路

2026-11-18，六项白名单 owner/Session/CMake/说明文件在检查既有 expected SHA256、保存原内容后同步到正式远端。备份位置为 `/tmp/opt-owner-deploy-v2.8-86ef9ffc183b/backup`。见 [同步清单](validation/2026-11-18-v2.8-owner-sync-manifest.json)。同步没有覆盖目录、重置工作树或修改 TaskServer。

SpaceHero 正式 checkout 配置成功，`autonomy_planning`、`autonomy_task`、`autonomy_control` 与八个相关测试目标构建成功。下列正式 CTest **8/8 通过，2.13 秒**：Vista 原生接口、Vista Server、multisensor exploration、Session、Phase、native exploration BT、navigation dispatcher、exploration owner。具体命令、退出码与日志摘要见 [原生验证](validation/2026-11-18-v2.8-owner-native-validation.json) 和 [CTest 日志](validation/2026-11-18-v2.8-owner-native-tests.log)。指定目标构建不等于全部目标构建；服务注入及受控时序不等于真实服务或任意线程交错已经通过。

第一次远端启动器因 fish 对 JSON 中反斜杠的处理失败，CTest 当时没有启动。重试采用 base64 命令参数后成功运行八项测试。原启动器错误日志被重试覆盖，无法提供匹配原摘要的原始日志；验证 JSON 将其移至单独的历史失败记录，保留原摘要、原路径与缺失原因。此前 manifest 原样保存在 `2026-11-18-v2.8-owner-native-validation-before-log-repair.json`。成功 configure/build/CTest 三份日志均重新核验 SHA256；证据修复没有重复业务构建或测试。

部署后只读核验见 [当前源文件核验](validation/2026-11-18-v2.9-review-native-postdeploy-check.json)：六项部署文件与同步指纹一致，map 的 471 个文件与保护基线一致；TaskServer 两文件与部署前源快照一致；SpaceHero running。仅凭 HEAD 不判断工作树版本，新增文档更新另有本地 artifact 指纹。

本轮只更新审查文档、离线页、画布及验证元数据。八项官方来源重新读取并保存 HTTP 状态和摘要，见 [来源记录](validation/2026-11-18-v2.9-review-sources.json)。联网工具没有可用来源编号，因此使用官方原文副本核对，未编造网页引用。

后续实施顺序与通过条件：

1. **真实运动资格与任务调度**：NavigationTask readiness、身份接受、BT pre-tick 及实际 action dispatch 连续核验可撤销许可；取消不等待长 readiness 锁；委托导航继承父探索运动槽；保留 TaskServer 用户改动并修正依赖关闭顺序。原生实际执行入口的门控尚未实现，不能用 owner Start 门控代替。
2. **三模式真实生产者与路线服务**：原始束/深度元数据、标定、独立定位、按实际时刻变换、在线地图来源、task 到 PlannerServer 的 Autolink 路线请求及版本兼容。必须逐模式回放验证。
3. **完成与地图收尾**：完整公平扫尾、外层队列有限前缀、真实融合屏障/保存回执、任意允许多边形、显式部分覆盖、手动 checkpoint。无真实回执时不得成功。
4. **任务回放与仿真**：在已验证的控制后端上先验证完整探索、动态门、遮挡、传感器退出、取消/恢复、返航与保存失败，比较同一覆盖标准下的时间/路程。
5. **学习与连续执行**：冻结 DeFM，适配三模态融合/扩散头；物理时间边界、运动学约束、完整扫掠体、停车空间和反馈跟踪分别验收。不能从三次样条内部 C2 推导跨周期平滑或安全。
6. **实机与 Orin**：三模式独立任务实验、传感器盲区、最坏延迟、截止时间违约、停止能力、实际 jerk/clearance。没有结果前不写达标数字。

完整实施目标继续保持未完成状态。最终方案可以审查，但训练和完整闭环不能因代码存在或专项通过而视为完成。

### 20.15 v2.10 历史审查：真实 BT 执行前检查与九项原生回归

2026-11-18，新 BtRunner/BtTaskApp、真实 XML 专项、原生 CMake 与说明共六项白名单文件在核验 baseline SHA256 后备份并同步。备份为 `/tmp/opt-bt-deploy-v2.10-432a286541be/backup`；见 [同步清单](validation/2026-11-18-v2.10-bt-sync-manifest.json)。没有覆盖目录或改动 TaskServer。

隔离 native 编译、链接现有真实原生库与十一类 BT 场景先通过，随后正式 checkout 配置、planning/task/control 及九个测试目标均构建成功。相关九项 CTest **9/9 通过，2.28 秒**，包含 autonomy.task.bt_runner。见 [正式验证](validation/2026-11-18-v2.10-bt-native-validation.json) 及 [CTest 日志](validation/2026-11-18-v2.10-bt-native-tests.log)。所有构建/测试日志与源码指纹分别保留；既有警告仍存在。这不是全工作树、真实传感器或整机任务通过。

实现 BeforeTick/BeforeTreeTick 执行前桥接；拒绝本轮真实节点效果；取消、暂停/恢复及时唤醒长周期等待；worker 回调重入不 self-join；标准/非标准回调与 halt 故障保持 FAILED。测试另补充等待边界唤醒、取消同时发生异常保留失败、typed host 的真实脚本效果为零。十一类隔离场景不与九项 CTest 累加。

审查稿补齐质量改善收益的硬筛选、ANY_SENSOR 保守去重及必要方向的独立兑现，明确离散候选完备性是“相对于任务分辨率完全覆盖”的前提；公平遍历不充分的候选集合不能证明连续空间完整覆盖。

实际 ActionSession dispatch 与 NavigationTask readiness/接受的 lease、父子调度和取消确认仍未接通。暂停树不等于底盘停止。三模式真实生产者/独立定位、跨进程 Vista、完整公平扫尾与有限截面、真实融合保存、任意多边形/部分覆盖/checkpoint、三模态训练、物理时间样条/3D Safety/跟踪、回放/仿真/实机/Orin 继续待完成。完整实施目标仍为 active。

### 20.16 v2.11 历史审查：接受元数据保护与十项原生回归

2026-11-18，typed-task 基类、真实原生专项、CMake 与说明共四项白名单文件经过 expected SHA256 核验、备份和正式同步。备份为 `/tmp/opt-typed-deploy-v2.11-2e43f82e178c/backup`，见 [同步清单](validation/2026-11-18-v2.11-typed-sync-manifest.json)。TaskServer 保持用户内容，map 471 文件未新增、删除或修改。

基类只在 OnGoal 接受后提交 task/client 身份；pending handler 期间 GetResult 与按 ID 读取返回 false 且不修改输出；较晚返回的旧接受不能覆盖新的已接受身份；拒绝不清除有效任务；异常释放 pending，Shutdown 使旧接受票据失效。身份更新分类默认兼容带 header 的请求，领域可覆盖 ShouldUpdateGoalIdentity。详见 [接受契约](typed_task_admission.md)。

九类隔离 native 场景通过，随后正式 checkout 配置、planning/task/control 与十个相关测试程序编译成功，相关 CTest **10/10 通过，2.22 秒**。见 [正式验证](validation/2026-11-18-v2.11-typed-native-validation.json) 和 [测试日志](validation/2026-11-18-v2.11-typed-native-tests.log)。本次收回前轮仍在运行的正式验证结果，不为审查材料重复业务构建。

只读部署核验确认四项部署源指纹及成功日志匹配，map 471 文件保持保护基线，TaskServer 两文件未变，SpaceHero running。见 [核验记录](validation/2026-11-18-v2.11-review-native-postdeploy-check.json)。指定目标与受控场景通过不代表全部系统任务通过；已有警告仍在。

真实 NavigationTask 尚未接入身份分类：同位姿新 task ID、匿名 START 和 PAUSE/STOP 仍需领域实现及专项。其 readiness 不能持有阻塞取消的长锁；接受、pre-tick、实际 ActionSession dispatch 与撤销使用同一 lease。基类元数据保护无法撤销领域回调已发送的动作，不能代替父子运动调度、晚接受取消及真正执行许可。

集中最终设计见 [审查稿](multisensor_exploration_final_plan.md)。三模式真实生产者/独立定位、共享跨进程路线、完整公平扫尾与有限截面、真实融合保存、多边形/部分覆盖/checkpoint、三模态训练、时间轨迹/三维 Safety/跟踪和回放/仿真/实机/Orin 仍待完成；完整实施目标保持 active。


### 20.17 v2.12 NavigationTask 领域完成与 v2.13 最终审查

原先 v2.11/早期 v2.12 审查中未完成的 NavigationTask 草稿已经完成本批次领域接入。START/REPLAN 提交新身份，匿名 START 清除旧身份，控制请求保留已接受身份；同位姿新 task ID 启动新树，完整命名 START 重试保留调用并撤销更早 pending 替换。几何和 XML 文件检查、服务准备失败不会打断当前调用；XML 内容及节点构造仍由 runner 在运行时验证。

readiness 默认二十秒，CV 等待最多五十毫秒，等待不持目标交接或生命周期锁；取消、暂停、Shutdown 与较新请求可撤销等待者。替换撤销旧 worker 资格，锁外停止并 join，安装 active/tree/motion epoch，再运行新树。终态不被晚接受覆盖；CancelForTask 只取消匹配工作，旧 active 取消保留无关新 pending。worker 控制重入不 self-join，START/REPLAN 回调重入被拒绝；真实 halt 故障和可见取消依赖异常保留 Failed，并禁止新 START/Resume，直至显式重新初始化。

**验证范围。** 真正 NavigationTask、typed 基类、Protobuf、注册器、XML、worker 和 StatefulActionNode 的 21 类隔离场景全部通过；仅控制 RPC readiness、截止时间和取消依赖。[隔离记录](validation/2026-11-18-v2.12-navigation-stage.json) 与 [场景日志](validation/2026-11-18-v2.12-navigation-native-navigation-scenarios.log) 保留具体范围。六项白名单 expected SHA256 核验、备份、同步见 [同步记录](validation/2026-11-18-v2.12-navigation-sync-manifest.json)。

SpaceHero 正式 checkout 配置、planning/task/control 与十一个相关测试程序重编成功；CTest **11/11 通过，2.67 秒**。新 `autonomy.task.navigation` 用时 0.41 秒；构建包含实际 navigation.cpp 与 TaskServer 消费者重编。[正式记录](validation/2026-11-18-v2.12-navigation-native-validation.json)、[构建日志](validation/2026-11-18-v2.12-navigation-native-build.log) 和 [CTest 日志](validation/2026-11-18-v2.12-navigation-native-tests.log) 已保存。21 类隔离场景不与 11 项 CTest 累加。[部署后核验](validation/2026-11-18-v2.12-navigation-postdeploy-check.json) 确认 map 471 文件未变、TaskServer 用户内容保持、SpaceHero running。

**仍须接入的运动边界。** motion epoch 是领域/worker tick 资格，并非外部 dispatch lease。ActionSession 实际 send、晚接受取消、父探索委托资格、取消确认和依赖关闭仍需闭合；一次发送前谓词检查不足以关闭撤销/发送竞争。生产 NavigationClient 的 CancelActiveMotion 是异步取消且部分异常被内部处理，返回不能当作物理停止 ACK。

**完整方案保持未完成。** 三模式真实观测与独立定位、跨进程 Vista、完整公平扫尾和有限截面、真实融合保存、任意多边形/部分覆盖/checkpoint、三模态训练、物理时间轨迹/三维 Safety/跟踪以及回放/仿真/实机/Orin 仍有独立验收工作。v2.13 更新集中方案、审查入口、离线页和画布，纠正旧草稿状态，不重复业务 build/CTest；历史证据不重写。方案政策与待实现接口以集中审查稿为准。


### 20.18 v2.15 本地派发实施与最终审查边界

本地 ActionSession、TaskActionClient、MotionDispatchGate、NavigationTask 和异步节点已接入逐目标取消责任；Autolink Client 增加发送 UUID、明确拒绝/不确定传输失败区分及按 UUID 取消。发送资格提交前登记责任，撤销关闭新提交，已提交/迟到接受继续负责取消；取消请求 ACK 不能当作权威终态。

既有隔离原生构建使用真实 Docker 编译参数、头文件与链接库，NavigationTask 和原有测试编译/链接成功，原有 21 类导航场景通过。2026-11-18 本轮复核十一项源码指纹及四步日志与保存记录一致，见 [隔离复核](validation/2026-11-18-v2.15-review-stage-check.json)。这不是新增派发竞态通过声明；真实节点消费者未正式重编/部署，阻塞 send、迟到接受、取消 ACK/终态、失败/异常、错误 UUID、结果先于接受及会话销毁仍需专项。

原生 transport 分离工作线程仍捕获客户端原始 this，依赖关闭尚需修复和验证；规划撤销的有界清理、父子任务资格与物理停车也未验收。当前目标设计、任务配置、接口与阶段门槛以 [集中审查稿](multisensor_exploration_final_plan.md) 为准。

本轮远端只读核验十七个业务文件保持派发实施前基线、门控头未部署、map 471 文件保持保护基线、TaskServer 用户内容保留、SpaceHero running；见 [远端核验](validation/2026-11-18-v2.15-review-native-check.json)。正式 v2.12 CTest 11/11 与本地 v2.15 隔离验证属于不同源码版本，不混用。原始来源八项 HTTP 200 及 SHA256 见 [来源记录](validation/2026-11-18-v2.15-review-sources.json)。本轮没有新业务构建或测试，完整实施目标继续未完成。
