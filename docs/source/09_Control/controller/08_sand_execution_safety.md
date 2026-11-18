# SanD 的物理时间轨迹与执行安全

新增实现位于 `autonomy/control/controller/policy_controller/sand_planner/`。
它检查具有物理时间和明确执行身份的 B 样条，分别处理参考连续性、底盘
运动学、机器人全身占据体积和实测地面支撑。

## 保留原生接口

原生 `local_esdf.hpp/.cpp` 仍为已有 SanD critic 提供深度图 ESDF 接口。
新增 `execution_esdf.hpp/.cpp` 构建带来源、时间和对齐证明的不可变执行
几何快照。二者同时编入实际 Control 库；没有通过覆盖旧接口迫使现有
critic 或原有测试改用新的类型。

旧 critic 的优化评分和新增执行准入有不同责任。保留旧 critic 不表示
它已经获得全身、安全停车或实际闭环跟踪证明。

## 参考与时间

`TimedBSpline` 显式保存坐标帧、对齐世代、参考 lease、版本、开始时刻、
持续时间及有效期。求导按物理时间缩放，而非把归一化样条参数当作秒。
姿态采用连续 yaw；轨迹身份不能仅由起点和终点推断。

`PrepareReferenceHandoff` 在计划切换时刻从旧参考求取位置、速度和加速度，
构造候选的 C2 接续条件。完成时间缩放或修改控制点后，需要再次运行
`CheckReferenceHandoff`。这验证参考拼接，不能证明实际轮速指令或机器人
运动已经连续。

## 几何与地面

真实观测包和定位者提供的 `ExecutionAlignment` 一起输入
`BuildLocalEsdf`。所有射线保留实际采集时间，输入必须属于相同执行帧及
对齐世代。未知体素、体积外部和过期证据阻止准入；自由射线不抹除已保留
的障碍命中。距离下界扣除体素、测量和定位对齐误差。

`BuildLocalGround` 从同一 ESDF 实例携带的真实表面命中拟合支撑片。
没有观测的栅格保持无支撑。拟合检查四象限样本、稳定性、平面残差、坡度、
相邻台阶和落差；名义地面高度不制造地面测量。

全身校验使用包围机器人所有允许姿态和突出部件的柱状包络。
连续弦段检查包含曲率偏离裕度、定位/跟踪误差、反应时间与保守制动距离。
地面和 ESDF 必须固定到同一不可变实例，只有相同版本号仍不足以复用。

## 差速约束与统一准入

`DifferentialDriveLimits` 显式要求轮距、标定版本及左右轮轮缘速度、加速度
和 jerk 限制，同时检查侧向、竖向和倒车限制。
`ValidateDifferentialDrive` 按真实三次样条结点分割物理时间区间，通过
导数界和区间细分验证整个区间，不只验证少量采样时刻。

生产组合入口是：

```cpp
ValidateLocalTrajectory(trajectory, limits, validation_options, safety_options,
                        esdf_snapshot, ground_snapshot, drive_limits, clock);
```

底盘运动学、连续几何、全身和地面校验共用一次工作量预算、区间预算和
截止时刻。中途过期、未知几何、缺少地面支撑或预算耗尽均不发布有效准入。

## 反馈跟踪与停止门控

`AdmitTrackingReference` 调用上述真实组合准入，成功后创建
`AdmittedReference`。它私有保存一份样条副本，并保留实测 ESDF、地面和
底盘约束。调用者后来修改原始 proposal 不会改变已经校验的样条。

`TrajectoryTracker` 由一个串行控制者持有。输入包括原始期限的
`TrackingLease`、同步位姿/轮速/轮加速度反馈、准入参考以及可选实际停止
回执。所有输入必须匹配执行帧、对齐世代、标定版本和时钟域。
反馈过期、心跳缺失、租约撤销/静默延期、重放、参考失效或跟踪误差越界
都会锁存停止请求；健康输入重试不会自行清除停止状态。

启动也需要停止确认。配置的底盘权威提供匹配 stop request 的回执，并
说明实际停稳时刻和看门狗状态；当前实测位姿速度和轮速还必须支持这个
回执。写出零指令或收到取消确认都不会产生 `kStopped`。已停止租约不能
通过相同 grant generation 恢复执行，新执行需新的底盘授权。

候选替换在计划切换时间重新验证物理 C2，且不允许静默修改机器人包络、
轮距/限制或制动参数。等待初始参考期间，若实际机器人已移动，立即请求
停止。反馈与实际轮速还须满足标定的平面差速关系。

跟踪器返回 `TrackingCommand`，包含参考/租约身份、反馈序号、几何版本、
未来生效时刻和绝对失效时刻。速度值是底盘在该时间窗内执行受限过渡的
终点目标，不是直接立即生效的普通 Twist。轮速及平均过渡斜率检查是必要
约束；连续执行 jerk 和安全制动仍要由实际底盘过渡曲线及独立校验实现。

`TrackingCommandVerifier` 必须验证加入反馈修正后的实际机器人全身、
实测地面及反应/制动扫掠。只检查原始参考不能代替这个回调。回执必须
匹配完整指令并覆盖时限；缺失、异常、未知几何、错配及超时均不输出
可执行指令。外部回调需要自己的有界执行保证，调用者只能检测超时，不能
强制抢占阻塞的回调。底盘也必须独立落实身份、过渡曲线与硬件看门狗，
以处理主进程退出后的失效指令。

## 原生 SanD 跟踪接入

生产执行链为 `PolicyController::ComputeVelocityCommands` →
`PolicyPipeline::ComputeCommand` → `PredictTwist` 选出完整样条 →
`NativeSplineTracker` → 现有 `NMPCController` 或 `MPPIController` →
`CommandSafetyFilter` → `ControllerServer::PublishVelocity(TwistStamped)`。
SanD 分支不再把 Q1 的航向角当作角速度，也不直接产生横向/竖向速度。
`action_sequence` 保留实际选中的候选，而不是候选列表第一项。

`native_spline_tracker.hpp/.cpp` 只适配参考与生命周期，不实现求解器。
`BuildSandPlannerPath` 调用 `common/math/bspline.hpp` 中的
`ClampedCubicBSpline<Eigen::Vector3d>::Sample/EvaluateDerivative`，将完整曲线
按推理开始时的真实位姿转换到已有执行坐标系，填充
`automsgs::msgs::nav_msgs::Path` 及 `PoseStamped`。朝向来自切线，首尾点都
保留。复用期间不会随机器人移动重新锚定参考。非有限值、未锚定 Q0、
过大的输入、静止曲线和超过 5 cm 的竖向控制偏移都会被拒绝。

`TimedBSpline` 只保留已有物理时间/身份校验及时间缩放职责。它的求值、
求导及导数控制点全部委托给同一个 `common/math` 实现，局部模块中没有
独立的 de Boor 或样条求导算法。起始 C2 状态的前三个控制点反算同样调用
共享的 `StartControlPoints`：控制层将速度和加速度换算为归一化参数的
一、二阶导数，共享数学库使用原始结点求解，后续控制点保持不变。
共享实现新增显式非均匀结点构造和导数
控制点访问，保留原来的均匀构造与短曲线回退行为。

共享 `ClampedCubicBSpline` 在构造时缓存一至三阶导数控制点和对应的
降阶结点数组。后续 `Evaluate`/`EvaluateDerivative` 读取这些不可变数据，
不再为每个求导采样点重新分配样条存储。标量和固定维度 Eigen 点的求值
不需要样条堆分配；动态点类型自身的代数操作仍可能分配内存。构造、复制、
返回 `DerivativeControlPoints` 的独立副本，以及 `Sample` 的输出数组仍
需要分配，因此此项优化不代表整个规划或控制周期没有分配。

代价是每条曲线额外保存三个导数曲线，空间复杂度仍为 O(N)。均匀和
非均匀结点共用相同缓存路径，C2 连续性、结点处右侧 jerk 求值和物理
时间缩放保持原有语义。缓存不引入可变的延迟初始化，多个线程可以对同一
稳定实例进行 const 求值；并发赋值或销毁仍须由调用者同步。

使用既有 `ControllerOptions.nmpc_controller_options` 和
`mppi_controller_options` 配置实际求解器。`policy_controller_options`
只新增两个标量配置字段：

```protobuf
policy_controller_options {
  sand_tracker: "NMPCController"  # 默认；可替换为 "MPPIController"
  sand_replan_period: 0.1         # 秒，默认 0.1，最大 1.0
}
```

MPPI 使用既有 `DiffDrive` 模型。Configure/Activate/Deactivate/Cleanup
传递给实际原生控制器；Reset、路径替换、模型替换、受阻和失败均清除
参考。每次生成调用原生 `SetPlan(Path)`，每个控制周期调用原生
`ComputeVelocityCommands(PoseStamped, TwistStamped, ...)`。原生 NMPC
`SetPlan` 会重置 warm start；本适配保留这一行为，相关效率需实测。
局部端点不是任务终点，任务完成仍由外层 PolicyController 的原始目标判断。

SanD 扩散按单调时钟的周期缓存；观测编码、新鲜度检查、反馈控制和安全
过滤仍逐控制周期执行。缓存原始期限不由 Compute 续期。消息采集/接收
时间使用 system clock 秒，执行耗时和扩散周期使用 steady clock，避免
将两个 epoch 混用于 age 检查。采集设备和进程需使用一致的消息时钟。

真实速度反馈必须存在 `twist` 字段，并具有有效采集时间及 body frame。
ControllerServer 缺少里程计时生成的仅含 Header 的默认消息被拒绝。
位姿必须使用已有 costmap 的坐标系。优化失败、未写入 Twist、非平面输出、参考过期、
反馈过期或超预算均丢弃原生输出并强制零速度，不沿用上一周期指令。
求解完成后重新检查反馈年龄。实测 Twist 的全部轴需有限；微小的实测
残差由原生运动模型处理，不伪造为零值。必需观测被零填充时不授权运动。
安全过滤后再次保证命令 Header 使用 body frame。原生求解失败和安全层
否决都清除参考复用标志与上一规划先验，下一周期重新生成并校验参考。
最后仍由现有 ControllerServer 发布 `TwistStamped`，没有新增消息协议。

## 当前执行边界

`Path` 是几何参考，不编码原 SanD 样条的物理时间、速度或加速度。
NMPC/MPPI 根据自己的既有模型、速度约束、优化代价和实时反馈跟踪它；
不能由样条 C2 推断最终 Twist/轮速的 jerk 已受约束。严格时间跟踪、
路径替换的 warm start、实测延迟、跟踪误差及最终速度平滑性仍需验收。

上述原生接入复用原控制器的 costmap 碰撞检查和 PolicyPipeline 安全过滤。
它尚未消费 `AdmittedReference`/`TrajectoryTracker` 的租约证书，也未接通
独立实际指令扫掠、底盘授权/停稳权威和硬件 watchdog。
普通 `TwistStamped` 不携带这些执行证据；写出零速度不是实际停稳回执。
这些边界不能用原生 NMPC/MPPI 测试通过替代，既有几何/停止证明类型
继续保留用于后续独立接入。

源码与模型约束测试也不验证轮地打滑、负载或材料承载，不能据此宣布
机器人执行安全或平滑性已经通过实机验收。

检查入口包括 `differential_drive_test.cpp`、`timed_trajectory_test.cpp`、
`local_ground_test.cpp`、`local_trajectory_safety_test.cpp` 和
`trajectory_tracker_test.cpp`。原生适配检查位于
`native_spline_tracker_test.cpp`，涵盖 automsgs 消息、固定参考、默认反馈
拒绝、生命周期、晚到命令拒绝、空输出拒绝，以及实际 NMPC/MPPI 求解。
原生求解夹具通过现有 costmap API 显式填充自由空间，MPPI 的 CostCritic
保持启用，圆形机器人使用 `consider_footprint=false`，与夹具的机器人
几何配置一致；该夹具只验证接入与求解，不验证障碍场景的避碰性能。
跟踪器测试使用真实点云解码、ESDF、
地面构建和准入；指令扫掠、底盘授权/停稳权威及定位回执为受控依赖，
不能用这些测试替代实机验收。
部署与原生检查见 [验证记录](../../17_Tasks/09_native_exploration_validation.md)。
