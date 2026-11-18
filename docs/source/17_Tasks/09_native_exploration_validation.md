# 探索原生构建与验证

本页记录源码同步与实际原生消费者验证的方法。远端源码位于
`/home/quandy/workspace/github/autonomy/src/autonomy`；SpaceHero 内源码为
`/workspace/autonomy/src/autonomy`，构建目录为
`/workspace/autonomy/build/autonomy`。

## 同步边界

`scripts/sync_native_exploration.py` 接受经过检查的明确文件清单，核对本地
内容摘要与远端预期摘要后逐个备份、原子替换并复验。没有隐式目录覆盖。
`autonomy/map` 的 471 个文件和远端 TaskServer 由独立摘要基线保护。
连接中断后需重新检查远端，不能仅凭客户端超时断言没有写入。

本次保留原生深度 critic 的 `local_esdf`，新增 `execution_esdf`。
Control CMake 从实时远端版本合并，保留原有 SanD 测试和
`BUILD_TESTING` 处理。新增场景程序自带 `main`，直接注册到 CTest，
不套用 GTest XML 验证器。

## 实际容器构建

可以进入用户指定环境执行：

```bash
docker exec -it SpaceHero /bin/bash
cmake -S /workspace/autonomy/src/autonomy \
      -B /workspace/autonomy/build/autonomy \
      -DAUTONOMY_BUILD_EXPLORATION_TESTS=ON
cmake --build /workspace/autonomy/build/autonomy \
      --target autonomy_planning autonomy_task autonomy_control -j 4
```

`scripts/validate_native_exploration.py` 进一步构建相关测试目标，
在真实构建树中运行 CTest，并保存每一步日志及源码摘要。它使用实际
Autonomy 库、Autolink、automsgs 生成类型和原生 BT，没有编译暂存替身。
仅启用探索检查选项，不把可选域全部开启。

测试覆盖包括：原生观测 Reader 生命周期、消息布局、观测入口退役与丢失；
Vista 地图/服务器接入；BT runner 和 typed task；NavigationTask 及动作
派发竞争；路径完整身份与结果等待；分层策略、公平遍历、有限观测前缀及
丢失恢复；物理时间样条、差速底盘、实测地面与全身几何准入。

## 证据与状态

2026 年 11 月 18 日，v2.35 的真实原生 configure 和三个库 build 均返回 0，
最终 CTest **26/26 通过，耗时 3.92 秒**。同步前后均核验全部 471 个 map
文件未变化、TaskServer 保留。首轮 build 已通过但 CTest 为 25/26：
`SensorPacketRejectionLoss` 把明确拒绝误当成序号缺口。修正测试后检查
`historical_discards` 保存序号 17，且 `historical_gaps` 不制造缺口。
生产拒绝/丢失处理没有为通过测试而放宽。

v2.36 新增反馈跟踪器、不可变准入证书和跟踪/停止回执场景。
三个库及 27 个测试目标的真实原生 build 返回 0，最终 CTest
**27/27 通过，耗时 3.96 秒**。验证前后核对 201 项源码/配置/证据摘要，
所有 471 个 map 文件和 TaskServer 仍保留。

新增场景验证实际点云/地面准入、proposal 可变别名隔离、指令时限、
健康/身份/重放拒绝、反馈越界、错误或过期安全回执、超时/异常、校验期间
停止和回调重入、停止权威匹配及计划时刻的物理 C2 切换。
指令几何与停止权威为受控依赖；没有把模拟回执解释为实际硬件证明。

- `docs/validation/2026-11-18-v2.35-native-sync-manifest.json`：82 项源码/config 同步。
- `docs/validation/2026-11-18-v2.35-control-test-sync-manifest.json`：测试注册修正。
- `docs/validation/2026-11-18-v2.35-native-validation.json`：实际原生 configure/build/CTest。
- `docs/validation/2026-11-18-v2.35-native-final-sync-manifest.json`：20 项注释、文档及测试修正同步。
- `docs/validation/2026-11-18-v2.35-native-final-validation.json`：最终 26/26 的实际原生结果。
- 同名 `-configure.log`、`-build.log`、`-tests.log`：每一步原始输出。
- `docs/validation/2026-11-18-v2.36-native-sync-manifest.json`：跟踪器、文档和既有证据共 16 项同步。
- `docs/validation/2026-11-18-v2.36-native-validation.json`：最终 27/27 的实际原生结果。
- `docs/validation/2026-11-18-v2.36-native-validation-tests.log`：27 项原始 CTest 输出。
- `docs/validation/2026-11-18-v2.36-local-validation.json`：同一跟踪器源码的独立 Opt build/CTest 记录。

这些检查仍不包括独立 SLAM、真实地图融合/落盘、DeFM 推理权重、三模式
SanD 模型训练、独立指令扫掠与实际停止权威，
以及 Orin 和机器人任务验收。
每个未完成接入应保持单独记录，不能由 CTest 全通过推断整体系统可部署。

v2.37 的控制改动使用 `common/math` 的样条实现、现有 NMPC/MPPI 和
`automsgs::Path/PoseStamped/TwistStamped`。新增原生适配检查目标为
`control.policy_native_spline_tracker_test`。它包含实际两个控制器在既有
costmap 上的有界求解，以及参考/反馈/生命周期场景；模型生成和硬件
闭环仍不在该检查范围内。

2026 年 11 月 18 日，v2.37 最终实际原生 configure、三个库及测试目标
build 均返回 0，CTest **28/28 通过，耗时 4.11 秒**。验证前后核对
222 项源码/配置/既有证据摘要，471 个 map 文件未变，TaskServer 保留。
同时保留用户在 `autonomy/task/CMakeLists.txt` 中新增的充电测试注册。

首轮 CTest 为 27/28：MPPI 的自由空间夹具采用圆形机器人，默认
CostCritic 却请求了不存在的完整 footprint。修正夹具，使 CostCritic
保持启用并按圆形机器人校验，使用既有 API 显式填充自由空间；同时
调整断言顺序，保留求解后生成的诊断。未修改 map 或原生求解器来通过测试。
最终回归还覆盖原生返回成功但未写入 Twist 的拒绝，以及失败后的参考
失效。安全层否决后也清除上一规划先验和复用标志。

Opt 的共享样条回归记录为 **22/22 通过，耗时 10.32 秒**，包括
非均匀结点、导数控制点界、物理时间求导及 C2 接续；该记录不包含
原生 NMPC/MPPI 求解，二者由上述真实容器检查覆盖。

- `docs/validation/2026-11-18-v2.37-native-spline-final-validation.json`：保留首轮失败及日志摘要。
- `docs/validation/2026-11-18-v2.37-native-spline-verified-manifest.json`：最终原生修复清单与保护基线。
- `docs/validation/2026-11-18-v2.37-native-spline-verified-sync.json`：明确文件同步与备份记录。
- `docs/validation/2026-11-18-v2.37-native-spline-verified-validation.json`：最终实际原生 configure/build/CTest。
- 同名 `-configure.log`、`-build.log`、`-tests.log`：最终三阶段原始输出。
- `docs/validation/2026-11-18-v2.37-local-common-math-validation.json`：Opt 共享样条回归与源码摘要。

`Path` 只保留几何参考，不能以这组结果推断最终 Twist 的 jerk、
障碍场景的闭环安全、训练模型性能或实际硬件停稳已经验收。


## v2.38 增量图版本连续性与数值边界

2026 年 11 月 18 日，实际 SpaceHero configure、三个原生库及 29 个测试目标
build 均返回 0，CTest **29/29 通过，耗时 4.72 秒**。验证前后核对
233 项源码/配置/既有证据摘要，全部 471 个 map 文件未变化；TaskServer
和用户新增的充电测试 CMake 注册继续保留。

修复前保存三项有界失败：跳过地图 revision、最新差分区域缺失，均使旧
可见性边跨越新障碍；非法大范围索引替换删除了此前有效边。修复后，版本
不连续或范围变化会全域复核；连续差分仍局部更新；原位匹配的顶点 ID 保留。
空间索引先检查范围再替换，宽查询转为已存包围盒过滤。全域更新与局部更新
通过独立几何检查逐边复核，包括正反两个方向；移除障碍后被阻塞边仍可重连。
无向边双向准入同时规避栅格边界插值顺序造成的校验差异。

新增 `autonomy.planning.vista_incremental_graph` 包含七个生产算法场景：
版本缺口、差分缺失、索引替换拒绝、数值边界、契约预检、连续局部失效/恢复、
地图范围收缩。Opt **23/23 通过，耗时 13.51 秒**；上述七场景另通过
AddressSanitizer、UBSan 和 float-cast-overflow 检查。macOS 不支持
LeakSanitizer，首次启用的失败日志保留；随后关闭该项，未关闭其余检查。

- `docs/validation/2026-11-18-v2.38-vista-baseline.log`：修复前的三项失败。
- `docs/validation/2026-11-18-v2.38-vista-first-fix.log`：初次修复仍有方向差异的记录。
- `docs/validation/2026-11-18-v2.38-vista-native-manifest.json`：13 项明确同步清单与保护基线。
- `docs/validation/2026-11-18-v2.38-vista-native-sync.json`：同步、备份及摘要核验。
- `docs/validation/2026-11-18-v2.38-vista-native-validation.json`：实际容器三阶段结果。
- 同名 `-configure.log`、`-build.log`、`-tests.log`：原始 configure/build/CTest 输出。
- `docs/validation/2026-11-18-v2.38-vista-local-validation.json`：Opt 回归结果。
- `docs/validation/2026-11-18-v2.38-vista-sanitized-verified-validation.json`：七场景数值检查。

该结果继续覆盖 SanD → `common/math` → 完整 automsgs Path → 原生
NMPC/MPPI → TwistStamped 的接入，不证明真实模型、SLAM 融合保存、
独立指令扫掠、硬件 watchdog 或实际停稳已经验收。探索生产组装仍需要
真实建图会话和可验证的融合/落盘回执；MappingClient 的当前 LoadMap
仅检查名称，不能当作已经加载地图的权威证据。


## 共享样条求导缓存：v2.39

2026-11-18，SanD 采样和物理时间参考继续复用
`common/math/bspline.hpp`。共享曲线在构造时缓存一至三阶导数曲线，
求值期间不再重建导数控制点和结点数组。没有在控制模块增加另一份样条
求值或求导算法，完整 Path 继续交给已有 NMPC/MPPI 产生 TwistStamped。

新增 `autonomy.common.bspline_cache` 检查解析三次曲线、非均匀结点线性
重现、有限差分、C2 连续性、结点处右侧 jerk、导数凸包、复制/移动和独立
控制点副本、短曲线回退、非法结点以及并发 const 求值。
同一标量测试对两条曲线做 80,000 次位置/导数求值，旧实现检测到
**300,000 次 C++ 堆分配，缓存后为 0**。该检查排除构造、复制和采样
输出数组，也不覆盖动态点类型内部的分配或整个控制周期延迟。

Opt **24/24 通过，12.45 秒**；新增检查另通过 AddressSanitizer、UBSan
和 float-cast-overflow。SpaceHero 的真实 configure、build、CTest 均返回
0，**30/30 原生测试通过，4.73 秒**，包括共享数学库、完整 Path、原生
NMPC/MPPI、探索及增量图回归。原有 map 的 471 文件、TaskServer 和用户
添加的 Task CMake 充电测试保持原样。

证据文件使用 `docs/validation/2026-11-18-v2.39-shared-bspline-` 前缀：

- `allocation-validation.json`、`baseline.log`、`cached.log`：相同测试的旧版/缓存版对照。
- `local-validation.json` 及三个阶段日志：Opt configure/build/CTest。
- `sanitized-validation.json` 及编译/执行日志：独立数值检查。
- `native-manifest.json`、`native-sync.json`：代码同步及保护基线。
- `native-validation.json` 及三个阶段日志：SpaceHero 原生验证。
- `evidence-manifest.json`、`evidence-sync.json`、`final-verified.json`：文档/证据同步及最后只读摘要核验。

真实应用组装核查及尚未闭合的定位、融合和保存入口记录在
[任务说明](08_multisensor_exploration.md)。本轮代码和测试不构成完整探索
系统、实机执行 jerk 或硬件停止的验收。

相关设计分散在现有章节中：[任务](08_multisensor_exploration.md)、
[规划](../08_Planning/07_vista_planner.md)、
[感知](../10_Perception/13_multisensor_observation.md)、
[控制](../09_Control/controller/08_sand_execution_safety.md)。

## 协作导航许可：v2.40

原生 NavigationTask 的 `SubmitGoalWithMotionPermit` 复用现有 typed
admission，并将探索许可绑定到 native MotionDispatchGate。未改变
automsgs 消息、map 或 TaskServer。该接口补足真实任务和实际发送门之间
的许可传递；父任务 exclusive slot 的生产组装仍须单独完成。

在已有测试中增加以下行为检查，不用替代实现模拟准入或动作会话：

- 共享 gate：外部许可拒绝、异常锁存、恢复不复活、未发送预留与已发送
  动作的终态责任，以及 128 次真实线程许可撤销/发送提交竞态。
- NavigationTask：初始拒绝、身份/命令约束、等待服务和就绪返回时撤销、
  join 旧 worker 时撤销、运行中停止、同身份重试/REPLAN 不移除 guard、
  暂停后恢复不复活、恢复许可查询允许另一线程读取生命周期，以及最后
  一个成功 tick 不越过撤销。
- 实际 ActionSession 和 native task：已进入节点后的许可失效、一次 UUID
  定向取消、取消 ACK 不能替代 terminal drain、已撤销 gate 不再发送。

验证报告使用 `docs/validation/2026-11-18-v2.40-native-permit-` 前缀。
`before.json` 记录同步前的完整摘要和保护文件，`native-manifest.json` 和
`native-sync.json` 记录代码/文档同步；`local-validation.json` 与
`native-validation.json` 分别记录本地和 SpaceHero 的实际阶段结果。
报告中的返回码和执行日志决定是否通过，源码新增测试本身不是通过证据。
tick 前撤销和已进入动作节点后撤销分别可能产生 canceled/failed，测试
保留这两条真实路径，并要求没有成功、没有额外发送且有真实终态清理。

2026-11-18 验证结果：Opt **24/24 通过，6.58 秒**；共享 gate 的
AddressSanitizer/UBSan 检查通过。SpaceHero 首轮 configure/build/CTest
全部返回 0，**30/30 通过，4.94 秒**。随后补充恢复时生命周期锁检查，
并修正动作已进入执行时的终态断言；该次源码编译成功，但链接时构建
目录缺少多项 Autonomy 库产物，build 返回 2，未运行 CTest。
保留 `reviewed-validation.json` 及配置/构建日志作为失败证据。

使用现有 CMake 重新生成缺失库后，最终代码的 configure、build、CTest
全部返回 0，**30/30 通过，4.93 秒**。最后验证使用
`reviewed-manifest.json` 的完整 271 项源码摘要，结果和阶段日志在
`repaired-validation.json`、`repaired-validation-configure.log`、
`repaired-validation-build.log`、`repaired-validation-tests.log`。
库产物缺失的原因未确定；没有改源代码来规避链接错误。

新增许可行为分别有 3 个共享 gate、8 个真实 NavigationTask 和 2 个
真实动作发送场景，均在现有测试程序中运行。原有 map 的 471 个文件、
TaskServer 和用户的 Task CMake 修改保持原样。完整探索生产组装、
三模式实际定位、融合/持久化、训练权重和底盘停稳仍不在这些结果的
证明范围内，具体缺口见 [实际组装说明](08_multisensor_exploration.md)。

## 共享样条起始状态求解：v2.41

2026-11-18，将控制层剩余的起始控制点反算移入
`common/math/bspline.hpp` 的 `ClampedCubicBSpline::StartControlPoints`。
`TimedBSpline` 只换算物理时间导数并复制共享函数返回的前三个控制点，
其余控制点与结点保持原样。采样、求导、导数界和起始 C2 接续均使用
公共数学实现；完整 automsgs Path 继续供已有 NMPC/MPPI 产生 TwistStamped。

新增检查通过实际曲线求值验证均匀/非均匀结点下的位置及一、二阶导数，
并检查输入曲线不变、远端控制点保留、非法边界拒绝和数值溢出。
原生控制测试另外验证 Eigen 三维控制点，已有物理时间测试覆盖 XYZ/yaw。
共享边界求解不承诺 jerk 连续，也不替代路径安全复核。

Opt **24/24 通过，14.37 秒**，共享数学检查通过 ASan/UBSan 和
float-cast-overflow 检查。SpaceHero 的 configure/build/CTest 全部返回 0，
**30/30 通过，4.89 秒**。同步前核对 v2.40 的 294 项完整摘要，map 的
471 个文件、TaskServer 和用户 Task CMake 修改保持不变。

证据文件使用 `docs/validation/2026-11-18-v2.41-shared-bspline-boundary-`
前缀：`local-validation.json`、`sanitized-validation.json`、
`native-manifest.json`、`native-sync.json`、`native-validation.json` 及对应
阶段日志。`reviewed.json` 记录源码摘要；`evidence-manifest.json`、
`evidence-sync.json` 与 `final-verified.json` 记录文档/证据同步和最终核对。
该结果不改变上述探索生产组装与实机验收的剩余范围。

## 探索父子导航所有权：v2.42

2026-11-18，新增真实 `autonomy.task.scheduler_motion`，直接链接
Scheduler、NavigationTask、ExplorationTask、原生 BT 与 Vista。
最后 SpaceHero configure/build/CTest 均返回 0，**31/31 通过，7.09 秒**。
CTest 选中数量和 Scheduler 注册已在执行前实际核验；保存的
`LastTest.log` 摘录确认该程序的 **13 个场景全部输出 PASS**。

覆盖领域准入、Request/Submit 预约、两种注册顺序、父探索委派和普通
导航接管、暂停恢复的子身份、禁用抢占、关闭全局独占仍保护共享引擎、
取消失败/异常、旧回收快照、legacy 终态回收、待准入撤销、关闭重入，
以及四线程各二十轮 Start/Stop。MotionAdmission 只使用内部 RAII
句柄；目标、路径、结果、取消与速度消息继续复用 automsgs。

父探索 dispatcher 和 Scheduler 可能同时取消一个子导航。重复取消
返回时，另一线程可能仍在 join；版本化取消回调在锁外有界等待对应
worker 的实际软件清理。NavigationTask 的 `MotionDrained` 还检查
runner 的运行标志：终态已经发布，但取消回调或树销毁尚未结束时，
仍不能交出所有权。新增有界回调阻塞场景明确验证这一阶段不会提前
接管，清理完成后才允许新任务。原有 NavigationTask 的 join 撤销场景
同时断言已清理执行编号，避免替换被拒绝后保留空的运动屏障。

验证保留了以下失败与修正，不以最终结果覆盖旧日志：

- 首轮 build 返回 2，Fields2Cover 缺少 GDAL include；补齐 GDAL 后，
  第二轮 build 仍返回 2，暴露该可选域的独立 API 大小写错误。
- 第三轮编译 Scheduler 成功，但 CTest 正则漏选它，只执行原有 30 项。
  `scheduler-third-review.json` 明确该记录没有验证 Scheduler 运行。
- 修正选取后，Scheduler 段错误，其余 30 项通过。源码中存在在异步
  SetupBlackboard 之前读取空门对象的竞态；调试器未重现该段错误，
  没有取得原始回溯。调试执行暴露测试缺少明确的 RGB-D 控制配置。
- 补齐真实 worker 发布等待及测试配置后，段错误消失；接管检查暴露
  并发取消时过早判定清理失败。修复有界等待后 31/31 通过，7.03 秒。
- 最后增加终态回调清理检查并补齐 runner 状态判定，重新编译后的
  31/31 和 13 个 Scheduler 场景通过，结果为上面的 7.09 秒。

Fields2Cover 通过 `AUTONOMY_BUILD_FIELDS2COVER` 作为独立可选域管理。
验证后保存的首次 SpaceHero cache 记录为 **OFF**，见
`2026-11-18-v2.json`。最后只读复核时，当前 cache 已为 **ON**，
远端 Planning CMake 也有后续修改；本轮保留这些字节，不覆盖配置。
上述 31/31 结果对应 `terminal-validation.json` 的源码基线，没有重新
验收该可选域或后续 CMake 修改。启用时使用 GDAL 的原生目标提供
包含目录与链接依赖。原始 Planning CMake 字节保存在
`2026-11-18-v2.42-planning-cmake-original.json`。

Opt 的已有 portable/BT 回归为 **30/30，20.52 秒**，该结果没有执行
真实 Scheduler 程序；它由上述容器检查独立覆盖。本轮最终检查继续
包含公共样条、非均匀结点、物理时间导数、起始 C2 接续以及真实
NMPC/MPPI 接入。SanD 的样条数学仍统一到 `common/math/bspline.hpp`，
没有在局部模块加入独立求值、求导或起始边界算法。

证据均位于 `docs/validation/`，使用
`2026-11-18-v2.42-scheduler-` 前缀：`first/second/third/reviewed/fixture`
验证及阶段日志保留各次失败和漏选；`fixture-review.json` 说明诊断边界；
`cancel-validation.json` 记录首次真正全选通过；`terminal-manifest.json`、
`terminal-sync.json`、`terminal-validation.json` 及阶段日志记录最终代码；
`terminal-output.json/.log` 保存再次读取的 13 场景输出和当前可选域配置。
`final-presync.json` 保存同步前的完整摘要及后续 Planning CMake 差异。
`evidence-manifest.json` 保留最初准备的清单；
`reviewed-evidence-manifest.json` 和 `evidence-sync.json` 记录补齐输出及
配置说明后的实际同步。`final-verified.json` 记录本地、远端和容器摘要
复核，并区分已测试代码与保留的后续配置。

这些检查验证软件所有权和清理。测试的传感器输入、可用性及外部 RPC
回复仍为受控依赖；未提供真实三模式定位、融合/保存回执、训练模型、
底盘停止或实测 Twist jerk 的证据。完整探索生产组装仍按
[任务说明](08_multisensor_exploration.md) 中的缺口推进。
