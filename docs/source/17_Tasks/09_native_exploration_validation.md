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
启用探索检查选项；DeFM 检查同时显式启用现有 ONNX 后端，不把可选域全部开启。

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

## v2.43：任务 owner 管理原生观测订阅

2026 年 11 月 18 日，输入生命周期改动的真实原生 configure、三个生产库和
32 个测试目标 build 返回 0，CTest **32/32 通过，7.47 秒**。新增
`autonomy.exploration.exploration_observation_source` 在 0.31 秒内通过四组
实际 Node/Reader/Writer 场景：三种模式的预 START/START/暂停/取消/新 START；
真实采集回调未退出时保留旧注册、只为最新世代重建；退役超时后保留错误
与 channel 所有权；模式不一致与原生 Reader 冲突拒绝。

生产 Source 创建与协调调用位于 `ExplorationTask::Impl::Run()`，不在测试
工厂里代替执行。原生消息、Source 与退役实现由现有
`autonomy_navigation_support` 统一编译，测试链接同一实现。
应用必须调用 `ConfigureObservationSource()` 安装真实定位查询及源配置。
现有 `task_main` 的真实 Session/定位/地图服务组装仍是后续接入，不能将
owner 生命周期自动协调解释为完整探索应用已经运行。

本轮首个 build 返回 2：新增测试没有显式包含 automsgs sensor 生成头。
补齐 `native_sensor_messages.hpp` 并让测试退出等待实际 Reader 清理后，
重新 configure/build/CTest 全部通过。首轮失败日志保留，没有覆盖。

验证使用当前 SpaceHero cache，`AUTONOMY_BUILD_FIELDS2COVER=ON`，
`AUTONOMY_BUILD_PERCEPTION=OFF`。保留并编译当前 Planning CMake
`c1bf4ff683a01c32f3117fffde3f904faf04cb8ad7816d19a41ee0ce4375991e`；
Task CMake 的充电测试修改保留。32 项是明确选取的原生测试集合，不代表
可选 Perception/模型或所有其他域都已经验收。全部 471 个 map 文件与
TaskServer 在同步和验证中再次核验未变。

证据位于 `docs/validation/`，前缀为 `2026-11-18-v2.43-input-`：
`before.json` 保存完整远端摘要及实际 cache；`manifest.json`、`sync.json`
记录首次九文件同步；`validation.json` 与阶段日志保留首轮编译失败；
`repair-manifest.json`、`repair-sync.json` 记录测试依赖修正；
`repaired-validation.json` 与三份阶段日志记录最终 32/32；
`terminal-output.json/.log` 保存重新读取的实际四场景及 Test Passed 输出。

本次不新增样条数学实现。SanD 继续通过 `common/math/bspline.hpp` 求值、
求导及求解起始控制点，`TimedBSpline` 仅处理物理时间、身份及约束换算。
参考曲线 C2 与真实底盘输出 Twist 的 jerk 是不同的验收项。

## v2.44：共享 Vista 图的原生路线服务

探索客户端使用已有 automsgs `GetPlan` 服务访问 PlannerServer 的实际
`VistaPlanner`，服务在同一地图/图快照中返回路径与证据。请求携带独立 token，
回复保留 frame、map epoch、geometry revision、LiDAR/RGB-D 各自的观测前缀、
恢复基线与世代。精确起点不进行普通导航的占用起点吸附；`NO_ROUTE` 与通信、
协议错误区分，任何全局路径都不携带执行安全许可。

新增原生测试包含实际 PlannerServer、`posix_spawn` 创建的独立客户端进程、
错误 token/状态/证据/几何、超时/取消及迟到回复退役。首次 build 因使用
Client 的私有 `IsInit()` 失败；改用公开幂等 `Init()` 后 build 成功，首轮
CTest 31/33 通过，8.51 秒，仍有旧 map epoch 返回码与跨进程测试失败。
这些失败日志保留在 `2026-11-18-v2.44-route-validation*` 和
`2026-11-18-v2.44-route-repaired-validation*`，没有覆盖或删除测试。

跨进程失败的 GDB 栈显示历史消息发送任务在发送器析构后继续读取其
`RoleAttributes`。修复后的 `HybridTransmitter::ThreadFunc` 是静态函数，
异步任务持有专用通道属性、预先选定的传输模式、缓存消息及共享 participant；
任务不再捕获原发送器的裸 `this`。旧地图 epoch 保留原生 `PLANNER_TF_ERROR`
语义，观测前缀冲突单独返回取消。

发送器修复后的原生重建成功，但 `lifetime-validation.json` 仍为 32/33，
8.95 秒；增加客户端回调世代与同请求重传后的 `retry-validation.json`
为 32/33，8.84 秒。两轮失败都在路线服务测试，均为 SIGSEGV。后续真实
core 栈定位到 `HybridReceiver::ThreadFunc` 的历史消息监听器：它引用
函数栈上的到达标志和接收器裸指针。详细证据保存为
`2026-11-18-v2.44-route-crash-retry-segv-core-gdb.log`；诊断日志、其他
可取得的 GDB 记录和缺失日志状态见 `route-crash-evidence.json`。大 core
保留在远端，没有作为源码或文档同步。

接收器修复使用基类 `Receiver::CallbackState` 保存原始身份和回调。
普通 SHM、进程内、RTPS 监听器复制拥有该状态的适配器，不捕获接收器
裸指针。历史线程是静态函数，持有专用通道、预先选定的模式、转发状态
和共享原子到达标志；接收器析构关闭入口，等待已进入的回调退出，再
撤销监听器。历史线程用 `USleep` 短轮询观察关闭状态，保留原有静默
一秒后结束的行为，不占用协程 worker 做阻塞等待。

新增测试通过真实 SHM 历史通道进入受控阻塞的回调，验证并发接收器
析构不能提前返回，释放回调后可以退出，已复制监听器和晚到消息不会
重新调用业务层。Client/Service 分别验证真实 response callback 和服务
worker 退役后重新初始化。首次接收器重建在分发器回调参数适配上失败，
记录于 `route-receiver-validation.json` 和 build 日志；随后补齐原生
两参数分发器与三参数接收监听器之间的适配，完整重建继续进行。

参数适配后的 configure/build 成功，CTest 为 32/33，12.66 秒：新增
历史端点夹具没有填入 `host_ip`，被实际 SHM 分发器过滤。补齐真实主机
属性后，单项测试通过历史接收退役、跨进程共享图、协议校验、超时取消、
同请求重传和 Client 退役，随后暴露 Service 过早退出拓扑的问题。
失败记录为 `route-history-endpoint-target.json` 及原始日志。

Service 现在先关闭回调入口、清理尚未执行的队列，唤醒并等待实际 worker
退出，再退出拓扑和释放 transport。这样已进入的 worker 发送回复时，
客户端的匹配监听器仍然存在。测试保留真实请求回复和重新初始化断言，
没有改成仅检查线程退出。

服务退役头文件触发依赖重建，首次单项脚本因等待预算不足返回 124；
构建仍在容器内继续，未同时启动第二份构建。确认原构建退出后重新
执行实际目标，build 返回 0，路线服务单项连续三次通过，耗时分别为
1.70、1.76、1.71 秒，总计 5.17 秒。记录为
`route-service-drain-recovered-target.json` 和对应 build/tests 日志。

2026 年 11 月 18 日，最终真实 SpaceHero configure、三个原生库及全部
33 个选定测试目标 build 均返回 0，CTest **33/33 通过，耗时 9.23 秒**。
结果为 `2026-11-18-v2.44-route-final-validation.json` 及其 configure/build/
tests 日志。验证前后完整源码清单匹配，全部 471 个 map 文件和 TaskServer
保持原样；用户 Task/Planning CMake 的当前修改保留，没有进入同步写集。

此次通过覆盖原生路线传输与回调退役、行为树和探索状态、增量图、公共
样条缓存与物理时间轨迹，以及既有 NMPC/MPPI 的原生跟踪器。原生路线
单项另有三次连续通过记录；失败和修复日志全部保留在同一 v2.44 前缀下。

应用的真实 Session factory、定位/地图融合回执、DeFM 权重推理及底盘
平滑性验收仍需独立接入。样条求值、导数和起始边界控制点始终调用
`common/math/bspline.hpp`，没有新增控制模块的样条数学实现。33 项通过
不能代替真实 SLAM 融合、训练模型性能或机器人 Twist jerk 的验收。

最终源码复核还将历史接收测试的退出清理注册移到第一次传输和等待之前，
异常退出时先释放回调屏障，再销毁接收器；退役线程引用的原子状态同样
保留到 join 完成。该夹具修正后的实际 configure/build/CTest 再次全部
通过：**33/33，9.21 秒**。当前最终源码对应
`2026-11-18-v2.44-route-cleanup-validation.json` 及三份原始日志，
生产修复与上述三次路线服务连续验证一致。

## 原生 Session 工厂：v2.45

新增 `native_session_factory.hpp/.cpp` 将真实 task Node、原生观测源、共享
路线客户端、覆盖策略与目标位姿适配器组装为未启动的 BT Session。静态配置
在安装前校验；源配置和工厂原子替换。实际 CameraInfo 与覆盖模型内参、
定位证明与安装外参不一致时拒绝输入。

新增 `autonomy.exploration.native_session_factory` 使用真实 Task owner、
Autolink Reader/Writer、generated automsgs 和 PlannerServer，检查非法配置
不部分安装、标定不一致、缺少融合/保存操作时的失败清理，以及三种传感器
模式的 owner 工厂与共享图。定位和建图回调为受控测试依赖，刻意不给融合
和落盘完成回执，不能据此声明真实 SLAM 或整机探索成功。

本节新增代码在实际 SpaceHero 中执行 configure/build/CTest，阶段结果与原始
日志按各次运行单独保留。既有 map、TaskServer 和用户 Task/Planning CMake
继续保持在保护集合内。公共样条求值、导数与起点约束仍复用
`common/math/bspline.hpp`，完整 automsgs Path 交给既有 NMPC/MPPI。

首轮 SpaceHero configure/build 成功，CTest 为 **33/34，14.71 秒**。
配置原子性、标定拒绝和缺少融合/保存接口后的 owner 清理均通过；三模式
夹具错误地要求 START 后立即产生融合屏障。实际 `BeginRefresh` 属于收尾/
保存路径，正常启动不会无条件请求该屏障。夹具改为先等待真实 post-START
观测使选定模式就绪，再推进受控单调时钟触发明确的任务预算退出，核验
真实融合请求、PartialBudget 结果及融合回执缺失时尚未发起保存。
原始失败记录保留在 `2026-11-18-v2.45-native-factory-validation.json` 和
对应 tests 日志中。生产行为没有为该夹具改动。

第二轮 configure/build 成功，CTest **33/34，15.41 秒**；就绪断言发现
测试源已接收，Session 的 accepted/processed 输入前缀也已推进，但当前
定位支持仍为空。原生单项诊断保存在 `native-factory-diagnostic-target`
记录中：夹具重复发送 3.25 秒采集帧，早于 3.4 秒的 Session 启动种子。
`RecordPacketPose` 正确地只将较早位姿写入历史，不用它确认当前定位。
夹具改为在每次发送时使用受控实际采集时间，保留原生消息时间戳和
启动后实际观测要求。未放宽健康阈值、定位证明或完成判据。

采集时间修正后的原生单项通过，耗时 **2.54 秒**，记录为
`2026-11-18-v2.45-native-factory-acquisition-target.json`。随后完整 SpaceHero
configure、Planning/Task/Control 原生库及所有选定测试目标 build 返回 0；
CTest **34/34 通过，耗时 10.83 秒**。结果为
`2026-11-18-v2.45-native-factory-final-validation.json` 及对应 configure/build/
tests 原始日志。CTest 注册和选择已逐项核验，包含新增工厂测试。

验证覆盖三种模式的实际订阅与 owner 组装、配置原子性、相机/安装标定
拒绝、缺少实际融合/保存接口后的清理、预算退出时的融合屏障，及既有
增量图、BT、共享样条、NMPC/MPPI 回归。验证前后全部 471 个 map 文件
和 TaskServer 保持原样，用户 Task/Planning CMake 不进入同步写集。
真实定位、地图融合/保存供应者、DeFM 权重推理、SanD 模型三模式训练
和实机平滑性验收仍需分别完成，34 项通过不代表整机探索任务完成。

## v2.47：真实 DeFM 与原生 RAW 特征接口

新增 `autonomy.perception.defm_encoder` 使用官方 ViT-S/14 权重导出的真实
模型和独立 Torch fixture，覆盖共享编码、原生深度字节布局、有限采集期限、
静态 token 轴、RAW 输入和过期零填充拒绝。模型及 fixture 缺失会失败。

完整 `480x640 -> 224x224` 相机实例的本地 ONNX/Torch parity 已通过，
normalized_depth、patch_tokens、global_feature 最大绝对误差分别为
`3.0934811e-5`、`6.9141388e-5`、`1.1444092e-5`。官方单图和 batch
预处理的最大绝对误差单独记录为 `2.3841127e-5`，不替换任何官方预处理。
实际权重哈希与图形状保存在 `2026-11-18-v2.47-defm-model-manifest.json`。

首轮 SpaceHero configure/build 成功，CTest 为 **34/35，10.86 秒**。
新增 DeFM 测试在模型推理前失败：原生构建缓存中 `BUILD_ONNXRUNTIME=OFF`，
`common::network::Engine` 报告没有编入推理后端并拒绝 `backend_id="onnx"`。
这次记录保存在 `2026-11-18-v2.47-defm-native-validation.json` 与对应日志，
不能将此前 34/34 当作新增模型验证已通过。

修复使用既有 ONNX 后端，没有增加测试替身。验证脚本显式传入
`BUILD_ONNXRUNTIME=ON` 和 `OnnxRuntime_ROOT`；测试 CMake 检查实际公共库
的 `AUTONOMY_HAS_ONNXRUNTIME`，后端缺失时提前失败。脚本要求
`--defm-model` / `--defm-fixture` 容器绝对路径，以及真实导出清单
`--defm-manifest` 和 C++ SDK 根目录 `--onnxruntime-root`。

容器临时 SDK 使用 Microsoft.ML.OnnxRuntime 1.23.2 官方 NuGet 包中的
Linux x64 库与 C++ 头文件。包摘要、NuGet SHA-512 元数据一致性及实际解包
文件 SHA-256 记录在 `2026-11-18-v2.47-defm-onnxruntime-sdk.json`；这是下载
内容与同源元数据的核验，不声称独立发布签名验证。临时目录不构成生产安装。

启用后端后，真实 DeFM 测试通过，耗时 **0.60 秒**；该时间包含测试场景和
多次推理，不是单帧实时性指标。本轮完整 CTest 为 **34/35，16.40 秒**：
跨进程 `autonomy.planning.native_route_service` 在五秒的发现期限内没有得到
有效服务证据。失败记录保存在 `2026-11-18-v2.47-defm-onnx-validation.json`
及对应原始日志。没有放宽生产期限或将超时判为几何不可达。

同一源码的路线测试独立连续三次通过，原始 CTest 日志和记录为
`2026-11-18-v2.47-defm-route-discovery-repeat.log/.json`。这是重现检查，
并未确定先前发现超时的根因，不能声称已修复该偶发问题。

随后最终完整 SpaceHero configure、原生库及所有测试目标 build 均返回 0，
CTest **35/35 通过，11.29 秒**，包括真实 DeFM、共享样条、已有 NMPC/MPPI、
BT、探索和增量可见性图。记录为
`2026-11-18-v2.47-defm-onnx-final-validation.json` 及对应三份原始日志。
模型与 fixture 在测试前后摘要一致，测试注册/筛选覆盖全部 35 项。
全部 471 个 map 文件、TaskServer 及用户 Task/Planning CMake 保留。

DeFM 原生入口不改 map / TaskServer，不改用户 Task / Planning CMake。
公共样条、既有 NMPC/MPPI 和 automsgs 继续复用。实际编码器及接口通过
仍不等于 SanD 适配训练、四帧时序对齐、真实 SLAM 或实机探索完成。

## v2.48：原生控制器消费准入参考

`TrajectoryTracker::TickWithController` 复用原轨迹准入、底盘授权、停稳回执
与 C2 切换状态机，在这些门控通过后调用已有 NMPC/MPPI。原生控制器只
填写现有内部 `TrackingCommand` 的前向速度和 yaw rate，其余身份和绝对
期限不可修改。随后仍检查标定轮速/加速度/jerk 的必要过渡界，以及独立
实际指令、地面和停车扫掠回执。原生失败、异常或超时锁存停止请求，不
回退原有 `Tick` 公式恢复运动。

`BuildAdmittedPlannerPath` 通过 `TimedBSpline/common/math` 保留原准入
参考的非均匀结点、绝对 XYZ、yaw、执行帧和采样时间。原生 API 继续
使用已有 Path、PoseStamped 和 TwistStamped；没有新增 automsgs 消息。
私有准入实例及轨迹/地图/租约证据期限不随缓存或求解续期。

默认求解预算 20 ms、独立校验预算 5 ms，指令原绝对窗口保持 40 ms。
路径设置计入求解时间，未使用的求解预算不延长校验预算。求解和校验后
分别复查反馈新鲜度、时钟与所有权。新的实际 NMPC/MPPI 测试使用受控
采集 epoch 加真实 steady-clock 工作耗时，不冻结求解期间的时间。
实测采集时间转换为纳秒时向下取整，不放宽未来反馈拒绝。

本地旧构建首先失败，未进入 CTest：其 glob 纳入真实 DeFM 编码器后，
Opt 独立工程缺少原生 `common/network/backend/engine.hpp`。记录保留在
`2026-11-18-v2.48-admitted-controller-local-validation.json` 和 build 日志。
没有使用网络或消息替身掩盖这个环境缺口，后续检查使用真实 SpaceHero。

首轮 SpaceHero configure/build 成功，CTest 为 **34/35，11.40 秒**。
严格跟踪器测试通过，实际 NMPC 原生准入场景通过；MPPI 场景产生倒车
候选，被声明禁止倒车的准入底盘正确拒绝。该夹具尚未给 MPPI 配置前进
下限。原始失败保留在
`2026-11-18-v2.48-admitted-controller-native-validation.json` 及三个日志。

代码核对进一步发现已有 MPPI 的 `vx_min` 用零值判断“未设置”，不能
表达明确的前进下限零值。修复保持字段编号 8，以 optional presence
区分省略和显式零值：省略仍取原 -0.35 m/s 默认值，非零设置保持原
语义。测试显式设置 `vx_min: 0`，并检查配置序列化后仍保留零值 presence。
没有通过扩大底盘倒车许可或放宽执行门控让测试通过。

MPPI 在既有 Savitzky-Golay 滤波后重新施加既有速度、加速度和运动模型
约束，历史保存约束后的实际指令。测试使用 32 步实际预测序列，覆盖
超过 20 步才启用的原生滤波路径；不是用短序列跳过滤波。此项边界检查
不证明连续执行 jerk，也不代替滤波后实际指令的独立几何验证。

修复后的 SpaceHero configure、原生库及全部选定测试目标 build 均返回 0；
CTest **35/35 通过，12.68 秒**。测试注册与筛选覆盖全部 35 项，实际
DeFM 模型/fixture 在验证前后摘要一致。结果保存在
`2026-11-18-v2.48-admitted-controller-final-validation.json` 及对应三个
原始日志。实际原生控制测试耗时 0.12 秒，包含多个场景，不能作为单周期
求解延迟指标。全部 471 个 map 文件、TaskServer 与用户 Task/Planning
CMake 保留。

普通 PolicyPipeline 仍使用几何 Path 入口，尚未组装严格状态机。
独立实际指令扫掠的生产供应者、底盘停稳权威、看门狗、真实 SLAM 与地图
融合/保存仍需由部署应用接通。准入测试中的指令扫掠和底盘回执为受控
依赖；原生求解夹具明确采用宽轮动态界，不代表实机标定或执行验收。

## v2.49–v2.50：真实 AutoSim 启动与生产应用验证

2026-11-18，在 SpaceHero 内用真实 Habitat、17DRP5 场景 GLB 和现有
`autosim.runtime.runner.Runner` 运行了独立 350-step RGB-D 启动验证。
GPU 为 RTX 4090 D；RGB-D 配置 320×240、10 Hz，关闭 LiDAR、IMU 和
语义相机。所有输入、真值及 cmd_vel 话题都位于
`/validation/exploration/v2_49_rgbd/`，使用独立实例锁，验证程序没有发送
运动指令，保留原有用户 AutoSim 实例。

收到 RGB、depth、CameraInfo 各 70 帧，真值 OccupancyGrid 7 帧，
真值 PoseStamped、odom、clock 各 350 条，所有必需流至少重复发布两次。
原始启动尝试保存在 `2026-11-18-v2.49-autosim-startup-attempt.json`；
首次原生消息及 report 已按原摘要逐一验证并导出到
`docs/validation/2026-11-18-v2.50-autosim-recording/`。
真值仅为独立评估输入，不能作为算法地图或融合屏障的替代。

新增 `scripts/validate_native_applications.py` 使用现有 SpaceHero CMake
cache 构建真实 `autonomy.task`、`autonomy.planning`、`autonomy.control`
应用目标，并逐个检查 `--help`。本次应用 build 和三次 CLI 均返回 0；
记录为 `2026-11-18-v2.50-native-applications.json` 及四份原始日志。
这项结果补充此前的库/CTest 验证，没有启动共享 TaskServer 端点。
构建前后验证同步源码清单以及全部 471 个受保护 map 文件和 TaskServer。

截至 v2.50 尚未启动完整自主探索任务，也没有取得真实算法覆盖地图。
实际 `task_main.cpp` 仍只 Configure/Start TaskServer，未调用
`ConfigureNativeExploration` 安装平台定位与会话建图供应者。
在该次验证时，容器 build/bin 下也没有 `autonomy.localization` 应用产物。
既有 Atlas 源码有 RGB-D/稠密地图入口，但源码存在不能代替实际运行证据。
因此本次报告显式保留 `exploration_task_started=false` 和
`coverage_comparison_performed=false`，不得把启动或应用编译当作探索成功。

真实模型接口另存 `2026-11-18-v2.50-autosim-model-interfaces.json`。
SanD 模型文件实际存在，但其条件是四帧原始深度；当前 DeFM 导出图接收
480×640 深度并输出 tokens。启动验证的 320×240 也不是该图的静态
输入尺寸。完整运行需采用实际匹配的采集/标定与模型配置，并取得经过
验证的 DeFM 条件 SanD 模型；本次没有用重命名或 reshape 替代适配。

## 离线真值与算法覆盖地图比较工具

新增 `scripts/compare_exploration_maps.py` 直接读取已有 automsgs
`OccupancyGrid` 和 `PoseStamped` 的序列化 `.pb`，不定义新消息。
它在导航闭环之外运行，没有发布者，不给算法提供真值。

评价域固定为真值中起点连通的二维自由栅格，先按配置的圆形机器人
footprint 排除净空不足区域，再作八邻接搜索并禁止斜向穿越阻塞角。
真值未知、中间概率栅格和地图外部不计为可达自由空间。
算法图缺失的范围继续作为未知，不能通过缩小算法图改变分母。
该域只是明确的静态二维评价模型，不证明三维可通行或动态环境可达性。

两图必须已处于同一 frame、相同 resolution 和对齐的栅格原点。
支持整数栅格偏移和不同范围；不同 frame、不同 resolution、旋转原点
或非整数原点偏移均拒绝，要求在评估之前明确完成对齐或重采样。
工具不拟合一个有利于得分的变换，不把未知当作自由空间。

输出分别包含已知覆盖率、正确自由空间覆盖率、可达域错误占用率、
真实障碍物被判为 free 的计数，以及剩余 unknown / 未确认 free 连通块。
概率介于 free 与 occupied 阈值之间的格子独立列为 uncertain；这类
格子可以是已知，但没有确认可通行。数据长度、占用值、有限数、尺寸、
footprint 计算工作量及文件大小都先作有界校验。

执行时设置原生生成消息路径，例如：

```bash
PYTHONPATH=/workspace/autonomy/build/autonomy/automsgs/proto/gen/python \
  /opt/venv/bin/python3 \
  /workspace/autonomy/src/autonomy/scripts/compare_exploration_maps.py \
  --ground-truth /absolute/recording/ground_truth_map.pb \
  --algorithm-map /absolute/recording/algorithm_map.pb \
  --start-pose /absolute/recording/start_pose.pb \
  --robot-radius 0.25 \
  --report /absolute/recording/map_comparison.json
```

脚本拒绝真值与算法输入为同一文件或相同字节；仍显式输出
`independent_map_provenance_verified=false` 和 `exploration_accepted=false`。
独立采集来源、任务终态、真实地图融合/保存、时序以及控制执行证据仍需
由实际运行提供，单份地图得分不证明它们已经成立。

`scripts/tests/test_compare_exploration_maps.py` 使用真实生成的 automsgs
类型构造小型合成回归地图，覆盖未知、错误占用、中间概率、不同地图
范围、封闭区域、禁止切角、footprint 与非法输入。它们只验证离线指标
语义，不能作为 AutoSim 探索或 SLAM 融合的替代证据。

本次在 SpaceHero 内使用实际生成的 automsgs Python 模块执行上述回归，
**11/11 通过**，日志为
`docs/validation/2026-11-18-v2.50-offline-map-comparison-tests.log`，
记录为同名前缀 `.json`。这个结果不改变完整探索尚未启动的验收状态。

## v2.51–v2.57：定位应用与真实 RGB-D 建图

定位依赖配置的实际失败来自 Argus ORB 私有依赖名称与消费者不一致。
第三方 CMake 原来创建 `atlas_orb_*` / `atlas::orb_*`，Argus 消费者要求
`argus_orb_*` / `argus::orb_*`，已在 Argus 的私有构建文件中统一。
父工程已经选定 OpenCV 5 时，Argus 不再重新查找另一套 OpenCV 4；
SpaceHero 使用 `/usr/local/lib/cmake/opencv5`。

修正后四个实际应用 `autonomy.task`、`autonomy.planning`、
`autonomy.control`、`autonomy.localization` 均完成构建与 `--help` 检查。
证据为 v2.52、v2.53 的 `-localization-applications.json` 及原始日志。
这没有修改 TaskServer、Task/Planning CMake 或 471 个 map 文件。

为隔离验证补齐原生定位 CLI、LocalizationOptions 和 AtlasBackend 的
七项输出话题转发；默认值保持已有 AtlasNode 的约定。全部复用已有
automsgs 类型。验证实例的传感器、算法输出和评估真值分别使用同一个
`/validation/exploration/v2_57_rgbd/` 前缀下的不同话题；Atlas 的 GT 输入、
GT 轨迹输出及动态 TF 发布显式关闭。

前三次实际启动分别暴露配置解析错误，原始失败记录仍保留：v2.54 的
主点 `320` 被 OpenCV 识别为 integer，而 Atlas 要求 real；v2.55 的
`Camera.fps: 10.0` 被识别为 real，而 Atlas 要求 integer；v2.56 缺少
即使关闭 GUI 仍必需的 `Viewer.*` 浮点字段。v2.57 配置测试使用真实
OpenCV FileStorage 验证这些类型，**4/4 通过**。

v2.57 随后使用真实 Habitat、17DRP5 场景和真实 `autonomy.localization`
运行 2500 步，640×480 RGB-D、10 Hz，关闭 LiDAR 与 IMU。相机内参来自
实际 CameraInfo，深度使用米单位 32FC1，安装高度从完整静态 TF 链得到
0.21 m。收到 RGB、depth、CameraInfo 各 500 条；Atlas 发布估计里程计
498 条、轨迹 249 条、算法占用栅格 50 条，最终算法图含 3073 个已知
格子；程序退出码为 0，运行约 54 秒。

源清单和受保护文件在运行前后均通过摘要检查。原始记录已导出到
`docs/validation/2026-11-18-v2.57-rgbd-localization-recording/`，其中
20 份首末消息、模拟器配置、相机配置与程序日志共 **23 个文件**逐一按
报告摘要验证。启动与退出记录为 `-attempt.json`；标定测试记录为
`-rgbd-calibration-tests.json`。这些是实际运行证据，不再仅由源码推测
Atlas 是否能够产生算法地图。

这次没有创建速度发布者，起止真值位姿都为 (-4, 1, 0)。因此它验证的是
静止 RGB-D 跟踪与建图入口，**没有启动自主探索任务**。算法地图的
`frame_id=atlas_optical_map`，原点四元数为绕 X 轴旋转 90 度，对应 XZ
投影；真值图为 `map` 中的 XY 投影。二者不能改 frame_id 后直接比较。

## 显式离线投影

`scripts/project_exploration_grid.py` 使用已有 automsgs
`TransformStamped`，将原算法图的栅格原点姿态与显式世界变换组合，然后
投影到评价栅格。目标图只提供尺寸、分辨率和原点；目标占用值不参与
算法格子的分类，也不拟合地图以提高得分。原始输入和变换摘要全部保留，
输出是新的 `.pb` 文件，没有发布者或控制接口。

每个目标格子在源栅格中的完整反投影包围盒必须全部已知；盒内任何未知
或地图外部使结果继续未知，其余取最大占用概率。该保守规则在旋转时
可能包含额外源格子，会降低自由覆盖率；不能解释为精确面积积分。
倾斜平面拒绝投影。平行平面的高度不一致时，必须显式设置
`--allow-parallel-plane-projection`，记录原高度差；这仅是二维评价约定，
不制造地面测量或三维净空。原来的 `compare_exploration_maps.py` 默认
仍拒绝未对齐、旋转或不同分辨率的输入。

v2.59 在 SpaceHero 中使用真实生成的 automsgs 执行投影回归 **7/7**、
既有比较回归 **11/11**，均通过。v2.58 首次回归中，一项旧测试因错误
提示缺少兼容的 `normalized grid` 文本而失败；v2.59 恢复该文本，没有
改变指标或投影规则。两次结果和原始日志均保留。

v2.60 补充投影输出写入失败的 CLI 回归：目标目录不可写时仍生成明确的
失败报告，`projected=false`，保留原始文件，不因状态字段缺失而再次异常。

随后对 v2.57 的真实静止记录执行离线比较。固定变换来自初始 GT 底盘
位姿、实际静态相机安装变换及第一份算法相机位姿，未利用占用格子拟合。
第一份 GT 与第一份估计相差 0.180555344 秒，因此报告明确记录
`stationary_initial_pose_assumption=true`，未声称严格同步的移动轨迹
关联。安装链为 `base_footprint→base_link→camera_link→camera_optical_frame`，
平行投影的高度差为 0.21 m。真值只在离线评价器中读取。

真值图为 327×165、0.05 m。采用显式 0.25 m 圆形 footprint 后，起点
连通的二维自由评价域为 6449 个格子、约 16.12 m²。原算法图的 3073 个
已知格子经过保守投影，在完整真值格网上留下 1658 个已知格子；评价域中
329 个已知、125 个确认自由，分别对应 **5.10% 已知覆盖**与
**1.94% 正确自由覆盖**。这只是起始观测，不能代表探索终态。
评价域中还有 204 个格子被算法判为占用；全图有 12 个真值占用格子被判
为自由。地图判断差异也需结合二维投影约定与后续移动观测分析，不能由
这份分数推导控制安全已经通过。

原始变换、投影图、投影报告、比较报告和变换来源记录位于
`docs/validation/2026-11-18-v2.59-initial-map-evaluation-recording/`；
执行程序及前后源码摘要校验保存在同版本的 `-attempt.json`。
报告保留 `exploration_task_started=false`、`exploration_accepted=false`
及 `algorithm_map_is_measured_coverage_ledger=false`：当前比较的是 Atlas
占用图，尚未获得探索任务的实测覆盖账本。

完整探索的主要接入仍未完成：实际 `task_main.cpp` 没有安装
`ConfigureNativeExploration` 的定位与建图供应者；MappingPipeline 的
`Flush`、快照和计数尚未与探索消费者的有限观测前缀及实际保存建立
版本对应；普通 PolicyPipeline 尚未接入严格执行状态机。另一个限制是
当前 SanD 图接收原始深度，没有经过验证的 DeFM token 条件权重。
不能通过消息计数、图像复制、模型重命名或测试回执填平这些接口。

Scheduler 已安装版本化导航结果、取消和 movement admission 回调。
TaskServer 的兼容 `SetNavigationProbe` 是空操作，不会覆盖这些回调；
无需再次安装或修改受保护 TaskServer。

## v2.61–v2.64：原生延迟定位、精确采集时间和定位缓存

v2.61 给 NativeObservationSource 增加了显式启用的有界关联等待。
RGB-D 和 LiDAR 独立保留原始消息、时间和标定；超时、消息/字节容量溢出
分别保留原始拒绝序号。shutdown 唤醒关联线程，并由 ReaderRetirement
完成 Reader 关闭和线程 join。SpaceHero 中原生回归 35/35 通过；其中
原生观测源增加延迟采集、FIFO 序号、两源隔离、有效期、关闭和最后引用
释放等真实 Reader/Writer 用例。它们的定位 authority 是明确的受控测试
依赖，不能代替真实视觉跟踪验证。

v2.62 添加实际 Atlas `DiagnosticArray` 状态输出。四个实际应用
`autonomy.task`、`autonomy.planning`、`autonomy.control`、
`autonomy.localization` 完成编译和 CLI 检查。第一次运行误导入旧安装
目录的空 Autolink Python 包，在创建真实传输前失败；记录保留该失败。
修正为当前 build/python 下的实际 `_core` 后，真实 Habitat + Atlas 运行
通过，但只有 279 组健康状态与位姿关联。检查原始 Protobuf Header 发现
ImageFrame 的 double epoch 时间重编码产生数百纳秒差异，不能用该差异
判断不同测量，也不能放宽为最新位姿匹配。

v2.63 的 MessageAdapter 保留原始输入整数秒/纳秒，写入 Odometry 顶层、
嵌套位姿和 Path Header。Atlas 状态也使用同一原始输入时间。内部 ORB
跟踪继续使用 double；消息身份不再经过该 double 重建。新增原生桥接
测试验证大 epoch 时间、两个 double 重合的原始整数时间、秒边界，以及
坐标变化时旧路径/位姿失效；链接实际 atlas_automsgs_bridge，测试通过。

随后重新运行实际 Habitat、17DRP5 场景和 RGB-D-only Atlas：640×480、
10 Hz、2500 steps。收到 RGB、depth、CameraInfo 各 500 条，Atlas
Odometry 和定位状态各 498 条、Path 249 条、算法 OccupancyGrid 50 条。
**498 组原始深度、健康状态、估计位姿的整数 Header 时间完全一致**；
没有因 double 时间重编码产生的假降级。地图包含 3073 个已知格子，
Atlas 退出码为零，用时约 54.98 秒，GT 输入保持关闭。

该次验证仍为静止定位建图，没有启动探索、发送运动或产生实测探索覆盖
账本。证据为 `2026-11-18-v2.63-rgbd-acquisition-attempt.json`；
原始记录位于 `2026-11-18-v2.63-rgbd-acquisition-recording/recording/`。
26 个记录文件已逐一按摘要验证，见同版本 `-export.json`。v2.59 的
起始地图比较没有因此升级为完整探索结果。

v2.64 的 `NativeAtlasLocalization` 为平台提供原生定位 lookup：两个
半消息只有按整数时间、帧和实际后端身份配齐后才能成为证明。世界对齐
须由平台显式提供，并锚定当前健康估计位姿；相机和底盘位姿由实际安装
变换组合。后端重启或坐标变化撤销旧对齐，不能复用旧消费者 map epoch。
错误源/帧、非对应采集时间、过期证明、矛盾消息和模糊 double 查询均拒绝。
原生回归 **36/36** 通过，记录为同版本 `-validation.json` 和原始日志。

这些改动已同步到远端对应目录；471 个 `autonomy/map` 文件、TaskServer
及用户 Task/Planning CMake 摘要保持一致。完整目标仍未通过：实际
`task_main` 平台装配、指定观测的真实融合与保存凭据、移动探索任务及
GT 对实测覆盖账本比较仍需继续完成；DeFM token 条件 SanD 权重也未
建立真实验证。不能以定位状态、原生回归或静止占用图替代这些验收项。
