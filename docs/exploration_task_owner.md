# ExplorationTask：会话所有权与任务生命周期

版本：v2.9。日期：2026-11-18。

本文对应 `autonomy/task/exploration/exploration.hpp/.cpp`、`exploration_session.cpp` 和 `exploration_owner_test.cpp`。十三类 owner 隔离场景通过后，修复已正式同步并进入 SpaceHero 原生构建和八项 CTest，8/8 通过。系统目标与剩余工作见 [最终方案](multisensor_exploration_review.md) 和 [详细设计](multisensor_exploration_design.md) §20.14；真实服务及完整任务未验收。

## 1. 线程与所有权

`ExplorationTask` 保留 `TypedTaskAppBase<ExplorationGoal, ExplorationFeedback, ExplorationResult>` 的原生任务表面，`Impl` 管理 inbox、owner 线程及其运行状态。

- task owner 调用 `create_session`、`Session::Start`、会话 Tick 和树生命周期操作，释放其持有的 Session/Tree。
- Session 的规划工作线程负责策略计算及 `strategy->Reset()`。Reset 不证明 Session 已启动，也不要求它位于 task owner。
- NavigationDispatcher 的独立工作队列调用可能等待 readiness 的原生提交与取消服务，避免 owner 被这些调用阻塞。
- 原始观测、位姿、地图和任务命令进入有界 inbox；生产者回调不推进会话或行为树。
- PlannerServer 拥有长期 Vista 图，MappingTask/上游建图拥有真实地图与保存。owner 不取得这些模块的内部对象所有权。

Session 构造/释放的线程证据与策略工作线程证据必须分别核验。测试注册表仅保存受保护的非拥有指针，读取快照不延长 Session 的寿命或改变析构线程。

## 2. 原生依赖注入

`ExplorationTaskBindings` 提供：

- `create_session(goal, options)`：返回新的、尚未 Start、处于 Idle、配置为 BT 驱动的 Session。禁止 factory 启动会话或保留可变 owner。
- `prepare_mapping(map_name)`：确认真实建图准备。成功语义由真实上游适配提供，不能只凭接受地图名字或发送切换请求判断就绪。
- `clock()`：与观测、位姿和 owner 计时一致的时钟。
- `options`：初始策略参数；任务目标及允许区域按版本配置。

`SetGuardedSubmitNavigation` 安装可撤销许可感知的原生接受接口；`SetVersionedNavigationProbe` 安装按 task_id 查结果及取消的服务。`NavigationServices(frame_id, planner_id)` 只允许当前代的 factory 在 task owner 上调用，返回该代的异步服务；默认路线实例为 `vista_planner`。

只安装旧 `SubmitNavigation` 或 active/result probe 不能满足身份关联与 cooperative 接受契约。缺失必需服务时明确拒绝，不将旧 probe 合成为成功凭据。依赖安装须处于 idle/terminal 且无在途运行。

## 3. 初始化与最终启动交接

启动处理以 generation 绑定所有候选和服务：

1. 验证并接收任务，创建本代运动门控与初始化状态。
2. 在 StartReady 补齐该代尚未完成的建图准备，记录 `mapping_prepared_`。
3. 准备回调在 inbox 锁外调用，返回后复核 generation 与运行意图。
4. 等待真实种子地图、位姿及配置依赖；过期或超时进入失败清理。
5. factory 返回未启动候选，owner 验证它的 Idle/BT 驱动契约。
6. 时钟和 freshness 等外部检查在锁外完成。紧邻 `prepared_->Start` 的最终交接在 inbox 锁内检查 stopping、当前 generation、运行意图和启用许可。
7. factory 或 final-clock 期间取消/抢占时丢弃旧候选；暂停时保留未启动候选，恢复后重新校验。

最终 Start 使用短交接锁的前提是经过验证的 Idle Session；此时 Start 只做内部初始化，没有在途导航、地图或规划回调。不能将任意可阻塞外部调用移入该锁。

## 4. 暂停、恢复与持续监测

Pause 撤销旧运动许可并保留阶段。owner 在树推进前按最新 inbox 意图执行 `ReconcileBeforeTree`，避免已提取的旧命令批次覆盖更新意图。暂停期间不推进运动树，直接对 Session 做有界监测。

初始化暂停不消耗恢复后的种子等待预算；`initialization_resumed_` 使恢复重新开始有界等待。暂停发生在准备之前时，恢复必须先准备建图，再允许 Start。

暂停期间可以消费有效观测、健康与覆盖要求，但停留时间不能据此兑现。Session 每次 Publish 从真实 CoverageState 更新 `pending_units`，包括暂停和终态的早返回路径。收到要求与发布计数同步，不凭缓存的旧反馈判断完成。

运动资格恢复使用新 lease。阶段恢复仍需复核地图、定位、传感器和参考版本；暂停不使旧几何或旧请求重新有效。

## 5. 失败清理、取消与关闭

准备拒绝、种子超时和 factory 异常统一进入 Run 的清理路径：撤销本代许可、停止 Session/Tree、取消匹配的在途服务、释放候选与暂存，清理完成后释放 `runtime_busy_`。仅发布 FAILED 或清空 waiting 不足以完成清理。

取消/停止不能确认时保留 motion fault 并拒绝新的运动。不得为了接收下一项任务而清掉失败的取消依据。

Shutdown 先关闭输入和任务接收、撤销派发资格、停止在途工作，再由 owner 释放会话/树，最后释放依赖。回调重入 Shutdown 只请求关闭，由外部拥有者 join；禁止自己的工作线程 join 自己。真实 TaskServer 关闭顺序仍须与此依赖顺序一致。

## 6. 输入、身份与反馈

`PushMap`、`PushPose`、`PushObservation`、`PushCoverageUnit` 的 bool 返回值只表达是否接收入队。上游处理计数、覆盖兑现和完成截面有独立语义；队列拒绝或缺失必须进入反馈及完成复核。

`GetExplorationSnapshot()` 返回一致快照，未启动时没有实际 session 身份。任务结果由覆盖、返航、保存及配置要求决定，树的 SUCCESS 或 legacy finished Bool 不直接构成任务成功。

请求身份需关联进程启动身份、task/session/request/generation/goal/epoch。迟到回执只影响匹配的仍然有效请求，同位置的新身份仍是新任务。

## 7. 实际专项证据

最终 [验证 JSON](validation/2026-11-18-v2.8-owner-final-owner.json) 保存 source_sha256、原生编译/链接命令、返回码及日志 SHA256；[场景日志](validation/2026-11-18-v2.8-owner-final-owner-owner-scenarios.log) 包含十三项 PASS：

1. `factory_cancel`：构造返回后的取消不会晚启动。
2. `factory_pause_resume`：暂停保留候选，恢复按有效意图启动。
3. `mapping_pause_prepare`：准备前暂停恢复不能绕过准备。
4. `mapping_refusal_cleanup`：准备拒绝后释放运行占用。
5. `seed_timeout_cleanup`：种子超时后执行统一清理。
6. `factory_preemption`：新启动使旧构造结果失效。
7. `cooperative_bindings`：旧或缺失服务不满足接受契约。
8. `factory_clock_cancel`：最后时钟检查期间取消阻止 Start。
9. `factory_clock_pause_resume`：最后时钟检查期间暂停保留未启动候选。
10. `paused_initialization_budget`：暂停时长不误计入恢复后的初始化预算。
11. `paused_monitoring`：暂停仍处理覆盖要求并发布实际待处理计数。
12. `native_submit_cancel`：受控提交/取消路径保持版本责任。
13. `native_shutdown_reentry`：受控原生回调重入关闭不死锁。

baseline、baseline-start-gate 与 fixed-owner-monitor 的失败日志分别保留；最终通过没有改低测试要求。现有 `task_interface.hpp:89` 的 extra semicolon 编译警告仍在。

## 8. 尚需完成的真实服务接入

十三类隔离 owner/Session 场景已经另纳入正式原生构建与相关 CTest；外部服务的部分时序仍由测试控制。以下不在其完成范围：

- NavigationTask 在 readiness、接受、树启动前及实际执行入口持续复核许可，取消不被长 readiness 锁阻塞。
- 父探索运动槽与委托导航调度，普通导航抢占与 TaskServer 依赖关闭顺序。
- 所有观测/返航/保存阶段的真实服务暂停恢复、队列饱和和跨进程故障。
- 三模式原始生产者、实际融合/保存截面及机器人完整任务验收。

后续部署按 expected SHA256 检查冲突、保存原内容、逐文件同步。用户 TaskServer 修改必须合并。`autonomy/map` 的任何代码、接口与消息保持不变。

## 9. 正式部署与原生验证

2026-11-18，owner/Session、owner 测试、测试 CMake 与本说明共六项文件按白名单核验、备份并同步。见 [同步清单](validation/2026-11-18-v2.8-owner-sync-manifest.json)。这是说明更新之前的部署指纹；本地 v2.9 说明更新独立记录，不能回写成原部署摘要。

SpaceHero 正式目录的 planning/task/control 与八个相关测试目标构建成功；八项正式 CTest 8/8 通过，2.13 秒，包含 exploration owner。见 [正式验证](validation/2026-11-18-v2.8-owner-native-validation.json) 与 [CTest 日志](validation/2026-11-18-v2.8-owner-native-tests.log)。编译存在既有宏、Autolink 参数和运行库路径警告，不宣称零警告。

部署后核验确认六项部署内容匹配指纹，map 471 文件保持不变，TaskServer 用户修改保持原状。见 [只读核验](validation/2026-11-18-v2.9-review-native-postdeploy-check.json)。首次启动器失败发生在 CTest 之前；原错误日志被重试覆盖，验证 JSON 如实保留缺失原因，成功阶段日志摘要均匹配。

正式专项通过没有补齐 §8 的原生执行门控、父子调度、真实传感器与建图服务，不代表完整探索任务已验收。
