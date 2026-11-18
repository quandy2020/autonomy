# NavigationTask 原生接受、版本与生命周期

日期：2026-11-18；实施批次：v2.12。

本次修改直接接入 `autonomy/task/navigation/navigation.hpp/.cpp` 与原生 BtRunner，不是独立演示导航器。目标是让 typed-task 的接受身份与真正运行的导航调用一致，并使取消能够中断服务准备。完整探索的外部运动资格、跨进程 action 派发和真实停止确认仍需后续闭合。

## 1. 接受和执行的区别

`SubmitGoal` 返回的是领域接受，不能当作已到达。typed-task 基类仅在 `OnGoal` 接受后提交任务身份，等待期间禁止把旧结果关联到新请求。

START 和 REPLAN 替换任务身份。没有 header 的 START 是新调用，清除上一项身份。PAUSE、RESUME、STOP 和 CANCEL 只控制相应工作，不改变已接受的 task/client 身份。

同一位姿、不同 task ID 要启动新树。只有相同非空 task ID、client ID 和完整 protobuf 内容的 START 重试复用当前调用。REPLAN、插件或其他请求字段改变会启动新调用。精确重试会撤销更早的等待替换，避免 typed 接受顺序和领域安装顺序分离。

## 2. 服务准备与替换

顺序为：检查命令和几何 → 检查依赖及 XML 普通文件 → 保留 pending epoch → 有截止时间地探测服务 → 获取交接锁 → 重验请求版本 → 撤销旧 tick 资格 → 停止并 join 旧树 → 安装新调用 → 启动新树。

几何检查包含非空目标、一致非空 frame、有限坐标、单位四元数及有效导航模式。单点模式只允许一个目标。此处文件检查不声称预先验证 XML 内容和所有节点；实际树构造、插件及节点错误由原生 runner 记录为运行失败。

服务等待默认二十秒，每次等待最多五十毫秒，并可由取消、暂停、替换和 Shutdown 唤醒。等待不持目标交接锁或生命周期锁。准备超时、准备异常、无效几何或文件缺失不会停止上一项已接受导航。

新请求替换 pending slot，旧请求只释放自己的 slot。被替换或撤销的请求返回 false。交接和启动后重新核验版本，过期调用不得取消较新的调用。

运行状态在 `Run` 之前设置。真实 BT 即刻完成后，接受线程不再把终态覆盖为 Running。调用身份和 epoch 一起进入 worker blackboard，节点可以使用真实领域身份，而非读取尚在提交中的公共元数据。

## 3. 四种身份与锁

- `goal_epoch`：最新请求代数，旧等待者据此退出。
- `pending_epoch`：尚在准备的请求，未发布为已接受工作。
- `active_epoch / tree_epoch`：已经安装的调用及其 worker 所有权。
- `motion_epoch`：当前调用的 tick 资格，暂停、取消和终态撤销。

生命周期 mutex 保护领域状态；goal mutex 保护交接及外部控制；ready mutex 只保护条件变量等待。停止/join worker 时不持生命周期 mutex。worker 回调通过 `BtRunner::IsWorkerThread` 识别重入，取消、暂停及关闭不等待正被外部 join 的目标交接锁；回调重新 START/REPLAN 明确拒绝。

该 tick 资格不是完整的跨进程 dispatch lease。原生 ActionSession 的实际 send、父探索委托资格以及迟到接受的取消确认仍须贯穿同一外部许可。

## 4. 控制与故障

`CancelForTask(id)` 只取消匹配 pending/active 工作。旧任务取消不会清除另一项新请求的 pending slot，也不会停止已安装的新树。无目标 ID 的公共 Cancel 对全部当前工作生效。

暂停撤销匹配等待及当前 tick 资格，保留已接受树和身份；恢复只允许保留调用。控制请求中的非空 ID 用作匹配条件。取消中的 epoch 标记防止取消路径重入后重复派发本地清理。

终态处理先清除 active 所有权，再调用远端取消依赖；异常记录为 Failed，禁止进一步 START 或 Resume，直到显式重新初始化。真实节点 halt 失败也保留 Failed，不冒充干净取消。

生产 NavigationClient 现有 `CancelActiveMotion` 仍采用异步取消且部分异常被其自身处理；本批次不把依赖返回、树暂停或树停止当作底盘已经物理停止的 ACK。这一生产取消契约及外部 dispatch 许可属于下一项接入任务。

Shutdown 先关闭领域入口并唤醒等待者，再取消与停止 worker，最后由 typed 基类关闭接受和清除公共状态。base Shutdown 会再次调用虚拟 Cancel，因此不能把非递归 goal mutex 持有到 base 调用中。worker 请求 Shutdown 只请求停止，最终 join 和对象销毁仍属于外部 owner。

## 5. 原生验证覆盖

`navigation_task_test.cpp` 使用真正 NavigationTask、typed 基类、Protobuf、注册器、XML、worker 和 StatefulActionNode。仅控制 RPC 可用性、截止时间和取消依赖，不模拟 OnGoal 或执行循环。

验证包括同位姿新身份、精确重试、REPLAN/插件变化、匿名 START、无效请求保留旧调用、准备超时、定向 pending 取消、等待替换、重试撤销旧 pending、取消旧 active 保留新 pending、带 header 控制、快速完成、等待期间关闭/暂停、worker 回调取消/启动拒绝/关闭/暂停恢复、准备异常、真实 halt 失败、取消依赖异常和 join 期间的请求替换。

隔离阶段的 **21 类场景全部通过**，链接实际 native 库。记录见 [隔离验证](validation/2026-11-18-v2.12-navigation-stage.json) 和 [场景日志](validation/2026-11-18-v2.12-navigation-native-navigation-scenarios.log)。较新等待者的替换用真实 readiness 调用线程身份同步，避免把调度延迟误当为 supersession 行为。

六项白名单已按 expected SHA256 核验、备份并同步：[同步清单](validation/2026-11-18-v2.12-navigation-sync-manifest.json)。SpaceHero 正式 checkout 的 planning/task/control 与十一个相关测试程序配置、编译成功；正式 CTest **11/11 通过，2.67 秒**，新 `autonomy.task.navigation` 用时 0.41 秒。构建日志包含 `navigation/navigation.cpp.o` 与 `task_server.cpp.o`，确认类布局变化后的实际消费者已重编。见 [正式记录](validation/2026-11-18-v2.12-navigation-native-validation.json)、[构建日志](validation/2026-11-18-v2.12-navigation-native-build.log) 和 [CTest 日志](validation/2026-11-18-v2.12-navigation-native-tests.log)。

[部署后核验](validation/2026-11-18-v2.12-navigation-postdeploy-check.json) 确认六项部署指纹匹配、map 471 文件未改、TaskServer 用户内容保持、SpaceHero running。21 类隔离场景与 11 项 CTest 各有范围，不合并成全系统通过数字。已有配置和头文件警告仍在；没有启动机器人或验证物理停止。

## 6. 对完整方案的边界

本批次处理真实导航领域接受和 worker 生命周期。三模式观测生产者、独立定位、跨进程 Vista、任务父子调度、完整公平扫尾和有限完成截面、真实融合保存、DeFM/SanD 训练、物理时间样条/三维 Safety/跟踪及实机验证仍不能由这些专项结果推出。
