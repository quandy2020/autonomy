# 原生 BT 执行前检查与生命周期

日期：2026-11-18。本文件说明 BtRunner 的实际修改及验证边界；完整探索方案见 [审查入口](multisensor_exploration_review.md)。

## 执行前检查

`BtRunnerHooks::BeforeTick()` 在每次未暂停的 `tree.tickOnce()` 之前调用。默认返回 true，保留没有外部许可的任务行为。`BtTaskApp::BeforeTreeTick()` 是派生任务的同一入口；检查失败结束当前树并调用 halt，不执行这一轮节点。

检查返回后再次读取取消与暂停状态，防止检查期间发生的撤销仍执行节点。暂停期间继续监测；恢复后重新检查。`OnTick()` 是执行后的监测及终态通知，不能用于阻止当前节点已经产生的动作。

这不是底盘运动停止的完整实现。暂停树不自动停止已经派发的异步控制；取消树也不是远端控制已确认取消的凭据。NavigationTask readiness、接受和 ActionSession 实际 dispatch 必须贯穿同一可撤销运动许可，父探索/子导航的运动槽继承仍须接入。单次 pre-tick 检查不能消除检查与派发之间的竞争。

## 等待、重入与清理

- 外部 Configure/Run/SetHooks/Cancel 的管理操作串行；旧 worker 完成清理后才能复用配置、树和回调。
- worker 回调内 Cancel 只提出取消请求，不 self-join。回调内 Configure/Run 拒绝，SetHooks 不替换当前 host。
- 取消和暂停/恢复通过条件变量唤醒等待；状态和唤醒代数在等待 mutex 下修改，防止等待边界丢通知。
- 树、节点与 logger 在 worker 上完成清理后，才发布 `IsRunning()==false`。回调 host 必须活到外部 owner 完成 join。
- setup、执行前检查、节点 tick、监测、状态日志和 halt 的标准或非标准异常转换为失败。取消不把已发生的回调/清理故障覆盖成正常取消。
- 回调必须有界返回。取消通知不能打断任意阻塞的第三方调用；外部 Cancel 的 join 延迟仍取决于当前回调及 halt。析构不能从 worker 回调中执行。

新增虚函数改变 native task 的 ABI。隔离对象链接检查不能代替正式 task 及相关派生实现重编，也不能热替换仍运行的旧进程。

## 有意义的原生测试

`bt_runner_test.cpp` 使用真实原生工厂、XML、Script 黑板计数、StatefulAction 的启动/继续/halt 效果，以及真实 BtTaskApp 桥接；没有替换 runner 循环。

十一类顶层场景为：默认执行；首 tick 拒绝；运行中撤销；检查期间外部取消及回调重入；检查期间暂停；五秒周期下取消及时唤醒；暂停/恢复在等待边界唤醒；取消与回调异常并发保留失败；四类回调的标准/非标准异常；halt 异常保留；typed host 拒绝后真实脚本效果为零。

受控回调用有期限的条件变量协作。取消/恢复对五秒循环的检查使用小于两秒的测试限期，仅证明该测试的有界节点行为，不是机器人制动时延或端侧最坏时延。

2026-11-18 隔离 native 编译、真实库链接与十一类场景均通过，见 [隔离证据](validation/2026-11-18-v2.10-bt-stage.json)。正式同步、重编及相关 CTest 的结果单独记录；最终审查稿只引用已有通过记录。

## 后续接入

1. NavigationTask 对 readiness、身份接受和执行前检查使用同一代运动许可；取消不等待长 readiness 锁。
2. 许可核验延伸到实际 ActionSession dispatch，接受/取消责任交接包含晚接受处理。
3. 父 ExplorationTask 持有运动槽，委托导航继承，外部抢占先撤销父代并取消子执行。
4. 调度关闭先关闭入口和撤销许可，再排空派发/取消、销毁 owner，最后释放导航与建图依赖。

这些接入完成前，十一类 runner 场景或整个相关 CTest 通过都不能宣称完整运动许可契约已经验收。
