# 探索导航派发器

实现：`autonomy/task/exploration/integration/navigation_dispatcher.hpp/.cpp`。

## 职责与线程

NavigationDispatcher 将可能等待原生 readiness 的提交移出 BT/Session owner。Enqueue、CancelRequest 只操作短锁保护的队列；独立工作线程按顺序调用原生 Submit 与 Cancel。Poll 读取按 task_id 关联的终态，不能从导航 idle 推断到达。

一条派发器绑定一个探索 generation 和 MotionGate。服务闭包持有共享 gate 与不可变原生回调；不保留可悬空的 Session 或 ExplorationTask 指针。外部 owner 必须在释放原生导航依赖前关闭并排空派发器。

## 运动许可

MotionGate 初始禁用。启用后，Lease 捕获当前 revision；Pause 改变 revision 并禁用，Resume 发放新 revision，旧 lease 永久失效。Retire 不可恢复。每个队列请求还有独立的不可逆 live token。

原生 Submit 必须持续检查 permit：等待 readiness、接受身份、启动树以及执行入口。派发器负责撤销/接受后的按 ID 取消，但不能替代原生执行门控。

## 接受与取消责任交接

请求经历排队、正在提交、接受或拒绝、终态。CancelRequest 立即撤销对应 live token，不能取消更新的请求。

Submit 返回后，工作线程在同一队列 mutex 内检查 stopping、失败状态、请求身份和 permit，再公布接受及取消责任。取消先发生时，接受线程负责晚接受的取消；接受先公布时，CancelRequest 接管。cancellation_queued 保证取消只有一个所有者，native Cancel 在锁外执行。

同一版本及相同 pose 可视作重复提交；同版本但不同 pose 拒绝。同 pose 的新身份仍为新请求。

## 错误与关闭

- Submit 抛异常时接受结果不明确，按 ID 排队取消；Result 抛异常时取消并报告失败。
- Cancel 抛异常意味着无法确认运动已停止；派发器设置 Failed 并 retire gate，禁止该代进一步运动。
- Shutdown 关闭入口、撤销许可、排空取消并 join。工作线程回调重入 Shutdown 只提出关闭，外部 owner 执行 join；禁止从工作回调销毁仍在执行的派发器。
- 许可撤销不等于原生已完成停止。依赖释放必须等待派发器及原生取消契约完成。

## 回归范围

测试为 `autonomy/task/exploration/navigation_dispatcher_test.cpp`，包含八类场景：lease 暂停恢复/退役、阻塞提交期间取消、5000 次接受/取消交接、身份及拒绝、Submit/Result 异常、取消失败、关闭重入和提交期间暂停恢复。

本地目标为 opt_navigation_dispatcher_tests，原生目标为 autonomy_navigation_dispatcher_test，CTest 名为 autonomy.exploration.navigation_dispatcher。原生测试链接真实 autonomy_task/planning/support，不制造 SDK 替代实现。

这些有限场景验证派发器模块。ExplorationTask owner、原生 guarded acceptance、父探索/委托导航调度、真实观测及地图保存必须另行验收。
