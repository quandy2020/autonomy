# TypedTask 目标接受与身份一致性

日期：2026-11-18。对应 `autonomy/task/common/typed_task.hpp` 与原生专项 `typed_task_test.cpp`。

## 问题与行为

旧 SubmitGoal 在调用 OnGoal 之前写入 task_id/client_id。若旧任务已成功，而新目标被拒绝，GetSnapshot/GetResult 的组合可能把旧成功关联到新目标；接受等待、异常和并发回调还可能导致晚返回覆盖较新身份。

现在 OnGoal 在生命周期 mutex 之外执行；只有接受后才提交身份。单调票据对接受顺序进行关联，晚返回的旧接受不覆盖较新接受，较新的拒绝也不抹除旧的有效接受。仍有 handler 在运行时，GetResult 与 GetResultForTask 快速返回 false，不等待 handler，也不改写输出。

GetResultForTask 在同一锁内检查非空任务身份、无 pending handler 以及终态，再填写结果。消费者应使用这个操作，避免分开的 GetSnapshot/GetResult 跨越抢占边界。

OnGoal 标准或非标准异常释放 pending 计数后传播。Shutdown 先关闭身份接受，再取消/清空状态；重初始化后旧票据仍失效。已经执行的域操作不能靠这个计数撤销。

## 域请求分类

ShouldUpdateGoalIdentity 是明确的分类钩子。默认保留无 header 请求的原兼容行为；导航域应区分 START/REPLAN 和 PAUSE/STOP。新任务需要更新身份，控制请求保留当前身份。允许匿名新任务的域显式返回 true，此时缺失 header 清除前一任务身份。

这个分类钩子必须有界、非阻塞；在生命周期锁内调用。测试域使用真实 NavigationGoal 类型验证命名 START、匿名 START 和控制请求；不能把测试域的分类自动当作真实 NavigationTask 已接入。

## 九类原生验证

1. 新目标拒绝后旧成功仍归属旧 ID，新 ID 查询失败。
2. 目标接受/拒绝等待期间，原生结果读取立即拒绝且输出未变，身份未提前发布。
3. 明确分类的匿名新任务不继承旧 ID。
4. 标准与非标准异常都释放结果读取屏障。
5. 并发晚旧接受不覆盖新接受；新拒绝不取消旧有效接受。
6. Shutdown 与重初始化窗口中的旧 handler 不能重新发布身份。
7. OnGoal 回调重入时较新接受保留身份。
8. OnGoal 内已经完成的新任务，其真实结果消息关联到新身份。
9. 接受控制请求不会把命令身份当成新任务身份。

专项使用真实 TypedTask 基类、Navigation protobuf、原生结果及快照 API；回调只控制域接受时序。条件变量等待有期限，不替换生命周期算法。隔离编译及真实库链接见 [隔离记录](validation/2026-11-18-v2.11-typed-stage.json)；正式同步和原生模板重编及相关 CTest 单独记录。

## 剩余执行责任

这项修复控制身份归属，不负责取消底盘运动。域 OnGoal 仍必须序列化或版本化执行状态，并保证接受确实代表新工作、合法重复请求或控制确认。真实 NavigationTask 的 START/REPLAN 分类、同位姿不同 ID、readiness 的可撤销等待、执行前许可与实际 ActionSession dispatch，以及父探索/委托导航调度均需继续接入与验证。

如果域把新 ID 当作旧位姿的重复请求，或返回 true 却留下旧成功状态，基类不能制造正确的新任务执行证据。此类问题应在真实域入口修复，不能从本专项通过推导整套导航协议已完成。
