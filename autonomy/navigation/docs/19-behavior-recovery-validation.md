# 19 · 行为树恢复闭环与真实脱困验收

日期：2026-11-09。SSH `quandy@192.168.66.124`，容器 `SpaceHero`，
源码 `/workspace/autonomy/src/autonomy/autonomy/navigation`。
本轮推进 T-09/T-16，基础仿真与模型的历史记录见 [18](18-autosim-validation.md)。

## 运行契约

`BehaviorComponent` 读取 `behavior` 配置，用 `BtCppEngine` 注册内建节点、
写入 `node_context` 与 `max_recovery_attempts`，加载并执行默认
`behavior/trees/navigate_with_recovery.xml`。默认 DAG 中与 Navigation 同进程，
验收脚本将它拆成独立 mainboard 进程，以验证 IPC 与独立退出/重建。
验收机器人几何半径固定为 0.30 m；脚本保存匹配的独立 platform 配置，并让
Navigation/Safety 使用该快照。组件测试同样生成独立配置，避免部署配置变化
改变夹具的几何含义。本证据不直接认证其他 footprint 配置。
XML 不存在或注册失败时初始化失败，不会静默改用简化引擎。
`BtEngineImplementation` 保留为独立原语测试，不是生产 XML 的替代执行器。

目标仍由 NavigationComponent 接收 Action/topic 并交给 Runner。
`BehaviorSession::SetGoal` 只确认当前快照中的目标，避免 PipelineSequence 重复
tick `StartNavigation` 时清空已有轨迹。Behavior 不写 raw/final 速度。

Runner 跟踪每个 pose 目标的距离改善（至少 0.05 m）；进入位置容差后跟踪朝向
改善（至少 0.05 rad）。steady clock 上 8 s 无进度即进入 `RECOVERY_REQUIRED`，
清空动作块、排队任务与已完成结果，并使在途推理结果失效，输出零速等待行为决策。
探索目标没有 pose 时不使用这套距离检测。

`WaitForGoalReached` 在无进度/恢复失败时返回 FAILURE。`RecoveryNode` 将控制权
交给 `RecoveryExecutor`，后者请求一次、等待接受确认及 Runner 执行结果，
若 IPC 样本未送达则每 200 ms 重发，2 s 未确认视为失败。
恢复完成后重新执行导航分支；单次动作失败可切换另一侧，所有尝试共享 Runner 中
每个目标的计数，Behavior 重建不会获得新的次数预算。
`PipelineSequence` 在下游动作 RUNNING 时重新检查上游，失败时 halt 下游。
状态动作的首个 tick 现在正确锁存初始化，避免下一 tick 再次提交恢复。

## 消息与目标所有权

新增 `/navigation/behavior_request`，类型 `navigation.BehaviorRequest`：
`kind`、`session_id`、`goal_epoch`、`expected_recovery_count`、`reason`、
`steady_stamp_ns`。`RECOVER` 的预期次数为当前次数 +1，`FAIL_GOAL` 为当前次数。
Navigation 验证目标活跃、会话/目标版本相同、次数相符、发送时间不在未来且
年龄不超过 500 ms。重复接受、取消后的缓存、抢占前的旧目标以及进程重建前
的会话都会被拒绝。

`NavigationStatus` 新增上述会话/目标标识，以及 `has_active_goal`、
`recovery_state`、`recovery_count`、`pose_valid`、`steady_stamp_ns`。
Behavior 同时检查接收租约与发送时间；无新鲜状态/有效位姿时暂停树决策。
Navigation 在 odom 缺失、validity 无效或非有限位姿时持续发布零 raw，跳过 Runner
更新，避免默认原点推进恢复阶段或误报到达；组件回归覆盖 depth-only 零输出。
默认 Timer 为 100 ms；`tick_rate_hz` 控制树的最大频率，不能超过组件 Timer。

恢复速度仍为 `/navigation/cmd_vel_raw` → Safety → `/cmd_vel`。
Runner 在恢复期间检查 Behavior 心跳的 500 ms 租约；Behavior 退出后恢复动作
暂停，Safety 按自身限幅减速，重建可继续当前动作。Safety 的 depth/raw 200 ms
租约和 plant watchdog 继续生效。
取消或新目标立即清空恢复状态/次数，旧推理不能复活。
耗尽后 `RequestOperatorAssist` 锁存求助原因，通过终止请求使 Action 返回
`NAVIGATION_PATH_BLOCKED`，并清空目标/输出零速；没有额外的人工通知服务。

## 恢复动作与安全修复

`RecoveryManeuver` 使用 odom 驱动四个阶段：`face_flank` → `flank` →
`face_goal` → `advance`，然后恢复原始目标的策略查询。奇数尝试左侧、偶数右侧。
先原地转向，再沿可观察方向前进；不执行盲退或差速底盘无法执行的横向平移。
各阶段根据实测位姿完成，时间经过本身不能使动作成功。

默认侧向绕行与前向距离各 2 m，前向速度上限 0.30 m/s，角速度上限
0.60 rad/s；单阶段 25 s 上限与 8 s 无进度超时。距离/阶段超时可由配置调整，
全目标恢复次数默认 3；这里验证差速平台，不是 Ackermann 脱困方案。

首轮真实场景暴露了 CBF 的缓冲边界死锁：中心深度停在约 0.45 m，某些样本
低于边界几个微米，旧逻辑将所有速度约束为不可行，连原地转向也拒绝。
现在在实际圆形碰撞包络之外、但处于安全缓冲内时禁止继续靠近，允许零平移
转向/远离。实际包络重叠或动态障碍使约束不可行时仍由安全链停车。
回归同时验证缓冲内不许前进、允许转向/退离，以及实际重叠时不许转动。

## 已获得的真实证据

`/workspace/autonomy/data/navigation-bt-detour3`：保留真实 Habitat 障碍
`[1,-1,1.3,1]`，目标 `(3,0,0)`，一次恢复后完整 Action 成功，45.16625 s，
终点误差 0.06323 m，半径 0.3 m 的 footprint 最小几何余量 0.17623 m，
无几何穿透。低速检测后 7.57644 s 收到首次恢复反馈；8 s 门限从最后距离改善
计时，两个计时起点不同。轨迹与 `status_transitions.json` 覆盖四个执行阶段、
`RECOVERY_SUCCEEDED` 和目标完成。

日志 `/tmp/navigation-bt-detour3.log`。失败证据保留在 `navigation-bt-detour1/2`，
第二轮的 `blocked_depth.npy` 固化缓冲边界深度。没有移除障碍来满足脱困检查。

`/workspace/autonomy/data/navigation-bt-control2`：恢复转向期间终止独立 Behavior，
raw 在 0.48205 s 归零，plant 在 1.85259 s 低于 0.01 的停止阈值；重建恢复转向，
次数仍为 1。随后取消清空目标/恢复次数，给新目标发送旧 epoch、重新盖时间戳的
FAIL_GOAL 也被拒绝，新 Action 成功。该场景无几何穿透，最小余量 0.13565 m。

同一运行使用独立配置将预算设为 1，保留跨越整个房间的墙体
`[1,-3.9,1.3,4.9]`。33.61348 s 后返回 `NAVIGATION_PATH_BLOCKED`（101），
停止并清空目标，没有持续恢复循环；最小余量 0.12836 m，无几何穿透。
脚本随后恢复原始配置并重建本次进程，退出码 0，startup 加两项恢复控制检查通过。

`navigation-bt-control1` 保存初次暂停检查失败：raw 已为零，但约两秒时 plant
角速度仍为 0.01448，Safety 仍在受加速度/jerk 限制的减速过程。检查现在分别要求
raw 在 0.8 s 内归零、plant 在 3 s 内制动，记录两种耗时，不改变生产安全限幅。

完整 `colcon build --packages-up-to autonomy` 通过（36.08 s），五个组件与所有导航
测试实际编译链接。全量导航 CTest **76/76 通过，0 失败，21.97 s**；新增
`bt_cpp_engine_test`、`recovery_maneuver_test`，扩展 Runner 恢复/取消与 Safety 缓冲
回归。原组件集成和抢占 race 回归继续通过。构建仍有 Autolink unused parameter 等
warning，不宣称零 warning；日志 `/tmp/navigation-bt-full-colcon.log`、
`/tmp/navigation-bt-full-ctest.log`。

首次完整仿真运行 `/workspace/autonomy/data/navigation-bt-full1` **九项全部通过**，
退出码 0，没有 `failure.json`。日志 `/tmp/navigation-bt-full1.log`：

- 启动 validity=31，生产 bridge/plant 实际运动。
- 点 `(2,1,0)`：8.93040 s，误差 0.04727 m，无几何穿透。
- 静态障碍停车/移除恢复：停车 x=0.51688 m，最小余量 0.18312 m。
- depth 暂停及 epoch=0 重置：0.38513 s 实际停车，恢复后继续。
- Action/topic 抢占、取消与计时重置通过。
- 默认 XML 自主绕行：障碍全程保留，一次恢复，45.15782 s 到达，误差
  0.06642 m，最小余量 0.17677 m，无几何穿透。
- 恢复中 Behavior 退出/重建：raw 0.54259 s 归零，plant 2.17652 s 停止，次数
  保持 1；随后取消与新目标拒绝旧请求通过，最小余量 0.12801 m。
- 不可绕过墙体：预算 1，33.71445 s 返回 PATH_BLOCKED（101）并停车，最小余量
  0.17907 m，无几何穿透。
- Behavior/Navigation/Safety 进程退出、降级与重建通过；Navigation/Safety 退出后
  plant 分别在 0.46688/0.60845 s 停止。

`navigation-bt-full2` 保留后续配置漂移的失败证据：工作区的 differential_drive
半径变为 0.12 m，而场景 oracle 仍为 0.30 m，障碍前最小余量为 -0.00950 m。
组件夹具的障碍检查也失败。修正夹具使用独立 0.30 m platform 快照，现有部署
配置仍按其自身场景使用；不缩小 oracle 来消除这条失败。

修正位姿门控与配置快照后再次完整构建通过（51.913 s），最终导航 CTest
**76/76 通过，0 失败，21.92 s**。日志 `/tmp/navigation-bt-fixture-colcon.log`、
`/tmp/navigation-bt-fixture-ctest.log`。组件测试明确覆盖 depth-only 的持续零 raw，
并继续覆盖正常 odom 后的目标、取消/抢占、障碍停车、断流和进程守护。

最终候选运行 `/workspace/autonomy/data/navigation-bt-full3` **九项全部通过**，
退出码 0，无 failure.json，使用输出目录中固定半径 0.30 m 的 platform 快照。
日志 `/tmp/navigation-bt-full3.log`。自主绕行一次恢复后 45.28956 s 到达，
误差 0.05585 m，最小几何余量 0.17639 m，无几何穿透。
Behavior 退出后 raw/plant 分别在 0.48258/1.73914 s 停止，
重建次数仍为 1，取消与旧请求拒绝通过。墙体预算 1，34.52636 s 返回
PATH_BLOCKED（101）并停车，最小余量 0.17965 m。
点导航 8.93845 s、误差 0.04349 m；障碍停车余量
0.18329 m；depth 暂停 0.36405 s 停车；抢占检查通过。
Navigation/Safety 退出后 plant 分别在 0.41938/0.61538 s 停止。


安装布局补齐 `share/autonomy/navigation/behavior/trees`，保留原 `navigation/trees`
供旧资源使用方兼容。这样 `NAVIGATION_CONF_DIR` 指向安装的 navigation 根目录时，
默认执行器与源代码使用同一 `behavior/trees` 路径。`bt_cpp_engine_test` 支持传入资源
根目录，复用实际 XML/恢复/预算测试验证安装后的树；安装目录验证已通过，日志
`/tmp/navigation-bt-installed-tree.log`。逐资源 SHA256 核对通过：19 个配置/模型资源、
3 DAG、4 launch、3 个默认布局 XML 与 3 个兼容布局 XML、20 篇文档及 README
均与源码一致，五个组件库已安装；日志 `/tmp/navigation-bt-install-check.log`。

## 复现

```sh
cd /workspace/autonomy
colcon build --packages-up-to autonomy
ctest --test-dir build/autonomy/autonomy/navigation --output-on-failure --no-tests=error
bash src/autonomy/autonomy/navigation/tools/run_autosim_acceptance.sh \
  /workspace/autonomy/data/navigation-bt-new
```

只跑恢复时追加 `--checks recovery,recovery_control,recovery_exhaustion`。
输出目录必须使用新名字。脚本保存配置/DAG 快照、各进程日志、Action feedback、
状态转移与 `trajectory.jsonl`；进程退出只清理本次创建的子进程。

## 适用边界与后续

Robot 采用运动学 set_pose；每个 plant step 的独立几何检查不等于刚体物理接触
响应或真实底盘认证。Safety 仍采样中心行 5 列，不是完整点云/全向感知。
本实现是参数化局部绕行，不会构建全局路径；复杂迷宫、动态障碍、不同大小
障碍与长期稳定性仍需新场景。失败终态用于限制恢复循环。

`navigate_with_door.xml` 和 `navigate_with_relocalization.xml` 保留为任务示例，
其完整组件编排没有随默认恢复树一同验收；不把 T-17 标成完成。
T-10 长期探索/map frontier、T-14/15 planner 权重、ASan/TSan、CUDA/TensorRT、
目标硬件时延仍未在本轮验证。下一步优先推进 T-10，详见 [13](13-todo.md)。
