# 01 需求

把一次业务失败从「一段不可检索的日志」变成「可归档、可追查、可复现、可训练、可回灌」的闭环对象。机上只旁路；云上只消费证据，不远程改写控制图。

## 目标

| 编号 | 陈述 |
|------|------|
| G1 | 不改规划 / 控制 / 感知的前提下，旁路捕获通道字节与双时钟戳，采集永不背压 `/cmd_vel`。 |
| G2 | 每个 episode 绑定确定性版本四元组；缺字段进诊断队列，禁止训练、正式回放与模型晋升。 |
| G3 | 默认事件触发落盘；测试全量用 `--full-dump` 会话落盘。边侧按 P0–P3 决定上传。 |
| G4 | 任意失败现场（日志一行、仪表一个点）能一次跳转到同一 `episode_id` 的 MCAP + sidecar + 四元组。 |
| G5 | 家居导航 / 定位 / 任务失败给出可引用的根因卡片（窗、通道、BT 节点）；L2 只承诺时间共现。 |
| G6 | 同一四元组下可 SIL / HIL / 回归；回放不得当作在线控制。 |
| G7 | 分诊 UI、机队仪表、治理卡片以同一 `episode_id` / `mono_ns` 游标对齐。 |
| G8 | 门禁通过后模型在任务间隙原子出现在机上 `current`，下一 episode 的 `policy_version` 能证明「跑的就是这一版」。 |

## 非目标

| 项 | 说明 |
|----|------|
| 修改 L0 控制图 | 不在 policy 与执行器之间插入 `/cmd_vel` 转发节点。 |
| 远程刹停 | 云端不发布速度指令、不代替 `autonomy.monitor` MRM。 |
| PTP / gPTP 硬件时钟 | 默认 software 双时钟；硬件时钟预留给 recorder proto。 |
| 采集进程调度训练 | `cloud/training/` 是离线入口，不在 `OnSample` 热路径。 |
| 任务中途热切模型 | `model_sync` 仅在 IDLE / 任务间隙切换。 |
| 可视化写控制 | autoviz / dashboard 只读 MCAP + sidecar + 指标。 |
| 替代 `autonomy` 导航栈 | Autoloop 树内 `drivers` / `localization` / `navigation` / `perception` 是 loop-runtime 接口。 |

## 约束

1. **编译隔离**：`-DAUTOLOOP_BUILD_RECORDER`（默认 ON）控制 `autoloop.recorder`；`AUTOLOOP_LATCH_ONLY=1` 由 `cmake/SafetyChecks.cmake` 注入。
2. **进程隔离**：采集 / 闩锁 / 控制分进程。上传失败只积压队列，不阻塞 `OnSample`。
3. **不背压**：订阅 QoS 为 BEST_EFFORT + KEEP_LAST；热路径只拷贝预序列化字节。
4. **容器与载荷**：MCAP 为容器，Protobuf 为载荷；通道字节不进 JSONL 事件。
5. **安全单向**：L4 只能使机器更安全（通过 / 闩锁回退 / 记录移交），不得发布速度指令。
6. **身份完整**：训练样本必须能从 LeRobot 行反查 `robot_id / episode_id / content_sha256`。

## 能力面

| 能力 | 要回答的问题 | 落点 |
|------|----------------|------|
| 采集 | 失败窗口是否落盘且不背压控制？ | recorder L1 + `data_agent` ring |
| 可回溯 | 这次跑的是哪版 policy / 地图 / git / seed？ | `VersionTuple` + `trace_id` |
| 可追查 | 根因落在定位、规划、执行还是接管？ | sidecar 事件图 / BT / 诊断队列 |
| 可复现 | SIL / HIL / 回归能否按同一四元组重放？ | recorder L6 + `cloud/simulation` |
| 可可视化 | 时间轴、因果、仪表能否对齐同一 episode？ | autoviz Evidence + dashboards |
| 可治理 | 成败如何分流、未准入如何隔离？ | success / failure / diagnostics 池 |
| 可回灌 | 新模型如何原子切换且可回滚？ | PublishGate + `model_sync` |
| 安全单向 | 采集 / 闩锁是否可能改写速度指令？ | `SafetyLatch` / `safety_guard` |

## 触发与优先级

| 规则 | 条件 | 优先级 | 闭环去向 |
|------|------|--------|----------|
| estop | 安全闩置位 | P0 | 失败池 + 安全链，立即上传 |
| task_fail | TaskResult / `*/feedback` FAILED | P0 | 失败池，根因卡片 |
| takeover | 遥控 / teleop 上升沿 | P0 / P1 | 上传，默认不进专家模仿 |
| loc_degrade | 定位质量低于阈值（边沿） | P1 | 可进回归 |
| path_deviate | odom vs plan | P2 | 可选人审 |
| manual | `trigger_snapshot.sh` | P2 | 测试 / 抽检 |
| periodic / 成功 | 心跳或任务 SUCCESS | P3 | 分层抽样 |

P0 立即上传；P1 上传但不当专家演示；P2 / P3 按抽样率。四元组不齐则无论优先级均写入诊断通道，可视化仍可打开，训练拒绝。测试全量：`--full-dump` 会话落盘，不靠触发截断。细则：[12 触发](12_triggers.md)。实现缺口：[15 TODO](15_todo.md)。
