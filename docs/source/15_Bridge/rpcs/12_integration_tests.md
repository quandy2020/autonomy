(rpc-integration-tests)=
# 集成测试

面向系统测试与集成开发的用例索引。契约：`automsgs.rpcs.*`（见 [02](02_service_overview.md)）。优先 `rpc-cli.py`；下表历史名（如 `SendNavigationCommand`）请映射为域服务方法（如 `NavigationService/Navigate`）。

### 13.1 如何跑测

前置：[01 §1.1](01_connection_guide.md#11-环境配置)。

```bash
export REPO=/path/to/autonomy
export BRIDGE=127.0.0.1:5005
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/system.proto"
export SVC=automsgs.rpcs.system.SystemService
```

| 步骤 | 命令 |
|------|------|
| 发现 | `grpcurl -plaintext $PROTO_OPTS $BRIDGE list` |
| 冒烟 | `GetCapabilities` / `GetActiveGoal` / `GetRobotFullInfo` |
| 急停 | `EmergencyStop` → `ClearEmergencyStop` |
| 取消 | `CancelAllGoals` |
| 导航等 | 见对应域 proto + `rpc-cli.py` |

```bash
# 一键冒烟（需 Bridge 已启动）
grpcurl -plaintext $PROTO_OPTS -d '{}' $BRIDGE $SVC/GetCapabilities
grpcurl -plaintext $PROTO_OPTS -d '{}' $BRIDGE $SVC/GetActiveGoal
grpcurl -plaintext $PROTO_OPTS -d '{"reason":"smoke"}' $BRIDGE $SVC/EmergencyStop
grpcurl -plaintext $PROTO_OPTS -d '{}' $BRIDGE $SVC/ClearEmergencyStop
```

Python：`pip install grpcio` + 已生成 stubs → `SystemServiceStub.GetCapabilities` 等；Bidi（Teleop）用 Python，勿依赖 grpcurl。

### 13.2 判读约定

| 类型 | 结束 | 成功 |
|------|------|------|
| Query / System Unary | 单条 JSON | 字段符合预期；System 看 `success` |
| Command Stream | 末帧 `ack.final=true` | 末帧 `ack.success=true` |
| Push Stream | Ctrl+C | 周期输出、字段非空 |

TaskType：导航=1 · 跟随=2 · 遥操=3 · 探索=4 · 对接=5 · 地图=6 
TaskStatus：IDLE=1 · RUNNING=2 · PAUSED=3 · SUCCEEDED=4 · FAILED=5 · CANCELED=6

### 13.3 用例索引

| ID | 场景 | 服务 / RPC | 判定要点 |
|----|------|------------|----------|
| TC-Q-001 | 能力探测 | `SystemService/GetCapabilities` | `bridgeVersion` 非空；待测 `supports*` |
| TC-Q-002 | 空闲快照 | `GetRobotSnapshot`（若有） | `activeTask` 空闲；`frameId` 多为 `map` |
| TC-Q-002b | 全信息 | `GetRobotFullInfo` | identity / state / capabilities |
| TC-Q-003 | 无活跃任务 | `GetActiveGoal` | type=NONE |
| TC-Q-004 | 导航中快照 | 同上 | type=NAVIGATION, RUNNING |
| TC-S-001 | 周期状态 | Sensor / 状态 Stream | 周期帧非空 |
| TC-S-002 | 生命周期事件 | 事件 Stream | 任务起止事件 |
| TC-S-003 | Snapshot 一致 | Snapshot + Stream | 同时刻误差 ≤1 帧 |
| TC-SYS-001 | 空闲急停 | `EmergencyStop` | `success` |
| TC-SYS-002 | 导航中急停 | Estop 抢占 | 导航末帧 CANCELED/FAILED |
| TC-SYS-003 | 取消全部 | `CancelAllGoals` | 无活跃 goal |
| TC-SYS-004 | 按类型取消 | 同上 + 类型过滤 | 仅目标类型取消 |
| TC-NAV-001 | 单点 START | `NavigationService/Navigate` | Stream 进度 → SUCCEEDED |
| TC-NAV-002 | 多点巡航 | Navigate + 多 goals | 依次到达 |
| TC-NAV-003 | PAUSE / RESUME | Pause / Resume | PAUSED → RUNNING |
| TC-NAV-004 | STOP | Cancel / Stop | 停止 |
| TC-NAV-005 | CANCEL | Cancel | 清理活跃任务 |
| TC-NAV-006 | REPLAN | Replan | 保持 goals |
| TC-NAV-NEG-001 | 缺 header | Navigate | INVALID_ARGUMENT |
| TC-NAV-NEG-002 | 重复 START | 二次 Navigate | 互斥拒绝 |
| TC-NAV-NEG-003 | 不可达目标 | Navigate | FAILED / NO_VALID_PATH |
| TC-EXP-001…004 | 探索 START / 暂停 / SAVE_MAP / SET_AREA | `ExplorationService` | 见探索页 |
| TC-FOL-001…004 | 跟随行人 / 位姿 / 更新 / 丢失 | `FollowService` | 见跟随页 |
| TC-TEL-001…003 | START / VELOCITY / STOP | `TeleopService` | VELOCITY 用 Python |
| TC-DCK-001…003 | 站点 / 位姿对接 / UNDOCK | `ChargeService` | 见对接页 |
| TC-MAP-001…005 | LOAD / SWITCH / 初位姿 / 清 costmap / 禁行区 | `MapService` | 见地图页 |
| E2E-001 | 导航闭环 | Caps → Navigate → 空闲 | 全程 SUCCEEDED |
| E2E-002 | 急停打断 | Navigate → Estop | CANCELED + 事件 |
| E2E-003 | 探索→存图→加载→导航 | Expl → Map → Nav | 新地图可导航 |
| E2E-004 | 重定位后导航 | SetInitialPose → Nav | pose 一致后规划 |

字段与示例载荷见各 Command 页（[07 导航](07_navigation_command.md) 等）及 [01 接入](01_connection_guide.md)。

### 13.4 参见

- [01 接入与验证](01_connection_guide.md)
- [02 服务概述](02_service_overview.md)
- [grpc/07 Handlers](../grpc/07_handlers.md)
