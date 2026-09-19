# 09 · `autonomy.bridge` 命令行手册与全业务测试用例

本文是 **`autonomy.bridge` CLI** 的权威说明：服务端启动参数 + 内置 RPC 客户端（`list` / `describe` / `call`），覆盖全部 `automsgs.rpcs.*` 业务的可复制测试命令，并写明**每条命令成功/失败时终端应出现的消息**（见 §2.5 与各节「终端期望」）。

相关：[`../README.md`](../README.md) · [`08_testing.md`](08_testing.md) · 字段级 proto 以 `automsgs/proto/rpcs/*.proto` 为准。

---

## 0. 快速索引

| 模式 | 命令 | 是否需要 Bridge 已启动 |
|------|------|------------------------|
| 帮助 / 版本 | `autonomy.bridge -h` / `-V` | 否 |
| 起服务 | `autonomy.bridge [-c conf]` | —（本进程即服务） |
| 配置自检 | `autonomy.bridge -n` / `-t` | 否 |
| 列方法 | `autonomy.bridge list [Service]` | 否 |
| 看定义 | `autonomy.bridge describe <符号>` | 否 |
| 调 RPC | `autonomy.bridge call <Method> -d JSON` | **是**（对端已 listen） |

默认客户端目标：`127.0.0.1:5005`（可用 `--target` / `--host`+`--port` 覆盖）。

---

## 1. 环境准备

```bash
# 将二进制加入 PATH（按本机 build 路径调整）
export PATH=$PWD/build/bin:$PATH

# 终端别名（下文一律用 BR）
export BR=autonomy.bridge
export TGT=${BRIDGE:-127.0.0.1:5005}
alias brc='$BR call --target $TGT'
```

（推荐先 `source scripts/setup.bash`，已设置 `PATH`、`AUTONOMY_PATH`、`BRIDGE`。）

**推荐双终端：**

```bash
# T1 — 起服务（开发机可开 reflection）
$BR -c bridge.pb.txt

# T2 — 客户端
$BR list
brc SystemService/Heartbeat -d '{"sequence":1}'
```

成功判据（业务层）：响应 JSON 中 `status.code` 为 `"OK"`（或数值 `0`）。  
传输失败时 CLI **stderr** 打印 `RPC failed: … (code=…)`，进程退出码非 0。

如何解读每条命令的终端输出：见 **§2.5**；各业务用例下的 **「终端期望」** 块给出示例。

---

## 2. 全局约定

### 2.1 Method 写法

| 写法 | 示例 | 说明 |
|------|------|------|
| `Service/Method` | `SystemService/Heartbeat` | **推荐** |
| 全名 | `automsgs.rpcs.system.SystemService/Heartbeat` | 最稳 |
| 仅 Method | `Heartbeat` | 仅当全局唯一；`GetStatus`/`Cancel`/`Pause` **必须**带 Service |

### 2.2 请求体 `-d`

| 形式 | 示例 |
|------|------|
| 内联 JSON | `-d '{"sequence":1}'` |
| 空对象 | `-d '{}'` |
| 文件 | `-d @/tmp/nav.json` |

常用嵌套（导航 / 语音等）：

```json
"header": { "stamp": { "sec": 0, "nanosec": 0 }, "frame_id": "map" }
"pose": {
  "position": { "x": 1.0, "y": 2.0, "z": 0.0 },
  "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
}
```

`PoseStamped`：

```json
{
  "header": { "frame_id": "map" },
  "pose": {
    "position": { "x": 1.0, "y": 0.0, "z": 0.0 },
    "orientation": { "w": 1.0 }
  }
}
```

### 2.3 元数据与鉴权

```bash
brc SystemService/Heartbeat -d '{"sequence":1}' \
  --bearer <token> \
  --robot-id robot-1 \
  -H 'x-request-id:smoke-001'
```

| 选项 | 作用 |
|------|------|
| `--bearer TOKEN` | `authorization: Bearer TOKEN` |
| `--robot-id ID` | `x-robot-id` |
| `-H key:value` | 任意 metadata（可重复） |
| `--timeout SEC` | deadline（默认 30；流式可加大） |
| `--tls` | TLS（默认 plaintext） |
| `-v` / `--verbose` | 打印 method path |

与 conf 对齐：`auth_mode=AUTH_MODE_BEARER_TOKEN`、`require_robot_id_metadata=true` 时必须带对应头，否则 gRPC `UNAUTHENTICATED` / `FAILED_PRECONDITION`（见 [02_policy.md](02_policy.md)）。

### 2.4 能力边界

| RPC 形态 | `autonomy.bridge call` |
|----------|------------------------|
| unary | ✅ |
| server-streaming | ✅（逐帧打印 JSON） |
| client-streaming / bidi（如 `TeleopService/Velocity`） | ❌ → 用 `grpcurl` 或 [`automsgs/tools/cli/rpc-cli.py`](../../../automsgs/tools/cli/README.md) |

### 2.5 如何读终端输出

`call` **成功时只向 stdout 打印响应 JSON**（pretty-print）；失败信息在 **stderr**，退出码 ≠ 0。

#### Unary 成功（典型）

```text
{
 "status": {
  "code": "OK"
 },
 "sequence": "1",
 "robot_time_ns": "1735689600123456789"
}
```

要点：

| 项 | 说明 |
|----|------|
| `status.code` | 成功多为字符串 `"OK"`（protobuf JSON 默认枚举名）；偶见数字 `0` |
| `status.message` | 成功时常省略或为空 |
| `uint64` / `int64` | JSON 里常为**字符串**（如 `"1"`、`"1735…"`），属正常 |
| 退出码 | `0`；无额外 “OK” 横幅 |

加 `-v` 时 **stderr** 多一行：

```text
+ /automsgs.rpcs.system.SystemService/Heartbeat @ 127.0.0.1:5005 [unary]
```

#### Server-streaming 成功

每一帧一条完整 JSON 对象，连续打印到 stdout，例如：

```text
{
 "status": { "code": "OK" },
 "state": "NAVIGATION_STATE_RUNNING",
 "goal_id": "nav-cli-001",
 ...
}
{
 "status": { "code": "OK" },
 "state": "NAVIGATION_STATE_ARRIVED",
 "goal_id": "nav-cli-001",
 ...
}
```

加 `-v` 结束时 stderr：

```text
# received N frames
```

#### 传输 / 拦截器失败（stderr，无业务 JSON 或仅部分）

```text
RPC failed: missing or invalid bearer token (code=16)
```

| 文案（示例） | gRPC code | 含义 |
|--------------|-----------|------|
| `missing or invalid bearer token` | 16 UNAUTHENTICATED | Bearer 错/缺 |
| `missing x-robot-id` | 9 FAILED_PRECONDITION | 缺强制 robot-id |
| `rate limit exceeded` | 8 RESOURCE_EXHAUSTED | 限流 |
| `Connection refused` / `failed to connect` | — | 服务未起或 target 错 |
| `Deadline Exceeded` | 4 | `--timeout` 过短或下游卡住 |
| `client/bidi streaming not supported` | — | 如 Velocity，换工具 |

#### 业务失败但仍返回响应体

部分 Handler 在 **gRPC OK** 下返回 `status.code != OK`（如 `CANCELLED`、`INVALID_ARGUMENT`）。此时 stdout 仍有 JSON，例如：

```text
{
 "status": {
  "code": "CANCELLED",
  "message": "cancelled"
 }
}
```

判读：先看进程退出码与是否 `RPC failed:`；再看 JSON 里的 `status.code`。

---

## 3. 服务端子命令与选项

```bash
$BR -h
$BR -V

# 指定 conf、覆盖监听
$BR -c bridge.pb.txt --host 0.0.0.0 --port 5005

# 只加载 conf 打印摘要
$BR -n -c bridge.pb.txt

# ApplyPlatform 自检（不 listen）
$BR -t --print-config

# dump 完整 BridgeOptions 文本
$BR -n --print-config
```

| 选项 | 含义 |
|------|------|
| `-c, --conf` | protobuf text（默认 `bridge.pb.txt`） |
| `-n, --dry-run` | 加载 conf → 摘要 → 退出 |
| `-t, --self-test` | `ApplyPlatform` 自检 → 退出 |
| `--print-config` | 打印 `BridgeOptions` DebugString |
| `--host` / `--port` | 覆盖 `grpc.host` / `grpc.port` |

平台字段（health / reflection / auth / rate_limit …）见 [06_configuration.md](06_configuration.md)。

**终端期望**

`-V`（示例，具体版本串随构建变化）：

```text
autonomy.bridge
  Autonomy x.y.z ...
  build: ...
  commit: abcdef0
```

`-n` dry-run：

```text
listen=127.0.0.1:5005 grpc_threads=5 event_threads=5 worker_threads=0 health=on reflection=off metadata_interceptor=on auth=NONE ssl=off rate_limit=off otel=off
```

（同时日志里可能有 `Bridge options: …` / `Dry-run: conf loaded OK`。）

`-t` self-test 成功：

```text
self-test: OK
channel_args=ok credentials=ok health=on reflection=off reflection_lib=yes|no interceptors={logging=1 auth=0 metadata=1 rate_limit=0 otel=0}
```

正常起服务（前台）：

```text
# 日志（非固定文案，量级如下）
ApplyPlatform: health=... reflection=...
gRPC bridge configured to listen on 127.0.0.1:5005 ...
Bridge server running. Press Ctrl+C to exit.
```

Ctrl+C 后：

```text
Shutdown autonomy bridge.
```

---

## 4. 客户端：`list` / `describe`

```bash
# 全部 Service + Method（标注 unary / server-streaming / …）
$BR list

$BR list SystemService
$BR list NavigationService
$BR list TeleopService

$BR describe SystemService
$BR describe SystemService/Heartbeat
$BR describe NavigationService/Navigate
$BR describe automsgs.rpcs.navigation.NavigateRequest
```

**终端期望 — `list`（节选）**

```text
automsgs.rpcs.system.SystemService
  /automsgs.rpcs.system.SystemService/Heartbeat  [unary]
  /automsgs.rpcs.system.SystemService/GetInfo  [unary]
  ...
automsgs.rpcs.navigation.NavigationService
  /automsgs.rpcs.navigation.NavigationService/Navigate  [server-streaming]
  ...
automsgs.rpcs.teleop.TeleopService
  /automsgs.rpcs.teleop.TeleopService/Velocity  [bidi-streaming]
  /automsgs.rpcs.teleop.TeleopService/DriveOnHeading  [server-streaming]
  ...

# 10 services, NN methods
```

**终端期望 — `describe SystemService/Heartbeat`**

```text
rpc automsgs.rpcs.system.SystemService.Heartbeat
  type:     unary
  request:  automsgs.rpcs.system.HeartbeatRequest
  response: automsgs.rpcs.system.HeartbeatResponse
  path:     /automsgs.rpcs.system.SystemService/Heartbeat

// request fields
  uint64 sequence = 1
```

**终端期望 — `describe` 未知符号**（stderr，退出码 ≠ 0）

```text
unknown symbol: FooBar
```

（其后可能附带解析细节行。）

---

## 5. 全业务测试用例

下列命令均假设：

```bash
export BR=autonomy.bridge
export TGT=127.0.0.1:5005
alias brc='$BR call --target $TGT'
```

建议顺序：**System 查询 → Localization → Map 查询 → Sensor 列表 → 命令流（Navigate/Follow/…）→ Teleop relative → Voice → Estop/CancelAll**。  
命令流会占 `TaskMuxer` 互斥槽；测完请 `Cancel` / `CancelAllGoals`。  
每条用例下的 **「终端期望」** 给出成功/失败时 stdout / stderr 应出现的消息（示例字段可随现场略变，但 `status.code` / 错误文案形态应对齐）。

---

### 5.1 SystemService — `automsgs.rpcs.system.SystemService`

| Method | 形态 | 说明 |
|--------|------|------|
| Heartbeat | unary | 保活 |
| GetInfo | unary | 身份 / 版本 |
| GetStatus | unary | 总状态 + 电量 |
| GetHealth | unary | 监控健康 |
| GetRobotFullInfo | unary | 画像聚合 |
| EmergencyStop | unary | 急停闩锁 |
| ClearEmergencyStop | unary | 清急停 |
| CancelAllGoals | unary | 取消活动目标 |
| GetActiveGoal | unary | 当前目标 |
| GetCapabilities | unary | 能力广告 |

#### 5.1.1 Heartbeat

```bash
brc SystemService/Heartbeat -d '{"sequence":1}'
brc SystemService/Heartbeat -d '{"sequence":2}' -v
```

**终端期望（stdout）**

```text
{
 "status": {
  "code": "OK"
 },
 "sequence": "1",
 "robot_time_ns": "1735689600123456789"
}
```

| 字段 | 期望 |
|------|------|
| `status.code` | `"OK"` |
| `sequence` | 与请求一致（JSON 常为字符串 `"1"`） |
| `robot_time_ns` | 非空纳秒时间戳字符串 |
| 退出码 | `0` |

`-v` 时 stderr 另有：`+ /automsgs.rpcs.system.SystemService/Heartbeat @ … [unary]`。

#### 5.1.2 GetInfo / GetStatus / GetHealth / GetRobotFullInfo / GetCapabilities / GetActiveGoal

```bash
brc SystemService/GetInfo -d '{}'
brc SystemService/GetStatus -d '{}'
brc SystemService/GetHealth -d '{}'
brc SystemService/GetRobotFullInfo -d '{}'
brc SystemService/GetCapabilities -d '{}'
brc SystemService/GetActiveGoal -d '{}'
```

**终端期望（各命令 stdout 要点）**

`GetInfo`：

```text
{
 "status": { "code": "OK" },
 "model": "...",
 "serial_number": "...",
 "software_version": "...",
 "hostname": "...",
 "autonomy_version": "..."
}
```

（具体字符串来自 `BridgeOptions.identity` / 运行时填充；空串也可接受，但应有 `status.code=OK`。）

`GetStatus`（空闲）：

```text
{
 "status": { "code": "OK" },
 "state": "SYSTEM_STATE_IDLE",
 "detail": "",
 "battery": { ... }   // 可能缺省，视 StateHub 是否有电量
}
```

| `state` | 含义 |
|---------|------|
| `SYSTEM_STATE_IDLE` | 无活动任务 |
| `SYSTEM_STATE_BUSY` | Muxer 有活动槽 |
| `SYSTEM_STATE_ESTOP` | 急停闩锁 |

`GetHealth`：含 `hazard_level`、`emergency_stop_latched`、`host` / `channels` 等（监控未就绪时部分字段可空，但外层 `status.code=OK`）。

`GetRobotFullInfo`：大体量 JSON，应含 `identity`、`system_state`、`capabilities`、`robot_time_ns`；`status.code=OK`。

`GetCapabilities`：布尔能力开关集合；`status.code=OK`。

`GetActiveGoal`（无活动目标时）：

```text
{
 "status": { "code": "OK" },
 "kind": "GOAL_KIND_NONE",
 "goal_id": "",
 ...
}
```

或 `kind` 为当前任务类型且 `goal_id` 非空。

#### 5.1.3 CancelAllGoals

```bash
# 取消全部
brc SystemService/CancelAllGoals -d '{"reason":"cli-smoke"}'

# 仅取消导航类
brc SystemService/CancelAllGoals -d '{
  "goal_kinds": ["GOAL_KIND_NAVIGATION"],
  "reason":"cancel-nav-only"
}'
```

**终端期望**

```text
{
 "status": {
  "code": "OK"
 }
}
```

（`common.Status`；无活动目标时通常仍为 OK。）

#### 5.1.4 EmergencyStop / ClearEmergencyStop

```bash
brc SystemService/EmergencyStop -d '{"reason":"cli-estop-test"}'
brc SystemService/GetStatus -d '{}'
brc SystemService/ClearEmergencyStop -d '{
  "reason":"cli-clear",
  "confirmation_token":""
}'
```

**终端期望**

`EmergencyStop` / `ClearEmergencyStop`：

```text
{
 "status": { "code": "OK" }
}
```

紧接的 `GetStatus` 在急停后：

```text
{
 "status": { "code": "OK" },
 "state": "SYSTEM_STATE_ESTOP",
 ...
}
```

清急停后再 `GetStatus`，`state` 回到 `IDLE` 或 `BUSY`。

> 生产环境 `confirmation_token` 可能非空；以现场 conf / 实现为准。

#### 5.1.5 平台强制（鉴权 / 元数据）

```bash
# 无 token（auth=BEARER 时应失败）
brc SystemService/Heartbeat -d '{"sequence":1}'

# 正确 token
brc SystemService/Heartbeat -d '{"sequence":1}' --bearer "$BRIDGE_TOKEN"

# require_robot_id=true 时缺头失败
brc SystemService/Heartbeat -d '{"sequence":1}' --bearer "$BRIDGE_TOKEN"
brc SystemService/Heartbeat -d '{"sequence":1}' \
  --bearer "$BRIDGE_TOKEN" --robot-id robot-1
```

**终端期望**

| 场景 | stderr（示例） | 退出码 |
|------|----------------|--------|
| Bearer 错/缺 | `RPC failed: missing or invalid bearer token (code=16)` | ≠0 |
| 缺 `x-robot-id` | `RPC failed: missing x-robot-id (code=9)` | ≠0 |
| 头齐全 | 同 §5.1.1 成功 JSON | 0 |

（仅当 conf 打开对应强制时；`AUTH_MODE_NONE` 且不要求 robot-id 时，无头也会成功。）

---

### 5.2 NavigationService — `automsgs.rpcs.navigation.NavigationService`

| Method | 形态 |
|--------|------|
| Navigate | server-streaming |
| Pause / Resume / Replan / Cancel | unary |
| GetStatus | unary |

#### 5.2.1 Navigate（单点）

```bash
brc SystemService/CancelAllGoals -d '{"reason":"before-nav"}'

brc NavigationService/Navigate -d '{
  "header": { "frame_id": "map" },
  "goal_id": "nav-cli-001",
  "waypoints": [{
    "header": { "frame_id": "map" },
    "pose": {
      "position": { "x": 1.0, "y": 0.0, "z": 0.0 },
      "orientation": { "w": 1.0 }
    }
  }],
  "options": {
    "maximum_linear_speed": 0.5,
    "timeout_seconds": 120
  }
}' --timeout 180
```

**终端期望（server-streaming，多帧 stdout）**

第 1 帧（接受 ACK，状态因实现而异，常见 RUNNING / 自定义）：

```text
{
 "status": { "code": "OK" },
 "state": "NAVIGATION_STATE_RUNNING",
 "goal_id": "nav-cli-001",
 "waypoint_index": 0,
 "number_of_waypoints": 1
}
```

中间帧：可能更新 `current_pose`、`remaining_distance_meters` 等。

终态帧示例：

```text
{
 "status": { "code": "OK" },
 "state": "NAVIGATION_STATE_ARRIVED",
 "goal_id": "nav-cli-001",
 ...
}
```

或失败：`NAVIGATION_STATE_FAILED` / `CANCELLED`，`status.code` 可能非 OK。  
全程每帧 `goal_id` 应为 `nav-cli-001`。`-v` 结束 stderr：`# received N frames`（N≥1）。

无 Task / Autolink 时：可能仅 1 帧失败，或 stderr `RPC failed: …`。

#### 5.2.2 Navigate（多点）

```bash
brc NavigationService/Navigate -d '{
  "goal_id": "nav-cli-multi",
  "waypoints": [
    { "header": { "frame_id": "map" },
      "pose": { "position": { "x": 1.0, "y": 0.0 }, "orientation": { "w": 1.0 } } },
    { "header": { "frame_id": "map" },
      "pose": { "position": { "x": 2.0, "y": 1.0 }, "orientation": { "w": 1.0 } } }
  ]
}' --timeout 300
```

#### 5.2.3 Pause / Resume / Replan / Cancel / GetStatus

```bash
brc NavigationService/GetStatus -d '{}'

brc NavigationService/Pause -d '{"goal_id":"nav-cli-001"}'
brc NavigationService/Resume -d '{"goal_id":"nav-cli-001"}'
brc NavigationService/Replan -d '{"goal_id":"nav-cli-001"}'
brc NavigationService/Cancel -d '{"goal_id":"nav-cli-001"}'

# 空 goal_id = 当前目标
brc NavigationService/Cancel -d '{}'
```

**终端期望**

`GetStatus`（无导航时）：

```text
{
 "status": { "code": "OK" },
 "state": "NAVIGATION_STATE_IDLE",
 "goal_id": "",
 ...
}
```

`Pause` / `Resume` / `Replan` / `Cancel` 成功：

```text
{
 "status": { "code": "OK", "message": "..." }
}
```

（`message` 可能为 `paused` / `resumed` / `cancelled` 等实现文案。）无匹配 `goal_id` 时可能 `status.code` 非 OK 或仍 OK——以现场为准。

---

### 5.3 FollowService — `automsgs.rpcs.follow.FollowService`

| Method | 形态 |
|--------|------|
| Follow | server-streaming |
| Pause / Resume / Cancel | unary |
| GetStatus | unary |

`FollowTargetType`：`FOLLOW_TARGET_TYPE_PERSON` 等（见 proto）。

```bash
brc FollowService/Follow -d '{
  "header": { "frame_id": "base_link" },
  "goal_id": "follow-cli-001",
  "target_type": "FOLLOW_TARGET_TYPE_PERSON",
  "target_id": "",
  "options": { "desired_distance_m": 1.2 }
}' --timeout 120

brc FollowService/GetStatus -d '{}'
brc FollowService/Pause -d '{"goal_id":"follow-cli-001"}'
brc FollowService/Resume -d '{"goal_id":"follow-cli-001"}'
brc FollowService/Cancel -d '{"goal_id":"follow-cli-001"}'
```

**终端期望**

`Follow` 流帧示例：

```text
{
 "status": { "code": "OK" },
 "state": "FOLLOW_STATE_ACQUIRING",
 "goal_id": "follow-cli-001",
 "target_type": "FOLLOW_TARGET_TYPE_PERSON",
 "active": true,
 "message": ""
}
```

后续可能变为 `FOLLOWING` / `LOST_TARGET` / `CANCELLED`。  
`GetStatus`：单帧快照，字段同响应。  
`Pause`/`Resume`/`Cancel`：同 Navigation，`status.code=OK` 的 `common.Status` JSON。

---

### 5.4 ChargeService — `automsgs.rpcs.charge.ChargeService`

| Method | 形态 |
|--------|------|
| Return / Leave | server-streaming |
| Pause / Resume / Cancel | unary |
| GetStatus | unary |

```bash
# 回充（空 station_id = 默认桩）
brc ChargeService/Return -d '{
  "header": { "frame_id": "map" },
  "station_id": "",
  "goal_id": "charge-return-001"
}' --timeout 300

brc ChargeService/GetStatus -d '{}'
brc ChargeService/Pause -d '{"goal_id":"charge-return-001"}'
brc ChargeService/Resume -d '{"goal_id":"charge-return-001"}'
brc ChargeService/Cancel -d '{"goal_id":"charge-return-001"}'

# 离桩
brc ChargeService/Leave -d '{
  "header": { "frame_id": "map" },
  "goal_id": "charge-leave-001"
}' --timeout 180
```

**终端期望**

`Return` / `Leave` 流帧示例：

```text
{
 "status": { "code": "OK" },
 "state": "CHARGE_STATE_NAVIGATING",
 "goal_id": "charge-return-001",
 "progress": 0.1,
 "battery_pct": 80,
 "active": true,
 "message": ""
}
```

终态常见：`CHARGE_STATE_CHARGING` / `DOCKED` / `SUCCEEDED` / `FAILED` / `CANCELLED`（以 proto 枚举为准）。  
`GetStatus`：单帧；lifecycle unary：`{ "status": { "code": "OK" } }`。

---

### 5.5 TeleopService — `automsgs.rpcs.teleop.TeleopService`

| Method | 形态 | CLI |
|--------|------|-----|
| Velocity | **bidi** | ❌ 见 §5.5.4 |
| DriveOnHeading / BackUp / Spin | server-streaming | ✅ |
| Pause / Resume / Cancel / GetStatus | unary | ✅ |

#### 5.5.1 DriveOnHeading

```bash
brc TeleopService/DriveOnHeading -d '{
  "header": { "frame_id": "base_link" },
  "goal_id": "teleop-drive-001",
  "distance_meters": 0.3,
  "options": {
    "maximum_linear_speed": 0.2,
    "timeout_seconds": 30
  }
}' --timeout 60
```

**终端期望（DriveOnHeading / BackUp / Spin 流）**

```text
{
 "status": { "code": "OK" },
 "state": "TELEOP_STATE_RUNNING",
 "goal_id": "teleop-drive-001",
 "detail": "",
 "remaining_distance_meters": 0.25,
 ...
}
```

终态：`TELEOP_STATE_SUCCEEDED` / `FAILED` / `CANCELLED` 等。  
`GetStatus`：单帧快照。  
`Cancel`：`{ "status": { "code": "OK", "message": "..." } }`。

#### 5.5.2 BackUp

```bash
brc TeleopService/BackUp -d '{
  "goal_id": "teleop-backup-001",
  "distance_meters": 0.2,
  "options": { "maximum_linear_speed": 0.15 }
}' --timeout 60
```

#### 5.5.3 Spin

```bash
brc TeleopService/Spin -d '{
  "goal_id": "teleop-spin-001",
  "target_yaw_radians": 0.5,
  "options": { "maximum_angular_speed": 0.4 }
}' --timeout 60

brc TeleopService/GetStatus -d '{}'
brc TeleopService/Cancel -d '{"goal_id":"teleop-spin-001"}'
```

**终端期望**：同 §5.5.1；Spin 帧可能带 `remaining_yaw_radians`。

#### 5.5.4 Velocity（bidi，需 grpcurl / rpc-cli）

```bash
# 示例：grpcurl（需 reflection 或 proto import）
# 连续发送 START → TWIST → STOP；此处仅作提示
grpcurl -plaintext -d @velocity_session.json \
  $TGT automsgs.rpcs.teleop.TeleopService/Velocity
```

或：

```bash
cd automsgs/tools/cli
./rpc-cli.py call TeleopService/Velocity -d @velocity_session.json -t $TGT
```

**`autonomy.bridge call` 终端期望（应失败）**

```text
client/bidi streaming not supported in autonomy.bridge call (bidi-streaming). Use grpcurl or rpc-cli.py for /automsgs.rpcs.teleop.TeleopService/Velocity
```

退出码 ≠ 0。

---

### 5.6 MapService — `automsgs.rpcs.mapping.MapService`

| Method | 形态 |
|--------|------|
| StartMapping / FinishMapping / CancelMapping | unary |
| GetMappingStatus | unary |
| ListMaps / GetMap / GetMapMetadata | unary |
| SaveMap / DeleteMap / SetCurrentMap | unary |

```bash
brc MapService/ListMaps -d '{}'
brc MapService/GetMappingStatus -d '{}'

brc MapService/StartMapping -d '{
  "header": { "frame_id": "map" },
  "map_name": "cli_map_tmp",
  "goal_id": "map-start-001"
}'

brc MapService/GetMappingStatus -d '{}'

brc MapService/FinishMapping -d '{
  "goal_id": "map-start-001",
  "persist": true
}'

brc MapService/GetMapMetadata -d '{ "map_name": "cli_map_tmp" }'
brc MapService/GetMap -d '{ "map_name": "cli_map_tmp" }' --timeout 120

brc MapService/SaveMap -d '{ "map_name": "cli_map_tmp" }'
brc MapService/SetCurrentMap -d '{ "map_name": "cli_map_tmp" }'

# 危险：删除测试图
# brc MapService/DeleteMap -d '{ "map_name": "cli_map_tmp" }'

brc MapService/CancelMapping -d '{ "goal_id": "map-start-001" }'
```

> `GetMap` / `SaveMap` / `DeleteMap` / `SetCurrentMap` 的请求字段以 `describe` 与 `mapping.proto` 为准（部分实现用 `map_id` 而非 `map_name`）。

**终端期望**

`ListMaps`：

```text
{
 "status": { "code": "OK" },
 "maps": [
  { "map_name": "...", "map_identifier": "...", ... }
 ]
}
```

（无图时 `maps` 可为空数组。）

`GetMappingStatus`（空闲）：

```text
{
 "status": { "code": "OK" },
 "state": "...",
 "goal_id": "",
 "map_name": "",
 ...
}
```

`StartMapping` / `CancelMapping` / `SetCurrentMap` / `DeleteMap`：

```text
{
 "status": { "code": "OK" }
}
```

`FinishMapping`：

```text
{
 "status": { "code": "OK" },
 "map_identifier": "..."
}
```

`GetMapMetadata`：

```text
{
 "status": { "code": "OK" },
 "map_name": "cli_map_tmp",
 "map_identifier": "...",
 ...
}
```

`GetMap`：大体量 `map`（OccupancyGrid）打到 stdout；注意 `--timeout`。失败时可能 `status.code` 非 OK，或 stderr：

```text
RPC failed: … (code=…)
```

---

### 5.7 ExplorationService — `automsgs.rpcs.exploration.ExplorationService`

| Method | 形态 |
|--------|------|
| Explore | server-streaming |
| Pause / Resume / Cancel / GetStatus | unary |
| SetArea / SaveMap | unary |

`Polygon` 至少 3 点（实现侧可能校验）。

```bash
brc ExplorationService/Explore -d '{
  "header": { "frame_id": "map" },
  "goal_id": "explore-cli-001",
  "map_name": "explore_cli",
  "area": {
    "points": [
      { "x": 0.0, "y": 0.0, "z": 0.0 },
      { "x": 3.0, "y": 0.0, "z": 0.0 },
      { "x": 3.0, "y": 3.0, "z": 0.0 },
      { "x": 0.0, "y": 3.0, "z": 0.0 }
    ]
  },
  "options": {
    "enable_mapping": true,
    "coverage_target": 0.8,
    "timeout_seconds": 600
  }
}' --timeout 600

brc ExplorationService/GetStatus -d '{}'
brc ExplorationService/SetArea -d '{
  "goal_id": "explore-cli-001",
  "area": {
    "points": [
      { "x": 0.0, "y": 0.0 },
      { "x": 2.0, "y": 0.0 },
      { "x": 2.0, "y": 2.0 }
    ]
  }
}'
brc ExplorationService/SaveMap -d '{
  "goal_id": "explore-cli-001",
  "map_name": "explore_cli_saved"
}'
brc ExplorationService/Pause -d '{"goal_id":"explore-cli-001"}'
brc ExplorationService/Resume -d '{"goal_id":"explore-cli-001"}'
brc ExplorationService/Cancel -d '{"goal_id":"explore-cli-001"}'
```

**终端期望**

`Explore` 流帧：

```text
{
 "status": { "code": "OK" },
 "state": "EXPLORATION_STATE_RUNNING",
 "goal_id": "explore-cli-001",
 "progress": 0.1,
 "frontier_count": 3,
 "map_name": "explore_cli",
 "active": true
}
```

`SetArea` / `SaveMap` / lifecycle：`{ "status": { "code": "OK" } }`。  
`GetStatus`：单帧快照。

---

### 5.8 LocalizationService — `automsgs.rpcs.localization.LocalizationService`

| Method | 形态 |
|--------|------|
| GetPose / GetStatus | unary |
| SetInitialPose | unary |

```bash
brc LocalizationService/GetPose -d '{}'
brc LocalizationService/GetStatus -d '{}'

brc LocalizationService/SetInitialPose -d '{
  "map_id": "",
  "pose": {
    "pose": {
      "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
      "orientation": { "w": 1.0 }
    }
  }
}'
```

> `SetInitialPoseRequest.pose` 类型为 `PoseWithCovariance`。字段细节：

```bash
$BR describe LocalizationService/SetInitialPose
$BR describe automsgs.rpcs.localization.SetInitialPoseRequest
```

**终端期望**

`GetPose`：

```text
{
 "status": { "code": "OK" },
 "pose": { "header": { "frame_id": "map" }, "pose": { ... } },
 ...
}
```

（无定位时 pose 可能为空/默认，但仍应有外层 `status`。）

`GetStatus`：含定位状态枚举字段；`status.code=OK`。

`SetInitialPose`：

```text
{
 "status": { "code": "OK" }
}
```

---

### 5.9 VoiceService — `automsgs.rpcs.voice.VoiceService`

| Method | 形态 |
|--------|------|
| Execute | server-streaming |
| Cancel / GetStatus | unary |

`VoiceIntent`：`VOICE_INTENT_NAVIGATE` / `FOLLOW` / `DOCK` / `UNDOCK` / `EXPLORE` / `STOP` / `CANCEL_ALL` 等。

```bash
# 停 / 全取消（无 payload）
brc VoiceService/Execute -d '{
  "goal_id": "voice-stop-001",
  "transcript": "停止",
  "intent": "VOICE_INTENT_STOP"
}' --timeout 30

brc VoiceService/Execute -d '{
  "goal_id": "voice-cancel-all",
  "intent": "VOICE_INTENT_CANCEL_ALL"
}' --timeout 30

# 语音导航（oneof navigate）
brc VoiceService/Execute -d '{
  "goal_id": "voice-nav-001",
  "transcript": "去原点附近",
  "intent": "VOICE_INTENT_NAVIGATE",
  "navigate": {
    "goal_id": "voice-nav-inner",
    "waypoints": [{
      "header": { "frame_id": "map" },
      "pose": {
        "position": { "x": 0.5, "y": 0.0 },
        "orientation": { "w": 1.0 }
      }
    }]
  }
}' --timeout 180

brc VoiceService/GetStatus -d '{}'
brc VoiceService/Cancel -d '{"goal_id":"voice-nav-001"}'
```

**终端期望**

`Execute` 流帧：

```text
{
 "status": { "code": "OK" },
 "state": "VOICE_STATE_DISPATCHING",
 "goal_id": "voice-nav-001",
 "intent": "VOICE_INTENT_NAVIGATE",
 "detail": "",
 "active": true
}
```

后续可能 `VOICE_STATE_RUNNING` → `SUCCEEDED` / `FAILED` / `CANCELLED`。  
`GetStatus`：单帧；`Cancel`：`common.Status` OK。

---

### 5.10 SensorService — `automsgs.rpcs.sensor.SensorService`

| Method | 形态 |
|--------|------|
| ListSensors / GetSample | unary |
| Get/Set/Save/LoadParameters | unary |
| Record | server-streaming |
| CancelRecord / GetRecordStatus | unary |

```bash
brc SensorService/ListSensors -d '{}'

# 将 list 返回的 sensor_id 填入下方
export SID=front_laser   # 示例，以 ListSensors 为准

brc SensorService/GetSample -d '{
  "sensor_id": "'"$SID"'",
  "prefer_compressed": false
}' --timeout 30

brc SensorService/GetParameters -d '{
  "sensor_id": "'"$SID"'",
  "names": []
}'

brc SensorService/SetParameters -d '{
  "sensor_id": "'"$SID"'",
  "parameters": [
    { "name": "example_key", "value": "1" }
  ]
}'

brc SensorService/SaveParameters -d '{ "sensor_id": "'"$SID"'" }'
brc SensorService/LoadParameters -d '{ "sensor_id": "'"$SID"'" }'

brc SensorService/Record -d '{
  "record_id": "rec-cli-001",
  "sensor_ids": ["'"$SID"'"],
  "uri": "/tmp/bridge_cli_record",
  "duration_seconds": 5
}' --timeout 60

brc SensorService/GetRecordStatus -d '{}'
brc SensorService/CancelRecord -d '{ "record_id": "rec-cli-001" }'
```

**终端期望**

`ListSensors`：

```text
{
 "status": { "code": "OK" },
 "sensors": [
  { "sensor_id": "front_laser", "type": "...", ... }
 ]
}
```

`GetSample`：`status.code=OK`，`sensor_id` 回显，`data` oneof 中之一有内容（或失败码表示无样本）。

`GetParameters` / `SetParameters`：带 `parameters` 数组；`status.code=OK`。

`Record` 流帧：

```text
{
 "status": { "code": "OK" },
 "state": "RECORD_STATE_RUNNING",
 "record_id": "rec-cli-001",
 "uri": "/tmp/bridge_cli_record",
 "active": true,
 "final": false
}
```

终态帧常见 `final: true`。  
`GetRecordStatus`：单帧；`CancelRecord`：`status.code=OK`。

---

## 6. 推荐冒烟脚本（最小全集）

在 Bridge 已启动、机载栈可用时，按序执行（失败即停）：

```bash
#!/usr/bin/env bash
set -euo pipefail
BR=${BR:-autonomy.bridge}
TGT=${TGT:-127.0.0.1:5005}
c() { "$BR" call --target "$TGT" "$@"; }

echo "== catalog =="
"$BR" list | head

echo "== system queries =="
c SystemService/Heartbeat -d '{"sequence":1}'
c SystemService/GetInfo -d '{}'
c SystemService/GetStatus -d '{}'
c SystemService/GetHealth -d '{}'
c SystemService/GetCapabilities -d '{}'
c SystemService/GetActiveGoal -d '{}'
c SystemService/GetRobotFullInfo -d '{}'

echo "== localization / map / sensor =="
c LocalizationService/GetPose -d '{}'
c LocalizationService/GetStatus -d '{}'
c MapService/ListMaps -d '{}'
c MapService/GetMappingStatus -d '{}'
c SensorService/ListSensors -d '{}'

echo "== lifecycle no-ops (empty goal) =="
c NavigationService/GetStatus -d '{}'
c FollowService/GetStatus -d '{}'
c ChargeService/GetStatus -d '{}'
c TeleopService/GetStatus -d '{}'
c ExplorationService/GetStatus -d '{}'
c VoiceService/GetStatus -d '{}'

echo "== cancel all =="
c SystemService/CancelAllGoals -d '{"reason":"smoke-end"}'

echo "OK smoke"
```

保存为 `scripts/bridge_cli_smoke.sh` 后：`bash bridge_cli_smoke.sh`。

**终端期望**：每个 `c …` 在 stdout 打印至少一段含 `"code": "OK"` 的 JSON（查询类）或空闲态快照；脚本以 `set -e` 在首个非 0 退出码处停止；最后一行打印 `OK smoke`。

---

## 7. 互斥 / 急停联调场景

```bash
# 1) 启动导航流（另一终端或后台）
brc NavigationService/Navigate -d '{ "goal_id":"mux-1", "waypoints":[...]}' --timeout 300 &

# 2) 再起跟随 — 期望拒忙 / 失败帧（TaskMuxer）
brc FollowService/Follow -d '{ "goal_id":"mux-2", "target_type":"FOLLOW_TARGET_TYPE_PERSON" }'

# 3) Estop — 新命令应被拒
brc SystemService/EmergencyStop -d '{"reason":"mux-test"}'
brc NavigationService/Navigate -d '{ "goal_id":"after-estop", "waypoints":[...] }'

# 4) 清理
brc SystemService/ClearEmergencyStop -d '{"reason":"done"}'
brc SystemService/CancelAllGoals -d '{"reason":"done"}'
```

**终端期望**

| 步骤 | 期望 |
|------|------|
| 步骤 2 Follow 拒忙 | stdout 拒绝帧（业务 `status`/`message` 含 busy）**或** stderr `RPC failed`；不应与 nav 同时成功占槽 |
| 步骤 3 后再 Navigate | 拒收 / Estop 相关错误 |
| `GetStatus` 在 Estop 后 | `"state": "SYSTEM_STATE_ESTOP"` |
| 清理后 | `CancelAllGoals` / `ClearEmergencyStop` 返回 `"code": "OK"` |

---

## 8. 与 grpcurl / rpc-cli.py 对照

| 工具 | 适用 |
|------|------|
| **`autonomy.bridge call`** | 机载同仓、已链接 automsgs；unary + server-stream；无需另装 grpcurl |
| **`rpc-cli.py`** | 任意机器 + grpcurl；含 Velocity bidi；手册字段极细 |
| **`grpcurl`** | reflection 开启或自备 proto import |

同一方法示例（三者成功时业务 JSON 应对齐，字段名/枚举名一致）：

```bash
# bridge — stdout 见 §5.1.1
brc SystemService/Heartbeat -d '{"sequence":1}'

# rpc-cli
./rpc-cli.py -t $TGT call SystemService/Heartbeat -d '{"sequence":1}'

# grpcurl + reflection
grpcurl -plaintext -d '{"sequence":1}' $TGT \
  automsgs.rpcs.system.SystemService/Heartbeat
```

---

## 9. 方法清单速查（勾选表）

### SystemService

- [ ] Heartbeat
- [ ] GetInfo
- [ ] GetStatus
- [ ] GetHealth
- [ ] GetRobotFullInfo
- [ ] EmergencyStop
- [ ] ClearEmergencyStop
- [ ] CancelAllGoals
- [ ] GetActiveGoal
- [ ] GetCapabilities

### NavigationService

- [ ] Navigate
- [ ] Pause / Resume / Replan / Cancel
- [ ] GetStatus

### FollowService

- [ ] Follow
- [ ] Pause / Resume / Cancel
- [ ] GetStatus

### ChargeService

- [ ] Return / Leave
- [ ] Pause / Resume / Cancel
- [ ] GetStatus

### TeleopService

- [ ] DriveOnHeading / BackUp / Spin
- [ ] Pause / Resume / Cancel / GetStatus
- [ ] Velocity（外部工具）

### MapService

- [ ] StartMapping / FinishMapping / CancelMapping / GetMappingStatus
- [ ] ListMaps / GetMap / GetMapMetadata
- [ ] SaveMap / DeleteMap / SetCurrentMap

### ExplorationService

- [ ] Explore
- [ ] Pause / Resume / Cancel / GetStatus
- [ ] SetArea / SaveMap

### LocalizationService

- [ ] GetPose / GetStatus / SetInitialPose

### VoiceService

- [ ] Execute（STOP / CANCEL_ALL / NAVIGATE / …）
- [ ] Cancel / GetStatus

### SensorService

- [ ] ListSensors / GetSample
- [ ] Get/Set/Save/LoadParameters
- [ ] Record / CancelRecord / GetRecordStatus

### 平台

- [ ] `-n` dry-run / `-t` self-test
- [ ] Bearer / x-robot-id 强制
- [ ] Health probe /（可选）Reflection

---

## 10. 故障排查

失败时消息多在 **stderr**（一行），进程退出码 ≠ 0；业务拒收但 gRPC OK 时仍可能只有 stdout JSON（见 §2.5）。

| 现象（终端可见） | 处理 |
|------------------|------|
| `unknown method: … (try: autonomy.bridge list)` | 用 `Service/Method`；先 `list` |
| `unknown service: …` / `unknown symbol: …` | 符号拼写；`list` / `describe` |
| `RPC failed: … failed to connect …` / `Connection refused` | 确认服务已起；`--target` 与 conf `host:port` 一致 |
| `RPC failed: missing or invalid bearer token (code=16)` | `--bearer` 与 conf `bearer_token` 一致 |
| `RPC failed: missing x-robot-id (code=9)` | `--robot-id` |
| `RPC failed: … (code=8)`（限流） | 降 QPS 或关 `enable_rate_limit` |
| `RPC failed: … (code=4)` Deadline | 加大 `--timeout`；查 Task / Autolink |
| `client/bidi streaming not supported in autonomy.bridge call (bidi-streaming). Use grpcurl or rpc-cli.py for /…/Velocity` | 换工具 |
| `invalid JSON for …:` | `describe` 看字段；enum 用名字字符串 |
| 流式过早结束 / `# received 0 frames` | 下游未起或立刻失败；查服务端日志 |

---

## 11. 相关文档

| 文档 | 内容 |
|------|------|
| [01_grpc_platform_overview.md](01_grpc_platform_overview.md) | 平台请求路径 |
| [02_policy.md](02_policy.md) | 拒绝码表 |
| [06_configuration.md](06_configuration.md) | GrpcOptions |
| [08_testing.md](08_testing.md) | gtest 矩阵 |
| [`../README.md`](../README.md) | 模块总览与 CLI 摘要 |
| [`automsgs/tools/cli/README.md`](../../../automsgs/tools/cli/README.md) | rpc-cli + grpcurl 字段级手册 |
