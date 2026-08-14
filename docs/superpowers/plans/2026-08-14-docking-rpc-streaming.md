# DockingService Server-Streaming RPC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Upgrade `DockingService` so `Dock`/`Undock` are server-streaming with `DockingPhase` + progress feedback, while `Cancel`/`GetStatus` stay unary.

**Architecture:** Breaking change on return type only: replace unary `DockResponse`/`UndockResponse` with shared `stream DockingFeedback`. Keep existing request messages, `DockingState` 0–8, and `CODE_CHARGING_*`. Stream closes on dock/undock terminal states (not charge-to-full). Document unary vs server-stream in README long-task section.

**Tech Stack:** Protocol Buffers 3 (`syntax = "proto3"`), gRPC-style `stream` keyword, existing `automsgs/proto/CMakeLists.txt` GLOB on `rpcs/*.proto`.

**Spec:** `docs/superpowers/specs/2026-08-14-docking-rpc-streaming-design.md`

**Commits:** Only when the user explicitly asks to commit (do not auto-commit during execution unless requested).

---

## File map

| Path | Action |
|------|--------|
| `automsgs/proto/rpcs/docking.proto` | Modify: server-stream Dock/Undock; add `DockingPhase`, `DockingFeedback`; extend `GetStatusResponse`; remove unary responses |
| `automsgs/proto/rpcs/README.md` | Modify: docking row + long-task rules for streaming vs unary |
| `automsgs/proto/rpcs/common.proto` | No change |

**Verify commands (repo root or `automsgs/`):**

```bash
cd /Users/quandy/Workspace/github/autonomy
grep -n "rpc Dock\|rpc Undock\|rpc Cancel\|rpc GetStatus\|stream DockingFeedback\|DockingPhase\|DockResponse\|UndockResponse" automsgs/proto/rpcs/docking.proto
grep -n "server streaming\|单端流\|DockingFeedback\|关流" automsgs/proto/rpcs/README.md
```

Expected: Dock/Undock show `returns (stream DockingFeedback)`; Cancel/GetStatus unary; no `DockResponse`/`UndockResponse`; README mentions streaming close semantics.

---

### Task 1: Rewrite `docking.proto` service + Feedback

**Files:**
- Modify: `automsgs/proto/rpcs/docking.proto`

- [ ] **Step 1: Replace service and messages**

Overwrite `automsgs/proto/rpcs/docking.proto` with the following (keep copyright header already in file; body from `syntax` onward):

```protobuf
syntax = "proto3";

package automsgs.rpcs.docking;

import "automsgs/msgs/std_msgs/header.proto";
import "automsgs/rpcs/common.proto";

// Auto-docking / charging contact RPCs.
// Dock/Undock: server streaming (process feedback; close on dock/undock terminal).
// Cancel/GetStatus: unary.
service DockingService {
  rpc Dock(DockRequest) returns (stream DockingFeedback);
  rpc Undock(UndockRequest) returns (stream DockingFeedback);
  rpc Cancel(CancelRequest) returns (CancelResponse);
  rpc GetStatus(GetStatusRequest) returns (GetStatusResponse);
}

message DockRequest {
  automsgs.msgs.std_msgs.Header header = 1;
  string dock_id = 2;  // Empty = default dock.
  string goal_id = 3;  // Optional client id.
}

message UndockRequest {
  automsgs.msgs.std_msgs.Header header = 1;
  string goal_id = 2;
}

message CancelRequest {
  string goal_id = 1;  // Empty = cancel current.
}

message CancelResponse {
  automsgs.rpcs.common.Status status = 1;
}

enum DockingState {
  DOCKING_STATE_UNSPECIFIED = 0;
  DOCKING_STATE_IDLE = 1;
  DOCKING_STATE_DOCKING = 2;
  DOCKING_STATE_UNDOCKING = 3;
  DOCKING_STATE_DOCKED_NOT_CHARGING = 4;
  DOCKING_STATE_CHARGING = 5;
  DOCKING_STATE_FULL = 6;
  DOCKING_STATE_FAILED = 7;
  DOCKING_STATE_CANCELLED = 8;
}

enum DockingPhase {
  DOCKING_PHASE_UNSPECIFIED = 0;
  DOCKING_PHASE_IDLE = 1;
  DOCKING_PHASE_APPROACHING = 2;
  DOCKING_PHASE_ALIGNING = 3;
  DOCKING_PHASE_CONTACTING = 4;
  DOCKING_PHASE_CHARGING_SETUP = 5;
  DOCKING_PHASE_UNDOCKING = 6;
  DOCKING_PHASE_COMPLETED = 7;
  DOCKING_PHASE_FAILED = 8;
  DOCKING_PHASE_CANCELLED = 9;
}

// Shared server-stream feedback for Dock and Undock.
message DockingFeedback {
  automsgs.rpcs.common.Status status = 1;
  DockingState state = 2;
  DockingPhase phase = 3;
  float progress = 4;  // 0..1; unknown may be 0.
  string goal_id = 5;
  string dock_id = 6;
  float battery_pct = 7;  // 0..100
  bool active = 8;
  string message = 9;
}

message GetStatusRequest {}

message GetStatusResponse {
  automsgs.rpcs.common.Status status = 1;
  DockingState state = 2;
  string goal_id = 3;
  string dock_id = 4;
  float battery_pct = 5;  // 0..100
  bool active = 6;
  string message = 7;
  DockingPhase phase = 8;
  float progress = 9;  // 0..1
}
```

- [ ] **Step 2: Verify proto shape**

Run:

```bash
cd /Users/quandy/Workspace/github/autonomy
grep -E "rpc Dock|rpc Undock|stream DockingFeedback|message DockResponse|message UndockResponse|DOCKING_PHASE_|phase = 8|progress = 9" automsgs/proto/rpcs/docking.proto
```

Expected:
- `rpc Dock(DockRequest) returns (stream DockingFeedback);`
- `rpc Undock(UndockRequest) returns (stream DockingFeedback);`
- `DockingPhase` values present
- `GetStatusResponse` has `phase = 8` and `progress = 9`
- **No** matches for `message DockResponse` or `message UndockResponse`

- [ ] **Step 3: Confirm no bidi**

Run:

```bash
grep -n "stream " automsgs/proto/rpcs/docking.proto
```

Expected: only `returns (stream DockingFeedback)` on Dock and Undock — **no** `rpc ...(stream` on the request side.

---

### Task 2: Update `rpcs/README.md` streaming rules

**Files:**
- Modify: `automsgs/proto/rpcs/README.md`

- [ ] **Step 1: Update docking inventory row**

Change the docking row in the file table to:

```markdown
| `docking.proto` | `automsgs.rpcs.docking` | `DockingService` | 自动回充：Dock/Undock 为 server streaming；Cancel/GetStatus 为 unary |
```

- [ ] **Step 2: Replace「长任务约定」section**

Replace the entire `## 长任务约定` section with:

```markdown
## 长任务约定

导航、跟随、建图等长任务（默认 unary 接受 + 轮询）：

1. `GoTo` / `StartFollow` / `StartMapping` 等 **立即返回** `goal_id`（接受目标）。
2. 进度与终态通过 `GetStatus`（或域内等价查询）轮询。
3. `Cancel(goal_id)`：空 id = 取消当前活动目标。
4. 单一活动目标；忙时拒绝新请求（对应模块 `*_BUSY` 错误码），不抢占。

### Docking（流式例外）

| RPC | 形态 | 说明 |
|-----|------|------|
| `Dock` / `Undock` | **server streaming（单端流）** | 返回 `stream DockingFeedback`（含 `state` / `phase` / `progress`） |
| `Cancel` / `GetStatus` | **unary** | 取消与快照查询；可与进行中的 stream 并行 |
| — | **无双端流** | 取消不走 client stream，一律 `Cancel` |

**关流：** 对接/离桩动作进入终态后发末条并关闭 stream（Dock 成功：`DOCKED_*` / `CHARGING` / `FULL`；Undock 成功：`IDLE`；或 `FAILED` / `CANCELLED`）。**不以充至满作为关流条件**；充电驻留用电量继续用 `GetStatus` 观察。

忙时新 `Dock`/`Undock`：首条 Feedback `CODE_CHARGING_BUSY`（504）并关流。`Cancel` 成功 → `CODE_OK`；对应 stream 末条 `state`/`phase` = `CANCELLED`，Feedback.`status` = `CODE_CHARGING_CANCELLED`（505）。
```

- [ ] **Step 3: Verify README**

Run:

```bash
grep -n "server streaming\|单端流\|无双端流\|关流\|DockingFeedback\|CODE_CHARGING_BUSY" automsgs/proto/rpcs/README.md
```

Expected: hits for server streaming / 单端流 / 无双端流 / 关流 / DockingFeedback / BUSY.

---

### Task 3: Spec status + acceptance checklist

**Files:**
- Modify: `docs/superpowers/specs/2026-08-14-docking-rpc-streaming-design.md` (status line only)
- Verify only (no code): docking.proto + README vs spec §8

- [ ] **Step 1: Mark spec approved**

Change the status line from `状态：待审阅` to `状态：已批准`.

- [ ] **Step 2: Run acceptance grep checklist**

```bash
cd /Users/quandy/Workspace/github/autonomy

# §8.1–8.2 streaming vs unary
grep "rpc Dock(DockRequest) returns (stream DockingFeedback);" automsgs/proto/rpcs/docking.proto
grep "rpc Undock(UndockRequest) returns (stream DockingFeedback);" automsgs/proto/rpcs/docking.proto
grep "rpc Cancel(CancelRequest) returns (CancelResponse);" automsgs/proto/rpcs/docking.proto
grep "rpc GetStatus(GetStatusRequest) returns (GetStatusResponse);" automsgs/proto/rpcs/docking.proto

# §8.3 phase + progress
grep "DockingPhase phase" automsgs/proto/rpcs/docking.proto
grep "float progress" automsgs/proto/rpcs/docking.proto

# §8.4–8.5 README + no bidi request stream
grep -E "单端流|关流|无双端流" automsgs/proto/rpcs/README.md
! grep -E "rpc .*\(stream " automsgs/proto/rpcs/docking.proto

# §8.6 no ListDocks / SoC in docking.proto
! grep -iE "ListDocks|target_soc|SoC" automsgs/proto/rpcs/docking.proto
```

Expected: all positive greps match; `! grep` commands exit 0 (no matches).

- [ ] **Step 3: Optional proto build**

If an `automsgs` build tree exists:

```bash
cmake --build /Users/quandy/Workspace/github/autonomy/automsgs/build --target automsgs_protos 2>/dev/null \
  || echo "SKIP: no build tree / target; GLOB will pick up docking.proto on next configure"
```

Expected: build success **or** explicit SKIP (proto-only change is still complete if grep checks pass).

---

## Spec coverage (self-review)

| Spec section | Task |
|--------------|------|
| §3 service surface + stream rules | Task 1 |
| §3.2 delete unary Dock/Undock responses | Task 1 |
| §4 Phase + Feedback + GetStatus fields | Task 1 |
| §5 error code conventions (docs) | Task 2 README |
| §6 file list | Task 1–2 |
| §7 breaking note | Task 2 README |
| §8 acceptance | Task 3 |

No bidi, no `common.proto` Code changes, no ListDocks/SoC — explicitly out of plan.
