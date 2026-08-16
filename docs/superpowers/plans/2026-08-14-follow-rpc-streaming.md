# FollowService Server-Streaming RPC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rewrite `FollowService` as a generalized target-follow API with server-streaming `Follow`, unary `Cancel`/`GetStatus`, and clear Google-style comments.

**Architecture:** Align with `DockingService` hybrid model, but keep the stream open for the whole follow session (`LOST_TARGET` does not end the stream). Replace person-only `StartFollow` with `Follow` + `FollowTargetType` + optional `hint_pose`. No bidi, no Retarget.

**Tech Stack:** Protocol Buffers 3, `automsgs.msgs.geometry_msgs.Pose`, `automsgs.rpcs.common.Status`.

**Spec:** `docs/superpowers/specs/2026-08-14-follow-rpc-streaming-design.md`

**Commits:** Do **not** git commit / push / PR unless the user explicitly asks.

---

## File map

| Path | Action |
|------|--------|
| `automsgs/proto/rpcs/follow.proto` | Rewrite service, enums, messages, comments |
| `automsgs/proto/rpcs/README.md` | Follow inventory row + Follow streaming subsection; drop `StartFollow` from unary list |
| `docs/superpowers/specs/2026-08-14-follow-rpc-streaming-design.md` | Mark `状态：已批准` after acceptance |
| `automsgs/proto/rpcs/common.proto` | No change |

**Verify:**

```bash
cd /Users/quandy/Workspace/github/autonomy
grep -n "rpc Follow\|stream FollowFeedback\|StartFollow\|FollowTargetType\|hint_pose\|FollowPhase\|progress" automsgs/proto/rpcs/follow.proto
grep -n "Follow\|LOST_TARGET\|单端流\|StartFollow" automsgs/proto/rpcs/README.md
```

---

### Task 1: Rewrite `follow.proto`

**Files:**
- Modify: `automsgs/proto/rpcs/follow.proto`

- [ ] **Step 1: Overwrite body (keep copyright header)**

Replace content from `syntax` onward with:

```protobuf
syntax = "proto3";

package automsgs.rpcs.follow;

import "automsgs/msgs/std_msgs/header.proto";
import "automsgs/msgs/geometry_msgs/pose.proto";
import "automsgs/rpcs/common.proto";

// Target following RPCs (person, object, vehicle, or custom).
// Follow: server streaming for the follow session.
//   Close stream on Cancel, FAILED, or normal session end.
//   LOST_TARGET keeps the stream open for reacquisition.
// Cancel / GetStatus: unary. No bidirectional streaming.
service FollowService {
  rpc Follow(FollowRequest) returns (stream FollowFeedback);
  rpc Cancel(CancelRequest) returns (CancelResponse);
  rpc GetStatus(GetStatusRequest) returns (GetStatusResponse);
}

// Target class for Follow. UNSPECIFIED lets the implementation choose a default.
enum FollowTargetType {
  FOLLOW_TARGET_TYPE_UNSPECIFIED = 0;
  FOLLOW_TARGET_TYPE_PERSON = 1;
  FOLLOW_TARGET_TYPE_OBJECT = 2;
  FOLLOW_TARGET_TYPE_VEHICLE = 3;
  FOLLOW_TARGET_TYPE_CUSTOM = 4;
}

message FollowRequest {
  automsgs.msgs.std_msgs.Header header = 1;
  FollowTargetType target_type = 2;
  // Empty = implementation selects a target (e.g. nearest person).
  string target_id = 3;
  // Optional pose hint to help lock the target.
  automsgs.msgs.geometry_msgs.Pose hint_pose = 4;
  // Optional client-defined id for tracking.
  string goal_id = 5;
  // Desired follow distance in meters; 0 = use implementation default.
  float desired_distance_m = 6;
}

message CancelRequest {
  // Empty = cancel the current active follow goal.
  string goal_id = 1;
}

message CancelResponse {
  automsgs.rpcs.common.Status status = 1;
}

// Coarse follow lifecycle.
enum FollowState {
  FOLLOW_STATE_UNSPECIFIED = 0;
  FOLLOW_STATE_IDLE = 1;
  FOLLOW_STATE_ACQUIRING = 2;
  FOLLOW_STATE_FOLLOWING = 3;
  FOLLOW_STATE_LOST_TARGET = 4;
  FOLLOW_STATE_FAILED = 5;
  FOLLOW_STATE_CANCELLED = 6;
}

// Fine-grained phase inside FollowFeedback.
enum FollowPhase {
  FOLLOW_PHASE_UNSPECIFIED = 0;
  FOLLOW_PHASE_IDLE = 1;
  FOLLOW_PHASE_SEARCHING = 2;
  FOLLOW_PHASE_LOCKING = 3;
  FOLLOW_PHASE_TRACKING = 4;
  FOLLOW_PHASE_REACQUIRING = 5;
  FOLLOW_PHASE_COMPLETED = 6;
  FOLLOW_PHASE_FAILED = 7;
  FOLLOW_PHASE_CANCELLED = 8;
}

// Server-stream feedback for Follow.
message FollowFeedback {
  automsgs.rpcs.common.Status status = 1;
  FollowState state = 2;
  FollowPhase phase = 3;
  string goal_id = 4;
  FollowTargetType target_type = 5;
  string target_id = 6;
  // Distance to target in meters; unknown may be 0.
  float distance_m = 7;
  // True while state is ACQUIRING, FOLLOWING, or LOST_TARGET.
  bool active = 8;
  string message = 9;
}

message GetStatusRequest {}

message GetStatusResponse {
  automsgs.rpcs.common.Status status = 1;
  FollowState state = 2;
  FollowPhase phase = 3;
  string goal_id = 4;
  FollowTargetType target_type = 5;
  string target_id = 6;
  float distance_m = 7;
  bool active = 8;
  string message = 9;
}
```

- [ ] **Step 2: Verify shape**

```bash
cd /Users/quandy/Workspace/github/autonomy
grep -E "rpc Follow|stream FollowFeedback|StartFollow|hint_pose|FollowTargetType|FollowPhase|float progress|rpc .*\(stream " automsgs/proto/rpcs/follow.proto
```

Expected:
- `rpc Follow(FollowRequest) returns (stream FollowFeedback);`
- `hint_pose`, `FollowTargetType`, `FollowPhase` present
- **No** `StartFollow`, **No** `progress`
- No request-side `rpc Foo(stream` (only `returns (stream ...)`)

- [ ] **Step 3: Confirm imports**

```bash
grep "^import " automsgs/proto/rpcs/follow.proto
```

Expected exactly:
- `automsgs/msgs/std_msgs/header.proto`
- `automsgs/msgs/geometry_msgs/pose.proto`
- `automsgs/rpcs/common.proto`

---

### Task 2: Update `rpcs/README.md`

**Files:**
- Modify: `automsgs/proto/rpcs/README.md`

- [ ] **Step 1: Update follow inventory row**

```markdown
| `follow.proto` | `automsgs.rpcs.follow` | `FollowService` | 泛化目标跟随：Follow 为 server streaming；Cancel/GetStatus 为 unary |
```

- [ ] **Step 2: Fix unary long-task list (remove StartFollow)**

In `## 长任务约定`, change item 1 to:

```markdown
1. `GoTo` / `StartMapping` 等 **立即返回** `goal_id`（接受目标）。
```

Also change the intro line from「导航、跟随、建图」to「导航、建图等」so Follow is not listed under unary polling defaults:

```markdown
导航、建图等长任务（默认 unary 接受 + 轮询）：
```

- [ ] **Step 3: Add Follow streaming subsection after Docking**

Insert immediately after the Docking subsection (after the BUSY/CANCELLED paragraph), before `## 状态码分模块`:

```markdown
### Follow（流式例外）

| RPC | 形态 | 说明 |
|-----|------|------|
| `Follow` | **server streaming（单端流）** | `stream FollowFeedback`（`state` / `phase` / `distance_m`）；会话期保持开流 |
| `Cancel` / `GetStatus` | **unary** | 可与进行中的 stream 并行 |
| — | **无双端流** | 取消一律 `Cancel` |

**目标：** `FollowTargetType`（PERSON / OBJECT / VEHICLE / CUSTOM）+ `target_id`（空=实现自选）+ 可选 `hint_pose`。

**关流：** Cancel / `FAILED` / 会话正常结束。**`LOST_TARGET` 不关流**（可重锁定）。

忙时新 `Follow`：首条 Feedback `CODE_FOLLOW_BUSY`（902）并关流。`Cancel` 成功 → `CODE_OK`；流末条 `CANCELLED`，Feedback.`status` = `CODE_FOLLOW_CANCELLED`（903）。
```

- [ ] **Step 4: Verify README**

```bash
grep -n "Follow\|LOST_TARGET\|StartFollow\|单端流\|hint_pose\|CODE_FOLLOW_BUSY" automsgs/proto/rpcs/README.md
```

Expected: Follow subsection + LOST_TARGET + BUSY; **no** remaining `StartFollow` as an active API name in the long-task list.

---

### Task 3: Spec status + acceptance

**Files:**
- Modify: `docs/superpowers/specs/2026-08-14-follow-rpc-streaming-design.md` (status line)

- [ ] **Step 1: Mark approved**

Change `状态：待审阅` → `状态：已批准`.

- [ ] **Step 2: Acceptance greps**

```bash
cd /Users/quandy/Workspace/github/autonomy

grep "rpc Follow(FollowRequest) returns (stream FollowFeedback);" automsgs/proto/rpcs/follow.proto
grep "rpc Cancel(CancelRequest) returns (CancelResponse);" automsgs/proto/rpcs/follow.proto
grep "rpc GetStatus(GetStatusRequest) returns (GetStatusResponse);" automsgs/proto/rpcs/follow.proto
grep "FollowTargetType" automsgs/proto/rpcs/follow.proto
grep "hint_pose" automsgs/proto/rpcs/follow.proto
grep "FollowPhase" automsgs/proto/rpcs/follow.proto
! grep -E "StartFollow|float progress" automsgs/proto/rpcs/follow.proto
! grep -E 'rpc [A-Za-z]+\(stream ' automsgs/proto/rpcs/follow.proto
! grep -iE "Retarget|ListTargets" automsgs/proto/rpcs/follow.proto
grep -E "LOST_TARGET|单端流|FollowTargetType" automsgs/proto/rpcs/README.md
```

Expected: positive greps match; negated greps exit 0.

- [ ] **Step 3: Do not commit**

Leave working tree as-is unless the user asks for git.

---

## Spec coverage

| Spec | Task |
|------|------|
| §3 service + stream rules | Task 1 |
| §4 types/fields/enums | Task 1 |
| §5 codes (docs only) | Task 2 |
| §6 comments/style | Task 1 |
| §7 files | Task 1–2 |
| §8 acceptance | Task 3 |
