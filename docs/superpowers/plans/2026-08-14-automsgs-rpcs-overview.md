# automsgs RPCs Overview Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reorganize `automsgs/proto/rpcs` into eight domain protos with complete third-party robot business APIs (nav, mapping, localization, docking, follow, sensor, system) per the overview spec.

**Architecture:** Rename `status.proto` → `common.proto`; split `world` into `mapping` + `localization`; replace `charging` with `docking`; fold `vehicle`/`task` into `system`; add `follow`. Long-running ops use async `goal_id` + `Cancel` + `GetStatus`. Prefer `automsgs/msgs` payload types. Breaking change; no deprecated shims.

**Tech Stack:** Protocol Buffers 3, existing `automsgs/proto/CMakeLists.txt` GLOB on `rpcs/*.proto`, msgs under `automsgs/proto/msgs/`.

**Specs:**
- `docs/superpowers/specs/2026-08-14-automsgs-rpcs-overview-design.md`
- `docs/superpowers/specs/2026-08-14-charging-rpc-nav-align-design.md` (docking details)

**Commits:** Only when the user explicitly asks to commit (do not auto-commit during execution unless requested).

---

## File map

| Path | Action |
|------|--------|
| `automsgs/proto/rpcs/common.proto` | Create (from status + new Code ranges) |
| `automsgs/proto/rpcs/status.proto` | Delete |
| `automsgs/proto/rpcs/docking.proto` | Create (from charging A + rename) |
| `automsgs/proto/rpcs/charging.proto` | Delete |
| `automsgs/proto/rpcs/mapping.proto` | Create (world map CRUD + Start/Stop mapping) |
| `automsgs/proto/rpcs/localization.proto` | Replace empty file (from world loc APIs) |
| `automsgs/proto/rpcs/world.proto` | Delete |
| `automsgs/proto/rpcs/navigation.proto` | Update service name + import |
| `automsgs/proto/rpcs/sensor.proto` | Rename `List`/`GetGrid`, fix import |
| `automsgs/proto/rpcs/follow.proto` | Create |
| `automsgs/proto/rpcs/system.proto` | Replace empty (info/heartbeat/task/status) |
| `automsgs/proto/rpcs/vehicle.proto` | Delete |
| `automsgs/proto/rpcs/task.proto` | Delete |
| `automsgs/proto/rpcs/README.md` | Rewrite inventory + Code ranges |

**Verify after each task (when build tree exists):**

```bash
cd /path/to/autonomy/automsgs
# Prefer project build if configured:
cmake --build build --target automsgs_protos 2>/dev/null || \
  find proto/rpcs -name '*.proto' | sort
grep -R "status\.proto\|charging\.proto\|world\.proto\|VehicleService\|TaskService\|ChargeService\|WorldService" proto/rpcs || true
```

Expected after final task: only the eight files + README; no hits on deleted names except historical docs outside `rpcs/`.

---

### Task 1: `common.proto` + Code ranges

**Files:**
- Create: `automsgs/proto/rpcs/common.proto`
- Delete: `automsgs/proto/rpcs/status.proto`

- [ ] **Step 1: Write `common.proto`**

Copy `status.proto` content into `common.proto`. Keep `package automsgs.rpcs.common;`. Update the header comment ranges, and append:

```protobuf
  // Charging / docking (500-599) — keep CODE_CHARGING_* wire values
  CODE_CHARGING_DOCK_NOT_FOUND = 500;
  CODE_CHARGING_DOCK_FAILED = 501;
  CODE_CHARGING_ALREADY_DOCKED = 502;
  CODE_CHARGING_ALREADY_UNDOCKED = 503;
  CODE_CHARGING_BUSY = 504;
  CODE_CHARGING_CANCELLED = 505;
  CODE_CHARGING_TIMEOUT = 506;
  CODE_CHARGING_NOT_READY = 507;

  // Map storage (600-699) — unchanged
  // Sensor (700-799) — unchanged
  // Localization (800-899) — unchanged

  // Follow (900-999)
  CODE_FOLLOW_TARGET_NOT_FOUND = 900;
  CODE_FOLLOW_FAILED = 901;
  CODE_FOLLOW_BUSY = 902;
  CODE_FOLLOW_CANCELLED = 903;

  // System (1000-1099)
  CODE_SYSTEM_NOT_READY = 1000;
  CODE_SYSTEM_ESTOP = 1001;
  CODE_SYSTEM_HARDWARE_FAULT = 1002;

  // Mapping process (1100-1199) — distinct from map storage 600-699
  CODE_MAPPING_BUSY = 1100;
  CODE_MAPPING_NOT_RUNNING = 1101;
  CODE_MAPPING_FAILED = 1102;
```

Also extend the top-of-enum comment to list follow/system/mapping-process ranges. Keep vehicle 200–299 and task 300–399 codes (still used by system/task semantics).

- [ ] **Step 2: Delete `status.proto` and fix temporary imports**

Until other files are updated, every `import "automsgs/rpcs/status.proto";` must become `import "automsgs/rpcs/common.proto";` in remaining protos (`navigation`, `sensor`, `charging`, `world`, `vehicle`, `task`) so the tree still parses. Do this import rename in the same step.

- [ ] **Step 3: Verify**

```bash
test -f automsgs/proto/rpcs/common.proto
test ! -f automsgs/proto/rpcs/status.proto
grep -n "CODE_FOLLOW_BUSY\|CODE_MAPPING_BUSY\|CODE_CHARGING_BUSY" automsgs/proto/rpcs/common.proto
grep -R "status\.proto" automsgs/proto/rpcs && exit 1 || true
```

Expected: codes present; no `status.proto` imports left.

- [ ] **Step 4: Commit (only if user asked)**

```bash
git add automsgs/proto/rpcs/common.proto automsgs/proto/rpcs/status.proto \
  automsgs/proto/rpcs/*.proto
git commit -m "refactor(automsgs): rename status.proto to common.proto and extend Code ranges"
```

---

### Task 2: `docking.proto` (from charging A)

**Files:**
- Create: `automsgs/proto/rpcs/docking.proto`
- Delete: `automsgs/proto/rpcs/charging.proto`

- [ ] **Step 1: Write full `docking.proto`**

```protobuf
// Copyright 2025 The Openbot Authors (duyongquan)
syntax = "proto3";

package automsgs.rpcs.docking;

import "automsgs/msgs/std_msgs/header.proto";
import "automsgs/rpcs/common.proto";

// Auto-docking / charging contact RPCs (async goals, Nav-aligned).
service DockingService {
  rpc Dock(DockRequest) returns (DockResponse);
  rpc Undock(UndockRequest) returns (UndockResponse);
  rpc Cancel(CancelRequest) returns (CancelResponse);
  rpc GetStatus(GetStatusRequest) returns (GetStatusResponse);
}

message DockRequest {
  automsgs.msgs.std_msgs.Header header = 1;
  string dock_id = 2;   // Empty = default dock.
  string goal_id = 3;   // Optional client id.
}

message DockResponse {
  automsgs.rpcs.common.Status status = 1;
  string goal_id = 2;
}

message UndockRequest {
  automsgs.msgs.std_msgs.Header header = 1;
  string goal_id = 2;
}

message UndockResponse {
  automsgs.rpcs.common.Status status = 1;
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

message GetStatusRequest {}

message GetStatusResponse {
  automsgs.rpcs.common.Status status = 1;
  DockingState state = 2;
  string goal_id = 3;
  string dock_id = 4;
  float battery_pct = 5;  // 0..100
  bool active = 6;
  string message = 7;
}
```

Note: Enum renamed to `DockingState` (overview naming); semantics match charging A `ChargingState` table.

- [ ] **Step 2: Delete `charging.proto`**

- [ ] **Step 3: Verify**

```bash
test -f automsgs/proto/rpcs/docking.proto
test ! -f automsgs/proto/rpcs/charging.proto
grep -n "service DockingService\|DOCKING_STATE_DOCKING\|battery_pct" automsgs/proto/rpcs/docking.proto
```

- [ ] **Step 4: Commit (only if user asked)**

```bash
git commit -m "feat(automsgs): add DockingService RPC aligned with Nav async model"
```

---

### Task 3: `mapping.proto` + `localization.proto`; remove `world.proto`

**Files:**
- Create: `automsgs/proto/rpcs/mapping.proto`
- Write: `automsgs/proto/rpcs/localization.proto`
- Delete: `automsgs/proto/rpcs/world.proto`

- [ ] **Step 1: Write `mapping.proto`**

Include map CRUD from `world.proto` (same messages/`MapSummary`), plus:

```protobuf
service MappingService {
  rpc StartMapping(StartMappingRequest) returns (StartMappingResponse);
  rpc StopMapping(StopMappingRequest) returns (StopMappingResponse);
  rpc GetMappingStatus(GetMappingStatusRequest) returns (GetMappingStatusResponse);
  rpc ListMaps(ListMapsRequest) returns (ListMapsResponse);
  rpc GetMap(GetMapRequest) returns (GetMapResponse);
  rpc GetMapMetadata(GetMapMetadataRequest) returns (GetMapMetadataResponse);
  rpc SaveMap(SaveMapRequest) returns (SaveMapResponse);
  rpc DeleteMap(DeleteMapRequest) returns (DeleteMapResponse);
  rpc SetCurrentMap(SetCurrentMapRequest) returns (SetCurrentMapResponse);
}

enum MappingState {
  MAPPING_STATE_UNSPECIFIED = 0;
  MAPPING_STATE_IDLE = 1;
  MAPPING_STATE_MAPPING = 2;
  MAPPING_STATE_SAVING = 3;
  MAPPING_STATE_FAILED = 4;
  MAPPING_STATE_CANCELLED = 5;
}

message StartMappingRequest {
  automsgs.msgs.std_msgs.Header header = 1;
  string map_name = 2;
  string goal_id = 3;
}
message StartMappingResponse {
  automsgs.rpcs.common.Status status = 1;
  string goal_id = 2;
}
message StopMappingRequest {
  string goal_id = 1;  // Empty = current mapping goal.
}
message StopMappingResponse {
  automsgs.rpcs.common.Status status = 1;
  string map_id = 2;  // Draft/saved id if available.
}
message GetMappingStatusRequest {}
message GetMappingStatusResponse {
  automsgs.rpcs.common.Status status = 1;
  MappingState state = 2;
  string goal_id = 3;
  string map_id = 4;
  bool active = 5;
  string message = 6;
}
```

CRUD request/response bodies: copy from current `world.proto` map section; change package to `automsgs.rpcs.mapping`; import `common.proto` + map msgs + header.

- [ ] **Step 2: Write `localization.proto`**

```protobuf
package automsgs.rpcs.localization;

service LocalizationService {
  rpc GetPose(GetPoseRequest) returns (GetPoseResponse);
  rpc GetStatus(GetStatusRequest) returns (GetStatusResponse);
  rpc SetInitialPose(SetInitialPoseRequest) returns (SetInitialPoseResponse);
}

message GetPoseRequest {
  string map_frame_id = 1;
}
message GetPoseResponse {
  automsgs.rpcs.common.Status status = 1;
  automsgs.msgs.std_msgs.Header header = 2;
  automsgs.msgs.geometry_msgs.Pose pose = 3;
  // Optional; leave empty if unknown.
  repeated double covariance = 4;  // 6x6 row-major if set (length 0 or 36).
}

enum LocState {
  LOC_STATE_UNSPECIFIED = 0;
  LOC_STATE_LOCALIZED = 1;
  LOC_STATE_LOST = 2;
  LOC_STATE_INITIALIZING = 3;
}

message GetStatusRequest {}
message GetStatusResponse {
  automsgs.rpcs.common.Status status = 1;
  LocState state = 2;
  string map_id = 3;
  string detail = 4;
}

message SetInitialPoseRequest {
  automsgs.msgs.std_msgs.Header header = 1;
  automsgs.msgs.geometry_msgs.Pose pose = 2;
  string map_id = 3;
}
message SetInitialPoseResponse {
  automsgs.rpcs.common.Status status = 1;
}
```

Imports: `header.proto`, `pose.proto`, `common.proto`.

- [ ] **Step 3: Delete `world.proto`**

- [ ] **Step 4: Verify**

```bash
test -f automsgs/proto/rpcs/mapping.proto
test -f automsgs/proto/rpcs/localization.proto
test ! -f automsgs/proto/rpcs/world.proto
grep -n "StartMapping\|SetCurrentMap" automsgs/proto/rpcs/mapping.proto
grep -n "SetInitialPose\|LOC_STATE_LOST" automsgs/proto/rpcs/localization.proto
```

- [ ] **Step 5: Commit (only if user asked)**

```bash
git commit -m "feat(automsgs): split world RPC into mapping and localization"
```

---

### Task 4: Polish `navigation.proto` + `sensor.proto`

**Files:**
- Modify: `automsgs/proto/rpcs/navigation.proto`
- Modify: `automsgs/proto/rpcs/sensor.proto`

- [ ] **Step 1: Navigation**

- Import `common.proto` (already done in Task 1 if fixed).  
- Rename `service NavService` → `service NavigationService`.  
- Keep `GoTo` / `Cancel` / `GetStatus` and `NavigationState` as-is.

- [ ] **Step 2: Sensor**

- Rename `rpc List` → `rpc ListSensors` (keep request/response type names `ListSensorsRequest` / `ListSensorsResponse` — already named that way; only RPC method was `List`).  
- Rename `rpc GetGrid` → `rpc GetOccupancyGrid`; rename messages `GetGridRequest`→`GetOccupancyGridRequest`, `GetGridResponse`→`GetOccupancyGridResponse`.

- [ ] **Step 3: Verify**

```bash
grep -n "service NavigationService" automsgs/proto/rpcs/navigation.proto
grep -n "rpc ListSensors\|rpc GetOccupancyGrid" automsgs/proto/rpcs/sensor.proto
grep -n "rpc List\|rpc GetGrid\|NavService" automsgs/proto/rpcs/*.proto && exit 1 || true
```

- [ ] **Step 4: Commit (only if user asked)**

```bash
git commit -m "refactor(automsgs): rename NavigationService and sensor RPC verbs"
```

---

### Task 5: `follow.proto`

**Files:**
- Create: `automsgs/proto/rpcs/follow.proto`

- [ ] **Step 1: Write `follow.proto`**

```protobuf
syntax = "proto3";
package automsgs.rpcs.follow;

import "automsgs/msgs/std_msgs/header.proto";
import "automsgs/rpcs/common.proto";

service FollowService {
  rpc StartFollow(StartFollowRequest) returns (StartFollowResponse);
  rpc Cancel(CancelRequest) returns (CancelResponse);
  rpc GetStatus(GetStatusRequest) returns (GetStatusResponse);
}

message StartFollowRequest {
  automsgs.msgs.std_msgs.Header header = 1;
  string target_id = 2;  // Empty = nearest person.
  string goal_id = 3;
}
message StartFollowResponse {
  automsgs.rpcs.common.Status status = 1;
  string goal_id = 2;
}
message CancelRequest {
  string goal_id = 1;
}
message CancelResponse {
  automsgs.rpcs.common.Status status = 1;
}

enum FollowState {
  FOLLOW_STATE_UNSPECIFIED = 0;
  FOLLOW_STATE_IDLE = 1;
  FOLLOW_STATE_FOLLOWING = 2;
  FOLLOW_STATE_LOST_TARGET = 3;
  FOLLOW_STATE_FAILED = 4;
  FOLLOW_STATE_CANCELLED = 5;
}

message GetStatusRequest {}
message GetStatusResponse {
  automsgs.rpcs.common.Status status = 1;
  FollowState state = 2;
  string goal_id = 3;
  string target_id = 4;
  bool active = 5;
  string message = 6;
}
```

- [ ] **Step 2: Verify**

```bash
grep -n "service FollowService\|FOLLOW_STATE_FOLLOWING" automsgs/proto/rpcs/follow.proto
```

- [ ] **Step 3: Commit (only if user asked)**

```bash
git commit -m "feat(automsgs): add FollowService RPC"
```

---

### Task 6: `system.proto`; delete `vehicle.proto` + `task.proto`

**Files:**
- Write: `automsgs/proto/rpcs/system.proto`
- Delete: `automsgs/proto/rpcs/vehicle.proto`, `automsgs/proto/rpcs/task.proto`

- [ ] **Step 1: Write `system.proto`**

```protobuf
syntax = "proto3";
package automsgs.rpcs.system;

import "automsgs/msgs/std_msgs/header.proto";
import "automsgs/msgs/sensor_msgs/battery_state.proto";
import "automsgs/rpcs/common.proto";

service SystemService {
  rpc Ping(PingRequest) returns (PingResponse);
  rpc Heartbeat(HeartbeatRequest) returns (HeartbeatResponse);
  rpc GetInfo(GetInfoRequest) returns (GetInfoResponse);
  rpc GetStatus(GetStatusRequest) returns (GetStatusResponse);
  rpc RunTask(RunTaskRequest) returns (RunTaskResponse);
  rpc CancelTask(CancelTaskRequest) returns (CancelTaskResponse);
  rpc GetTaskStatus(GetTaskStatusRequest) returns (GetTaskStatusResponse);
}

message PingRequest {}
message PingResponse {
  automsgs.rpcs.common.Status status = 1;
}

message HeartbeatRequest {
  uint64 sequence = 1;
}
message HeartbeatResponse {
  automsgs.rpcs.common.Status status = 1;
  uint64 sequence = 2;
  int64 robot_time_ns = 3;
}

message GetInfoRequest {}
message GetInfoResponse {
  automsgs.rpcs.common.Status status = 1;
  string model = 2;
  string serial_number = 3;
  string firmware_version = 4;
  string software_version = 5;
  string hostname = 6;
}

enum SystemState {
  SYSTEM_STATE_UNSPECIFIED = 0;
  SYSTEM_STATE_IDLE = 1;
  SYSTEM_STATE_BUSY = 2;
  SYSTEM_STATE_ERROR = 3;
  SYSTEM_STATE_ESTOP = 4;
}

message GetStatusRequest {}
message GetStatusResponse {
  automsgs.rpcs.common.Status status = 1;
  SystemState state = 2;
  string detail = 3;
  automsgs.msgs.sensor_msgs.BatteryState battery = 4;
}

message RunTaskRequest {
  automsgs.msgs.std_msgs.Header header = 1;
  string task_id = 2;
  string task_args = 3;
}
message RunTaskResponse {
  automsgs.rpcs.common.Status status = 1;
  string run_id = 2;
}

message CancelTaskRequest {
  string run_id = 1;
}
message CancelTaskResponse {
  automsgs.rpcs.common.Status status = 1;
}

enum TaskRunState {
  TASK_RUN_STATE_UNSPECIFIED = 0;
  TASK_RUN_STATE_PENDING = 1;
  TASK_RUN_STATE_RUNNING = 2;
  TASK_RUN_STATE_SUCCEEDED = 3;
  TASK_RUN_STATE_FAILED = 4;
  TASK_RUN_STATE_CANCELLED = 5;
}

message GetTaskStatusRequest {
  string run_id = 1;
}
message GetTaskStatusResponse {
  automsgs.rpcs.common.Status status = 1;
  TaskRunState state = 2;
  string run_id = 3;
  string progress = 4;
}
```

- [ ] **Step 2: Delete `vehicle.proto` and `task.proto`**

- [ ] **Step 3: Verify**

```bash
test -f automsgs/proto/rpcs/system.proto
test ! -f automsgs/proto/rpcs/vehicle.proto
test ! -f automsgs/proto/rpcs/task.proto
grep -n "Heartbeat\|RunTask\|BatteryState\|SYSTEM_STATE_ESTOP" automsgs/proto/rpcs/system.proto
ls automsgs/proto/rpcs/*.proto | sort
```

Expected listing:

```text
common.proto
docking.proto
follow.proto
localization.proto
mapping.proto
navigation.proto
sensor.proto
system.proto
```

(+ README.md)

- [ ] **Step 4: Commit (only if user asked)**

```bash
git commit -m "feat(automsgs): add SystemService and remove vehicle/task RPCs"
```

---

### Task 7: Rewrite `rpcs/README.md`

**Files:**
- Modify: `automsgs/proto/rpcs/README.md`

- [ ] **Step 1: Replace inventory table** with the eight files/services from the overview spec; document import `automsgs/rpcs/common.proto`; update Code range table (add follow/system/mapping-process; note docking uses 500–599 `CODE_CHARGING_*`); remove references to world/vehicle/charging/task/status filenames; note `NavigationService` (not `NavService`).

- [ ] **Step 2: Verify**

```bash
grep -n "DockingService\|MappingService\|LocalizationService\|FollowService\|SystemService\|common.proto" automsgs/proto/rpcs/README.md
grep -nE "status\.proto|charging\.proto|world\.proto|VehicleService|ChargeService" automsgs/proto/rpcs/README.md && exit 1 || true
```

- [ ] **Step 3: Final tree check**

```bash
ls automsgs/proto/rpcs/
grep -R "status\.proto\|charging\.proto\|world\.proto\|package automsgs.rpcs.vehicle\|package automsgs.rpcs.task\|package automsgs.rpcs.charging\|package automsgs.rpcs.world" automsgs/proto/rpcs && exit 1 || true
```

- [ ] **Step 4: Commit (only if user asked)**

```bash
git commit -m "docs(automsgs): update rpcs README for domain API layout"
```

---

## Spec coverage self-check

| Spec requirement | Task |
|------------------|------|
| `common.proto` + Code ranges | 1 |
| Docking async API | 2 |
| Mapping CRUD + Start/Stop | 3 |
| Localization GetPose/Status/SetInitialPose | 3 |
| NavigationService rename | 4 |
| Sensor ListSensors / GetOccupancyGrid | 4 |
| FollowService | 5 |
| System ping/heartbeat/info/status/task | 6 |
| Delete old protos | 2,3,6 |
| README sync | 7 |

## Placeholder scan

No TBD/TODO left in task bodies; proto snippets are complete enough to paste and adjust copyright headers.

---

Plan complete and saved to `docs/superpowers/plans/2026-08-14-automsgs-rpcs-overview.md`.

**执行方式二选一：**

1. **Subagent-Driven（推荐）** — 每任务新开子代理，任务间复查  
2. **Inline Execution** — 本会话按 `executing-plans` 连续执行并设检查点  

选哪种？
