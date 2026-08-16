# NavigationService Server-Streaming RPC Implementation Plan

> **For agentic workers:** Use subagent-driven-development or executing-plans. Checkbox steps for tracking.

**Goal:** Upgrade `NavigationService` so `GoTo` is session-style server streaming; Cancel/GetStatus stay unary.

**Architecture:** Align with Follow session stream: keep open after `ARRIVED` until Cancel/FAILED/disconnect. Add `NavigationPhase` and `NavigateResponse`. No git unless user asks.

**Spec:** `docs/superpowers/specs/2026-08-14-navigation-rpc-streaming-design.md`

**Commits:** Do not git commit unless user asks.

---

### Task 1: Rewrite `navigation.proto`

Keep copyright. Body:

```protobuf
syntax = "proto3";

package automsgs.rpcs.navigation;

import "automsgs/msgs/std_msgs/header.proto";
import "automsgs/msgs/geometry_msgs/pose.proto";
import "automsgs/rpcs/common.proto";

// Point-to-pose navigation RPCs.
// GoTo: server streaming for the navigation session.
//   ARRIVED keeps the stream open (dwell feedback) until Cancel, FAILED, or client disconnect.
// Cancel / GetStatus: unary. No bidirectional streaming.
service NavigationService {
  rpc GoTo(GoToRequest) returns (stream NavigateResponse);
  rpc Cancel(CancelRequest) returns (CancelResponse);
  rpc GetStatus(GetStatusRequest) returns (GetStatusResponse);
}

message GoToRequest {
  automsgs.msgs.std_msgs.Header header = 1;
  automsgs.msgs.geometry_msgs.Pose target_pose = 2;
  // Optional client-defined id for tracking.
  string goal_id = 3;
}

message CancelRequest {
  // Empty = cancel the current active navigation goal.
  string goal_id = 1;
}

message CancelResponse {
  automsgs.rpcs.common.Status status = 1;
}

// Coarse navigation lifecycle.
enum NavigationState {
  NAVIGATION_STATE_UNSPECIFIED = 0;
  NAVIGATION_STATE_IDLE = 1;
  NAVIGATION_STATE_NAVIGATING = 2;
  NAVIGATION_STATE_ARRIVED = 3;
  NAVIGATION_STATE_FAILED = 4;
  NAVIGATION_STATE_CANCELLED = 5;
}

// Fine-grained phase inside NavigateResponse.
enum NavigationPhase {
  NAVIGATION_PHASE_UNSPECIFIED = 0;
  NAVIGATION_PHASE_IDLE = 1;
  NAVIGATION_PHASE_PLANNING = 2;
  NAVIGATION_PHASE_CONTROLLING = 3;
  NAVIGATION_PHASE_RECOVERING = 4;
  NAVIGATION_PHASE_ARRIVED = 5;
  NAVIGATION_PHASE_FAILED = 6;
  NAVIGATION_PHASE_CANCELLED = 7;
}

// Server-stream feedback for GoTo.
message NavigateResponse {
  automsgs.rpcs.common.Status status = 1;
  NavigationState state = 2;
  NavigationPhase phase = 3;
  string goal_id = 4;
  // Remaining path distance in meters; unknown may be 0.
  float remaining_distance_m = 5;
  // True only while state is NAVIGATING.
  bool active = 6;
  string message = 7;
}

message GetStatusRequest {}

message GetStatusResponse {
  automsgs.rpcs.common.Status status = 1;
  NavigationState state = 2;
  NavigationPhase phase = 3;
  string goal_id = 4;
  float remaining_distance_m = 5;
  bool active = 6;
  string message = 7;
}
```

Verify: stream GoTo; no GoToResponse; NavigationPhase; remaining_distance_m; no request-side stream.

### Task 2: README

- Row: Navigation = GoTo server streaming；Cancel/GetStatus unary  
- Long-task default list: remove GoTo from unary accept list (only StartMapping etc.)  
- Add ### Navigation（流式例外） after Follow (or before Docking): ARRIVED 不关流；BUSY→GOAL_REJECTED 100；Cancel→103

### Task 3: Acceptance greps + no commit
