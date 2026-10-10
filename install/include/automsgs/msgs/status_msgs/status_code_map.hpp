/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <automsgs/msgs/status_msgs/status_msgs.pb.h>

namespace automsgs {
namespace msgs {
namespace status_msgs {

/// True if |code| is already allowed on rpcs.common.Status (thin layer).
inline bool IsThinStatusCode(StatusCode code) {
  const int v = static_cast<int>(code);
  if (v >= 0 && v <= 999) {
    return true;
  }
  if (v >= 1100 && v <= 1399) {
    return true;
  }
  if (v >= 2000 && v <= 2099) {
    return true;
  }
  return false;
}

/// Map internal fine StatusPb codes to thin RPC Status codes.
/// Thin codes pass through unchanged. Unknown fine ranges → INTERNAL.
inline StatusCode ToThinStatusCode(StatusCode code) {
  if (IsThinStatusCode(code)) {
    return code;
  }

  switch (code) {
    // --- control (1000) → navigation / system ---
    case CONTROL_CANCELLED:
      return NAVIGATION_CANCELLED;
    case CONTROL_INVALID_PATH:
    case CONTROL_NO_VALID_CMD:
    case CONTROL_MISSED_PATH:
    case CONTROL_MISSED_GOAL:
      return NAVIGATION_NO_PATH;
    case CONTROL_COLLISION:
    case CONTROL_OSCILLATION:
    case CONTROL_ROBOT_STUCK:
    case CONTROL_BLOCKED_GOAL:
    case CONTROL_BLOCKED_PATH:
    case CONTROL_FAILED_TO_MAKE_PROGRESS:
      return NAVIGATION_PATH_BLOCKED;
    case CONTROL_PATIENCE_EXCEEDED:
    case CONTROL_CONTROLLER_TIMED_OUT:
      return NAVIGATION_TIMEOUT;
    case CONTROL_ESTOP_ERROR:
      return SYSTEM_ESTOP;
    case CONTROL_NOT_INITIALIZED:
    case CONTROL_INIT_ERROR:
      return UNAVAILABLE;
    case CONTROL_ERROR:
    case CONTROL_COMPUTE_ERROR:
    case CONTROL_TF_ERROR:
    case CONTROL_INVALID_PLUGIN:
    case CONTROL_INTERNAL_ERROR:
    case CONTROL_STOPPED:
    case CONTROL_UNKNOWN:
      return INTERNAL;

    // --- localization (3000) ---
    case LOCALIZATION_NOT_READY:
      return LOCALIZATION_UNAVAILABLE;
    case LOCALIZATION_ERROR:
    case LOCALIZATION_TF_ERROR:
      return LOCALIZATION_LOST;

    // --- recovery (4000) → navigation ---
    case RECOVERY_CANCELLED:
      return NAVIGATION_CANCELLED;
    case RECOVERY_TIMEOUT:
      return NAVIGATION_TIMEOUT;
    case RECOVERY_IMPASSABLE:
      return NAVIGATION_PATH_BLOCKED;
    case RECOVERY_ERROR:
    case RECOVERY_TF_ERROR:
    case RECOVERY_NOT_INITIALIZED:
    case RECOVERY_INVALID_PLUGIN:
    case RECOVERY_INTERNAL_ERROR:
    case RECOVERY_STOPPED:
      return INTERNAL;

    // --- recovery primitives (4500) → teleop ---
    case SPIN_TIMEOUT:
    case BACK_UP_TIMEOUT:
    case DRIVE_ON_HEADING_TIMEOUT:
    case ASSISTED_TELEOP_TIMEOUT:
    case WAIT_TIMEOUT:
      return TELEOP_TIMEOUT;
    case SPIN_COLLISION_AHEAD:
    case BACK_UP_COLLISION_AHEAD:
    case DRIVE_ON_HEADING_COLLISION_AHEAD:
      return TELEOP_COLLISION;
    case BACK_UP_INVALID_INPUT:
    case DRIVE_ON_HEADING_INVALID_INPUT:
      return INVALID_ARGUMENT;
    case SPIN_ERROR:
    case SPIN_TF_ERROR:
    case BACK_UP_ERROR:
    case BACK_UP_TF_ERROR:
    case DRIVE_ON_HEADING_ERROR:
    case DRIVE_ON_HEADING_TF_ERROR:
    case ASSISTED_TELEOP_ERROR:
    case ASSISTED_TELEOP_TF_ERROR:
    case WAIT_ERROR:
      return TELEOP_REJECTED;

    // --- planning (6000) → navigation ---
    case PLANNING_CANCELLED:
      return NAVIGATION_CANCELLED;
    case PLANNING_TIMEOUT:
      return NAVIGATION_TIMEOUT;
    case PLANNING_NO_PATH_FOUND:
    case PLANNING_EMPTY_PATH:
    case PLANNING_NO_WAYPOINTS:
      return NAVIGATION_NO_PATH;
    case PLANNING_INVALID_START:
    case PLANNING_INVALID_GOAL:
    case PLANNING_BLOCKED_START:
    case PLANNING_BLOCKED_GOAL:
    case PLANNING_START_OUTSIDE_MAP:
    case PLANNING_GOAL_OUTSIDE_MAP:
    case PLANNING_START_OCCUPIED:
    case PLANNING_GOAL_OCCUPIED:
      return NAVIGATION_GOAL_REJECTED;
    case PLANNING_NOT_READY:
    case PLANNING_NOT_INITIALIZED:
      return UNAVAILABLE;
    case PLANNING_ERROR:
    case PLANNING_TF_ERROR:
    case PLANNING_INVALID_PLUGIN:
    case PLANNING_INTERNAL_ERROR:
    case PLANNING_UNKNOWN:
      return INTERNAL;

    // --- smoother (6200) → navigation ---
    case SMOOTHER_TIMEOUT:
      return NAVIGATION_TIMEOUT;
    case SMOOTHER_PATH_IN_COLLISION:
    case SMOOTHER_INVALID_PATH:
    case SMOOTHER_FAILED:
      return NAVIGATION_NO_PATH;
    case SMOOTHER_ERROR:
    case SMOOTHER_INVALID_PLUGIN:
    case SMOOTHER_UNKNOWN:
      return INTERNAL;

    // --- map / costmap (7000) ---
    case MAP_NOT_READY:
      return UNAVAILABLE;
    case MAP_OUT_OF_MAP:
    case MAP_DATA_ERROR:
      return MAP_INVALID;
    case MAP_ERROR:
      return MAP_LOAD_FAILED;

    // --- routing (8000) → navigation ---
    case ROUTING_NOT_READY:
      return UNAVAILABLE;
    case ROUTING_PATH_INVALID:
      return NAVIGATION_NO_PATH;
    case ROUTING_ERROR:
    case ROUTING_REQUEST_ERROR:
    case ROUTING_RESPONSE_ERROR:
      return INTERNAL;

    // --- BT task (9000) → task surface / navigation ---
    case TASK_BT_CANCELLED:
    case TASK_PREEMPTED:
      return TASK_CANCELLED;
    case TASK_TIMEOUT:
      return DEADLINE_EXCEEDED;
    case TASK_NO_VALID_PATH:
    case TASK_PATH_INVALID:
    case TASK_PLANNER_FAILED:
    case TASK_SMOOTHER_FAILED:
      return NAVIGATION_NO_PATH;
    case TASK_CONTROLLER_FAILED:
    case TASK_GOAL_CHECKER_FAILED:
      return NAVIGATION_PATH_BLOCKED;
    case TASK_INVALID_GOAL:
      return NAVIGATION_GOAL_REJECTED;
    case TASK_NOT_INITIALIZED:
      return UNAVAILABLE;
    case TASK_ERROR:
    case TASK_BT_LOAD_FAILED:
    case TASK_TF_ERROR:
    case TASK_UNKNOWN:
      return TASK_FAILED;

    // --- waypoints (9200) → navigation ---
    case WAYPOINTS_NO_VALID_WAYPOINTS:
    case WAYPOINTS_STOP_ON_MISSED:
      return NAVIGATION_NO_PATH;
    case WAYPOINTS_ERROR:
    case WAYPOINTS_TASK_EXECUTOR_FAILED:
    case WAYPOINTS_UNKNOWN:
      return INTERNAL;

    default:
      return INTERNAL;
  }
}

}  // namespace status_msgs
}  // namespace msgs
}  // namespace automsgs
