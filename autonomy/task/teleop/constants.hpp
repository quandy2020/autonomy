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

namespace autonomy {
namespace task {
namespace teleop {

// BT blackboard key for TeleopClient shared pointer.
constexpr char kTeleopClientBlackboardKey[] = "teleop_client";

// Published body velocity command topic.
constexpr char kCommandVelocityTopic[] = "/cmd_vel";

// Planning / visualization / costmap frame (always base_link for teleop).
constexpr char kDefaultBaseFrame[] = "base_link";

// Selected follow path (nav_msgs/Path), aligned with CMU local_planner /path.
constexpr char kTeleopPathTopic[] = "/autonomy/task/teleop/path";

// Feasible candidate paths as LINE_STRIP markers (CMU local_planner style).
constexpr char kTeleopFreePathMarkersTopic[] =
    "/autonomy/task/teleop/free_path_markers";

// Legacy PointCloud2 topic (deprecated; use free_path_markers).
constexpr char kTeleopFreePathsTopic[] = "/autonomy/task/teleop/free_paths";

// Rolling local costmap for Autoviz (OccupancyGrid, frame=base_link).
constexpr char kTeleopLocalCostmapTopic[] =
    "/autonomy/task/teleop/local_costmap";

// Default max linear speed when TeleopGoal omits the field (m/s).
constexpr double kDefaultMaxLinearSpeed = 0.5;
// Default max angular speed when TeleopGoal omits the field (rad/s).
constexpr double kDefaultMaxAngularSpeed = 1.0;
// Default command watchdog timeout (seconds).
constexpr double kDefaultWatchdogTimeoutSec = 0.5;

}  // namespace teleop
}  // namespace task
}  // namespace autonomy
