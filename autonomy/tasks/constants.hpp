/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <string>
#include <vector>

namespace autonomy {
namespace tasks {

constexpr char kTaskNodeName[] = "task_manager";

/** @brief Registered BT navigator ids (NavigatorRegistry / tasks.lua). */
constexpr char kNavigatorNavigateToPose[] = "navigate_to_pose";
constexpr char kNavigatorNavigateThroughPoses[] = "navigate_through_poses";
constexpr char kNavigatorNavigateToDocking[] = "navigate_to_docking";
constexpr char kNavigatorTrackToTarget[] = "track_to_target";
constexpr char kNavigatorExploreToAnywhere[] = "explore_to_anywhere";
constexpr char kNavigatorTeleopDrive[] = "teleop_drive";

inline constexpr std::size_t kMinPathPoses = 2;
inline constexpr double kDirectNavDefaultTimeoutSec = 300.0;
inline constexpr int kBtWaitPollMs = 50;
inline constexpr int kSpinRateHz = 10;

}  // namespace tasks
}  // namespace autonomy