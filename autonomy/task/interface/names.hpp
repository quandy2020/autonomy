/*
 * Copyright 2026 The Openbot Authors
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

#ifndef AUTONOMY_TASK_INTERFACE_NAMES_HPP_
#define AUTONOMY_TASK_INTERFACE_NAMES_HPP_

namespace autonomy {
namespace task {

// External interface names (Nav2 BtNavigator surface + domain goal channels).
// Bridge / Autoviz must match these strings.

constexpr char kGoalPose[] = "/goal_pose";
constexpr char kNavigateToPose[] = "navigate_to_pose";
constexpr char kNavigateThroughPoses[] = "navigate_through_poses";

constexpr char kTeleopGoal[] = "/autonomy/task/teleop/goal";
constexpr char kTeleopFeedback[] = "/autonomy/task/teleop/feedback";    

constexpr char kTrackingGoal[] = "/autonomy/task/tracking/goal";
constexpr char kTrackingFeedback[] = "/autonomy/task/tracking/feedback";

constexpr char kExplorationGoal[] = "/autonomy/task/exploration/goal";
constexpr char kExplorationFeedback[] = "/autonomy/task/exploration/feedback";

constexpr char kChargingGoal[] = "/autonomy/task/charging/goal";
constexpr char kChargingFeedback[] = "/autonomy/task/charging/feedback";

constexpr char kMappingGoal[] = "/autonomy/task/mapping/goal";
constexpr char kMappingFeedback[] = "/autonomy/task/mapping/feedback";

constexpr char kLocalizationGoal[] = "/autonomy/task/localization/goal";
constexpr char kLocalizationFeedback[] =
    "/autonomy/task/localization/feedback";

}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_INTERFACE_NAMES_HPP_
