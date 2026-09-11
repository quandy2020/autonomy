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

#include "autonomy/task/common/names.hpp"

namespace autonomy {
namespace bridge {

constexpr const char* kNavigateToPoseActionServerName =
    task::kNavigateToPose;
constexpr const char* kNavigateThroughPosesActionServerName =
    task::kNavigateThroughPoses;

constexpr const char* kTeleopGoalChannel = task::kTeleopGoal;
constexpr const char* kTeleopFeedbackChannel = task::kTeleopFeedback;
constexpr const char* kTrackingGoalChannel = task::kTrackingGoal;
constexpr const char* kTrackingFeedbackChannel = task::kTrackingFeedback;
constexpr const char* kChargingGoalChannel = task::kChargingGoal;
constexpr const char* kChargingFeedbackChannel = task::kChargingFeedback;
constexpr const char* kMappingGoalChannel = task::kMappingGoal;
constexpr const char* kMappingFeedbackChannel = task::kMappingFeedback;
constexpr const char* kExplorationWaypointChannel = task::kExplorationWaypoint;
constexpr const char* kExplorationFinishedChannel = task::kExplorationFinished;

constexpr const char* kRobotStateChannel = "/robot_state";
constexpr const char* kRobotEventChannel = "/robot_event";
constexpr const char* kSpinActionName = "/spin";
constexpr const char* kBackUpActionName = "/backup";
constexpr const char* kDriveOnHeadingActionName = "/drive_on_heading";
constexpr const char* kMapChannel = "/map";

}  // namespace bridge
}  // namespace autonomy
