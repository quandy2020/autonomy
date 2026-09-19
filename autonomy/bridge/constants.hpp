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

/**
 * @file constants.hpp
 * @brief Bridge Autolink channel / action name constants.
 *
 * @details
 * Must stay string-aligned with TaskServer publishers (automsgs.task.* wire).
 * Bridge must not include autonomy/task headers — only automsgs protos.
 *
 * @note Legacy `*Channel` aliases equal the GoalChannel topic constants.
 */

#pragma once

namespace autonomy {
namespace bridge {

// --- GoalChannel topics (automsgs.task.* Goal / Feedback) ---

/** @brief Navigation GoalChannel goal topic. */
constexpr const char* kNavigationGoal = "/autonomy/task/navigation/goal";
/** @brief Navigation GoalChannel feedback topic. */
constexpr const char* kNavigationFeedback =
    "/autonomy/task/navigation/feedback";

/** @brief Teleop GoalChannel goal topic (velocity session). */
constexpr const char* kTeleopGoal = "/autonomy/task/teleop/goal";
/** @brief Teleop GoalChannel feedback topic. */
constexpr const char* kTeleopFeedback = "/autonomy/task/teleop/feedback";

/** @brief Tracking / Follow GoalChannel goal topic. */
constexpr const char* kTrackingGoal = "/autonomy/task/tracking/goal";
/** @brief Tracking / Follow GoalChannel feedback topic. */
constexpr const char* kTrackingFeedback = "/autonomy/task/tracking/feedback";

/** @brief Charging GoalChannel goal topic. */
constexpr const char* kChargingGoal = "/autonomy/task/charging/goal";
/** @brief Charging GoalChannel feedback topic. */
constexpr const char* kChargingFeedback = "/autonomy/task/charging/feedback";

/** @brief Voice GoalChannel goal topic. */
constexpr const char* kVoiceGoal = "/autonomy/task/voice/goal";
/** @brief Voice GoalChannel feedback topic. */
constexpr const char* kVoiceFeedback = "/autonomy/task/voice/feedback";

/** @brief Mapping GoalChannel goal topic. */
constexpr const char* kMappingGoal = "/autonomy/task/mapping/goal";
/** @brief Mapping GoalChannel feedback topic. */
constexpr const char* kMappingFeedback = "/autonomy/task/mapping/feedback";

/** @brief Localization GoalChannel goal topic (SetInitialPose). */
constexpr const char* kLocalizationGoal = "/autonomy/task/localization/goal";
/** @brief Localization GoalChannel feedback topic. */
constexpr const char* kLocalizationFeedback =
    "/autonomy/task/localization/feedback";

/** @brief Exploration GoalChannel goal topic. */
constexpr const char* kExplorationGoal = "/autonomy/task/exploration/goal";
/** @brief Exploration GoalChannel feedback topic. */
constexpr const char* kExplorationFeedback =
    "/autonomy/task/exploration/feedback";

// --- Legacy aliases ---

/** @brief Legacy alias for @ref kTeleopGoal. */
constexpr const char* kTeleopGoalChannel = kTeleopGoal;
/** @brief Legacy alias for @ref kTeleopFeedback. */
constexpr const char* kTeleopFeedbackChannel = kTeleopFeedback;
/** @brief Legacy alias for @ref kTrackingGoal. */
constexpr const char* kTrackingGoalChannel = kTrackingGoal;
/** @brief Legacy alias for @ref kTrackingFeedback. */
constexpr const char* kTrackingFeedbackChannel = kTrackingFeedback;
/** @brief Legacy alias for @ref kChargingGoal. */
constexpr const char* kChargingGoalChannel = kChargingGoal;
/** @brief Legacy alias for @ref kChargingFeedback. */
constexpr const char* kChargingFeedbackChannel = kChargingFeedback;
/** @brief Legacy alias for @ref kMappingGoal. */
constexpr const char* kMappingGoalChannel = kMappingGoal;
/** @brief Legacy alias for @ref kMappingFeedback. */
constexpr const char* kMappingFeedbackChannel = kMappingFeedback;

/** @brief Exploration waypoint Autolink topic (non-GoalChannel). */
constexpr const char* kExplorationWaypointChannel = "/exploration/waypoint";
/** @brief Exploration finished Autolink topic (non-GoalChannel). */
constexpr const char* kExplorationFinishedChannel = "/exploration/finished";

// --- Action / topic names used by Bridge clients ---

/** @brief NavigateToPose Action server name. */
constexpr const char* kNavigateToPoseActionServerName = "/navigate_to_pose";
/** @brief NavigateThroughPoses Action server name. */
constexpr const char* kNavigateThroughPosesActionServerName =
    "/navigate_through_poses";

/** @brief Live robot-state Autolink topic (`/robot_state`). */
constexpr const char* kRobotStateChannel = "/robot_state";
/** @brief Robot event Autolink topic (`/robot_event`). */
constexpr const char* kRobotEventChannel = "/robot_event";
/** @brief Relative Spin Action server name. */
constexpr const char* kSpinActionName = "/spin";
/** @brief Relative BackUp Action server name. */
constexpr const char* kBackUpActionName = "/backup";
/** @brief Relative DriveOnHeading Action server name. */
constexpr const char* kDriveOnHeadingActionName = "/drive_on_heading";
/** @brief Live occupancy-grid Autolink topic (`/map`). */
constexpr const char* kMapChannel = "/map";

}  // namespace bridge
}  // namespace autonomy
