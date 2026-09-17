/*
 * Copyright 2026 The Openbot Authors
 *
 * MoveIt MoveItErrorCodes analogue (subset).
 * Numeric values align with automsgs.msgs.moveit_msgs.MoveItErrorCodes.Code.
 */

#pragma once

#include <string>

namespace autonomy {
namespace manipulation {

/**
 * @brief Manipulation / planning result codes (MoveIt-compatible subset).
 *
 * Numeric values align with `automsgs.msgs.moveit_msgs.MoveItErrorCodes.Code`.
 */
enum class ErrorCode {
  kSuccess = 1,
  kFailure = 99999,
  kPlanningFailed = -1,
  kInvalidMotionPlan = -2,
  kMotionPlanInvalidatedByEnvironmentChange = -3,
  kControlFailed = -4,
  kUnableToAquireSensorData = -5,
  kTimedOut = -6,
  kPreempted = -7,
  kStartStateInCollision = -10,
  kStartStateViolatesPathConstraints = -11,
  kGoalInCollision = -12,
  kGoalViolatesPathConstraints = -13,
  kGoalConstraintsBadlyDefined = -14,
  kInvalidGroupName = -15,
  kInvalidGoalConstraints = -16,
  kInvalidRobotState = -17,
  kInvalidLinkName = -18,
  kInvalidObjectName = -19,
  kFrameTransformFailure = -21,
  kCollisionCheckingUnavailable = -22,
  kRobotStateStale = -23,
  kSensorInfoStale = -24,
  kNoIkSolution = -31,
  kInvalidPlanningScene = -32,
};

/**
 * @brief Human-readable name for a subset of @ref ErrorCode values.
 * @param[in] code Error code to name.
 * @return Static C-string label; unknown codes map to `"FAILURE"`.
 */
inline const char* ErrorCodeName(ErrorCode code) {
  switch (code) {
    case ErrorCode::kSuccess:
      return "SUCCESS";
    case ErrorCode::kPlanningFailed:
      return "PLANNING_FAILED";
    case ErrorCode::kInvalidMotionPlan:
      return "INVALID_MOTION_PLAN";
    case ErrorCode::kControlFailed:
      return "CONTROL_FAILED";
    case ErrorCode::kTimedOut:
      return "TIMED_OUT";
    case ErrorCode::kPreempted:
      return "PREEMPTED";
    case ErrorCode::kNoIkSolution:
      return "NO_IK_SOLUTION";
    case ErrorCode::kInvalidGroupName:
      return "INVALID_GROUP_NAME";
    case ErrorCode::kStartStateInCollision:
      return "START_STATE_IN_COLLISION";
    case ErrorCode::kGoalInCollision:
      return "GOAL_IN_COLLISION";
    case ErrorCode::kFailure:
    default:
      return "FAILURE";
  }
}

/**
 * @brief Whether @p code indicates success.
 * @param[in] code Error code to test.
 * @return true if @p code is @ref ErrorCode::kSuccess.
 */
inline bool IsSuccess(ErrorCode code) {
  return code == ErrorCode::kSuccess;
}

}  // namespace manipulation
}  // namespace autonomy
