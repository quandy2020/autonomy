/*
 * Copyright 2026 The Openbot Authors
 *
 * Thin aliases over autonomy.manipulation.proto.ErrorCode (internal).
 */

#pragma once

#include "autonomy/manipulation/proto/error_codes.pb.h"

namespace autonomy {
namespace manipulation {

/**
 * @brief Internal manipulation result code (planning / IK / execution).
 *
 * Defined in autonomy/manipulation/proto/error_codes.proto.
 * Not for external RPC Status — use automsgs.msgs.status_msgs.StatusCode.
 */
using ErrorCode = ::autonomy::manipulation::proto::ErrorCode;

/**
 * @brief Human-readable name for a subset of @ref ErrorCode values.
 */
inline const char* ErrorCodeName(ErrorCode code) {
  switch (code) {
    case ErrorCode::SUCCESS:
      return "SUCCESS";
    case ErrorCode::PLANNING_FAILED:
      return "PLANNING_FAILED";
    case ErrorCode::INVALID_MOTION_PLAN:
      return "INVALID_MOTION_PLAN";
    case ErrorCode::CONTROL_FAILED:
      return "CONTROL_FAILED";
    case ErrorCode::TIMED_OUT:
      return "TIMEOUT";
    case ErrorCode::PREEMPTED:
      return "PREEMPTED";
    case ErrorCode::NO_INVERSE_KINEMATICS_SOLUTION:
      return "NO_INVERSE_KINEMATICS_SOLUTION";
    case ErrorCode::INVALID_GROUP_NAME:
      return "INVALID_GROUP_NAME";
    case ErrorCode::START_STATE_IN_COLLISION:
      return "START_STATE_IN_COLLISION";
    case ErrorCode::GOAL_IN_COLLISION:
      return "GOAL_IN_COLLISION";
    case ErrorCode::FAILURE:
    default:
      return "FAILURE";
  }
}

/** @brief Whether @p code indicates success. */
inline bool IsSuccess(ErrorCode code) {
  return code == ErrorCode::SUCCESS;
}

}  // namespace manipulation
}  // namespace autonomy
