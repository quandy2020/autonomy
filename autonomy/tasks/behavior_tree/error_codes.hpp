/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <cstdint>

#include "autonomy/tasks/proto/error_code.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/** BT blackboard error_code_id values (mirrors BehaviorTreeErrorCode in error_code.proto). */
struct BtErrorCode {
    static constexpr uint16_t NONE =
        static_cast<uint16_t>(proto::BT_NONE);
    static constexpr uint16_t UNKNOWN =
        static_cast<uint16_t>(proto::BT_UNKNOWN);
    static constexpr uint16_t TIMEOUT =
        static_cast<uint16_t>(proto::BT_TIMEOUT);
    static constexpr uint16_t CONTROLLER_TIMED_OUT =
        static_cast<uint16_t>(proto::BT_CONTROLLER_TIMED_OUT);
    static constexpr uint16_t PLANNER_FAILED =
        static_cast<uint16_t>(proto::BT_PLANNER_FAILED);
    static constexpr uint16_t CONTROLLER_FAILED =
        static_cast<uint16_t>(proto::BT_CONTROLLER_FAILED);
    static constexpr uint16_t SMOOTHER_FAILED =
        static_cast<uint16_t>(proto::BT_SMOOTHER_FAILED);
    static constexpr uint16_t TF_ERROR =
        static_cast<uint16_t>(proto::BT_TF_ERROR);
    static constexpr uint16_t INVALID_PATH =
        static_cast<uint16_t>(proto::BT_INVALID_PATH);
    static constexpr uint16_t GOAL_CHECKER_FAILED =
        static_cast<uint16_t>(proto::BT_GOAL_CHECKER_FAILED);
    static constexpr uint16_t SERVICE_UNAVAILABLE =
        static_cast<uint16_t>(proto::BT_SERVICE_UNAVAILABLE);
    static constexpr uint16_t SERVER_TIMEOUT =
        static_cast<uint16_t>(proto::BT_SERVER_TIMEOUT);
    static constexpr uint16_t PATH_INVALID =
        static_cast<uint16_t>(proto::BT_PATH_INVALID);
    static constexpr uint16_t COSTMAP_ERROR =
        static_cast<uint16_t>(proto::BT_COSTMAP_ERROR);
    static constexpr uint16_t TRANSFORM_UNAVAILABLE =
        static_cast<uint16_t>(proto::BT_TRANSFORM_UNAVAILABLE);
    static constexpr uint16_t CANCELED =
        static_cast<uint16_t>(proto::BT_CANCELED);
    static constexpr uint16_t PREEMPTED =
        static_cast<uint16_t>(proto::BT_PREEMPTED);
    static constexpr uint16_t NOT_INITIALIZED =
        static_cast<uint16_t>(proto::BT_NOT_INITIALIZED);
    static constexpr uint16_t INVALID_GOAL =
        static_cast<uint16_t>(proto::BT_INVALID_GOAL);
};

/** Map a BT pipeline error to navigate_to_pose action result code. */
proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    proto::BehaviorTreeErrorCode bt_error);

/** Map a BT pipeline error to navigate_through_poses action result code. */
proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    proto::BehaviorTreeErrorCode bt_error);

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
