/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/error_codes.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace {

proto::NavigateToPoseErrorCode MapBehaviorTreeError(
    proto::BehaviorTreeErrorCode bt_error,
    proto::NavigateToPoseErrorCode planner_failed,
    proto::NavigateToPoseErrorCode controller_failed,
    proto::NavigateToPoseErrorCode smoother_failed) {
    switch (bt_error) {
        case proto::BT_NONE:
            return proto::NAV_TO_POSE_NONE;
        case proto::BT_TF_ERROR:
        case proto::BT_TRANSFORM_UNAVAILABLE:
            return proto::NAV_TO_POSE_TF_ERROR;
        case proto::BT_TIMEOUT:
        case proto::BT_CONTROLLER_TIMED_OUT:
        case proto::BT_SERVER_TIMEOUT:
            return proto::NAV_TO_POSE_TIMEOUT;
        case proto::BT_PLANNER_FAILED:
            return planner_failed;
        case proto::BT_CONTROLLER_FAILED:
        case proto::BT_GOAL_CHECKER_FAILED:
            return controller_failed;
        case proto::BT_SMOOTHER_FAILED:
            return smoother_failed;
        case proto::BT_INVALID_PATH:
        case proto::BT_PATH_INVALID:
            return proto::NAV_TO_POSE_PATH_INVALID;
        case proto::BT_CANCELED:
            return proto::NAV_TO_POSE_CANCELED;
        case proto::BT_PREEMPTED:
            return proto::NAV_TO_POSE_PREEMPTED;
        case proto::BT_INVALID_GOAL:
            return proto::NAV_TO_POSE_INVALID_GOAL;
        case proto::BT_NOT_INITIALIZED:
            return proto::NAV_TO_POSE_NOT_INITIALIZED;
        case proto::BT_SERVICE_UNAVAILABLE:
        case proto::BT_COSTMAP_ERROR:
        case proto::BT_UNKNOWN:
        default:
            return proto::NAV_TO_POSE_UNKNOWN;
    }
}

}  // namespace

proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    proto::BehaviorTreeErrorCode bt_error) {
    return MapBehaviorTreeError(
        bt_error, proto::NAV_TO_POSE_PLANNER_FAILED,
        proto::NAV_TO_POSE_CONTROLLER_FAILED,
        proto::NAV_TO_POSE_SMOOTHER_FAILED);
}

proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    proto::BehaviorTreeErrorCode bt_error) {
    switch (MapBehaviorTreeError(
        bt_error, proto::NAV_TO_POSE_PLANNER_FAILED,
        proto::NAV_TO_POSE_CONTROLLER_FAILED,
        proto::NAV_TO_POSE_SMOOTHER_FAILED)) {
        case proto::NAV_TO_POSE_NONE:
            return proto::NAV_THROUGH_POSES_NONE;
        case proto::NAV_TO_POSE_TF_ERROR:
            return proto::NAV_THROUGH_POSES_TF_ERROR;
        case proto::NAV_TO_POSE_TIMEOUT:
            return proto::NAV_THROUGH_POSES_TIMEOUT;
        case proto::NAV_TO_POSE_PLANNER_FAILED:
            return proto::NAV_THROUGH_POSES_PLANNER_FAILED;
        case proto::NAV_TO_POSE_CONTROLLER_FAILED:
            return proto::NAV_THROUGH_POSES_CONTROLLER_FAILED;
        case proto::NAV_TO_POSE_SMOOTHER_FAILED:
            return proto::NAV_THROUGH_POSES_SMOOTHER_FAILED;
        case proto::NAV_TO_POSE_PATH_INVALID:
            return proto::NAV_THROUGH_POSES_PATH_INVALID;
        case proto::NAV_TO_POSE_CANCELED:
            return proto::NAV_THROUGH_POSES_CANCELED;
        case proto::NAV_TO_POSE_PREEMPTED:
            return proto::NAV_THROUGH_POSES_PREEMPTED;
        case proto::NAV_TO_POSE_INVALID_GOAL:
            return proto::NAV_THROUGH_POSES_INVALID_GOAL;
        case proto::NAV_TO_POSE_NOT_INITIALIZED:
            return proto::NAV_THROUGH_POSES_NOT_INITIALIZED;
        case proto::NAV_TO_POSE_NO_VALID_PATH:
            return proto::NAV_THROUGH_POSES_NO_VALID_PATH;
        case proto::NAV_TO_POSE_GOAL_CHECKER_FAILED:
            return proto::NAV_THROUGH_POSES_GOAL_CHECKER_FAILED;
        case proto::NAV_TO_POSE_UNKNOWN:
        default:
            return proto::NAV_THROUGH_POSES_UNKNOWN;
    }
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
