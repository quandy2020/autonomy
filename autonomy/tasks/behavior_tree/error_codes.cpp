/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/error_codes.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace {

proto::NavigateToPoseErrorCode MapGetPathError(
    proto::GetPathErrorCode outcome) {
    switch (outcome) {
        case proto::GET_PATH_SUCCESS:
            return proto::NAV_TO_POSE_NONE;
        case proto::GET_PATH_CANCELED:
        case proto::GET_PATH_STOPPED:
            return proto::NAV_TO_POSE_CANCELED;
        case proto::GET_PATH_INVALID_GOAL:
        case proto::GET_PATH_INVALID_START:
            return proto::NAV_TO_POSE_INVALID_GOAL;
        case proto::GET_PATH_TF_ERROR:
            return proto::NAV_TO_POSE_TF_ERROR;
        case proto::GET_PATH_NOT_INITIALIZED:
        case proto::GET_PATH_INVALID_PLUGIN:
            return proto::NAV_TO_POSE_NOT_INITIALIZED;
        case proto::GET_PATH_NO_PATH_FOUND:
        case proto::GET_PATH_EMPTY_PATH:
        case proto::GET_PATH_BLOCKED_START:
        case proto::GET_PATH_BLOCKED_GOAL:
        case proto::GET_PATH_OUT_OF_MAP:
            return proto::NAV_TO_POSE_NO_VALID_PATH;
        case proto::GET_PATH_PAT_EXCEEDED:
            return proto::NAV_TO_POSE_TIMEOUT;
        case proto::GET_PATH_MAP_ERROR:
            return proto::NAV_TO_POSE_PATH_INVALID;
        case proto::GET_PATH_INTERNAL_ERROR:
            return proto::NAV_TO_POSE_SMOOTHER_FAILED;
        case proto::GET_PATH_FAILURE:
        default:
            return proto::NAV_TO_POSE_PLANNER_FAILED;
    }
}

proto::NavigateToPoseErrorCode MapExePathError(
    proto::ExePathErrorCode outcome) {
    switch (outcome) {
        case proto::EXE_PATH_SUCCESS:
            return proto::NAV_TO_POSE_NONE;
        case proto::EXE_PATH_CANCELED:
        case proto::EXE_PATH_STOPPED:
            return proto::NAV_TO_POSE_CANCELED;
        case proto::EXE_PATH_TF_ERROR:
            return proto::NAV_TO_POSE_TF_ERROR;
        case proto::EXE_PATH_NOT_INITIALIZED:
        case proto::EXE_PATH_INVALID_PLUGIN:
            return proto::NAV_TO_POSE_NOT_INITIALIZED;
        case proto::EXE_PATH_INVALID_PATH:
        case proto::EXE_PATH_MISSED_PATH:
        case proto::EXE_PATH_BLOCKED_PATH:
        case proto::EXE_PATH_OUT_OF_MAP:
        case proto::EXE_PATH_MAP_ERROR:
            return proto::NAV_TO_POSE_PATH_INVALID;
        case proto::EXE_PATH_PAT_EXCEEDED:
            return proto::NAV_TO_POSE_TIMEOUT;
        case proto::EXE_PATH_MISSED_GOAL:
        case proto::EXE_PATH_BLOCKED_GOAL:
            return proto::NAV_TO_POSE_GOAL_CHECKER_FAILED;
        case proto::EXE_PATH_NO_VALID_CMD:
        case proto::EXE_PATH_ROBOT_STUCK:
        case proto::EXE_PATH_OSCILLATION:
        case proto::EXE_PATH_COLLISION:
        case proto::EXE_PATH_FAILURE:
        default:
            return proto::NAV_TO_POSE_CONTROLLER_FAILED;
    }
}

proto::NavigateToPoseErrorCode MapRecoveryError(
    proto::RecoveryErrorCode outcome) {
    switch (outcome) {
        case proto::RECOVERY_SUCCESS:
            return proto::NAV_TO_POSE_NONE;
        case proto::RECOVERY_CANCELED:
        case proto::RECOVERY_STOPPED:
            return proto::NAV_TO_POSE_CANCELED;
        case proto::RECOVERY_TF_ERROR:
            return proto::NAV_TO_POSE_TF_ERROR;
        case proto::RECOVERY_NOT_INITIALIZED:
        case proto::RECOVERY_INVALID_PLUGIN:
            return proto::NAV_TO_POSE_NOT_INITIALIZED;
        case proto::RECOVERY_PAT_EXCEEDED:
            return proto::NAV_TO_POSE_TIMEOUT;
        case proto::RECOVERY_IMPASSABLE:
            return proto::NAV_TO_POSE_PATH_INVALID;
        case proto::RECOVERY_FAILURE:
        case proto::RECOVERY_INTERNAL_ERROR:
        default:
            return proto::NAV_TO_POSE_UNKNOWN;
    }
}

proto::NavigateToPoseErrorCode MapMoveBaseError(
    proto::MoveBaseErrorCode outcome) {
    switch (outcome) {
        case proto::MOVE_BASE_SUCCESS:
            return proto::NAV_TO_POSE_NONE;
        case proto::MOVE_BASE_CANCELED:
            return proto::NAV_TO_POSE_CANCELED;
        case proto::MOVE_BASE_TF_ERROR:
            return proto::NAV_TO_POSE_TF_ERROR;
        case proto::MOVE_BASE_START_BLOCKED:
        case proto::MOVE_BASE_GOAL_BLOCKED:
            return proto::NAV_TO_POSE_NO_VALID_PATH;
        case proto::MOVE_BASE_COLLISION:
        case proto::MOVE_BASE_OSCILLATION:
            return proto::NAV_TO_POSE_CONTROLLER_FAILED;
        case proto::MOVE_BASE_PLAN_FAILURE:
            return proto::NAV_TO_POSE_PLANNER_FAILED;
        case proto::MOVE_BASE_CTRL_FAILURE:
            return proto::NAV_TO_POSE_CONTROLLER_FAILED;
        case proto::MOVE_BASE_FAILURE:
        case proto::MOVE_BASE_INTERNAL_ERROR:
        default:
            return proto::NAV_TO_POSE_UNKNOWN;
    }
}

proto::NavigateThroughPosesErrorCode MapNavToThrough(
    proto::NavigateToPoseErrorCode code) {
    switch (code) {
        case proto::NAV_TO_POSE_NONE:
            return proto::NAV_THROUGH_POSES_NONE;
        case proto::NAV_TO_POSE_TF_ERROR:
            return proto::NAV_THROUGH_POSES_TF_ERROR;
        case proto::NAV_TO_POSE_TIMEOUT:
            return proto::NAV_THROUGH_POSES_TIMEOUT;
        case proto::NAV_TO_POSE_NO_VALID_PATH:
            return proto::NAV_THROUGH_POSES_NO_VALID_PATH;
        case proto::NAV_TO_POSE_CANCELED:
            return proto::NAV_THROUGH_POSES_CANCELED;
        case proto::NAV_TO_POSE_PREEMPTED:
            return proto::NAV_THROUGH_POSES_PREEMPTED;
        case proto::NAV_TO_POSE_INVALID_GOAL:
            return proto::NAV_THROUGH_POSES_INVALID_GOAL;
        case proto::NAV_TO_POSE_NOT_INITIALIZED:
            return proto::NAV_THROUGH_POSES_NOT_INITIALIZED;
        case proto::NAV_TO_POSE_PLANNER_FAILED:
            return proto::NAV_THROUGH_POSES_PLANNER_FAILED;
        case proto::NAV_TO_POSE_CONTROLLER_FAILED:
            return proto::NAV_THROUGH_POSES_CONTROLLER_FAILED;
        case proto::NAV_TO_POSE_SMOOTHER_FAILED:
            return proto::NAV_THROUGH_POSES_SMOOTHER_FAILED;
        case proto::NAV_TO_POSE_PATH_INVALID:
            return proto::NAV_THROUGH_POSES_PATH_INVALID;
        case proto::NAV_TO_POSE_GOAL_CHECKER_FAILED:
            return proto::NAV_THROUGH_POSES_GOAL_CHECKER_FAILED;
        case proto::NAV_TO_POSE_FAILED_TO_LOAD_BEHAVIOR_TREE:
            return proto::NAV_THROUGH_POSES_FAILED_TO_LOAD_BEHAVIOR_TREE;
        case proto::NAV_TO_POSE_UNKNOWN:
        default:
            return proto::NAV_THROUGH_POSES_UNKNOWN;
    }
}

}  // namespace

proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    proto::GetPathErrorCode outcome) {
    return MapGetPathError(outcome);
}

proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    proto::ExePathErrorCode outcome) {
    return MapExePathError(outcome);
}

proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    proto::RecoveryErrorCode outcome) {
    return MapRecoveryError(outcome);
}

proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    proto::MoveBaseErrorCode outcome) {
    return MapMoveBaseError(outcome);
}

proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    const uint32_t outcome) {
    if (outcome >= 9000 && outcome < 9200) {
        return static_cast<proto::NavigateToPoseErrorCode>(outcome);
    }
    if (outcome >= 150 && outcome <= 199) {
        return MapRecoveryError(static_cast<proto::RecoveryErrorCode>(outcome));
    }
    if (outcome >= 100 && outcome <= 149) {
        return MapExePathError(static_cast<proto::ExePathErrorCode>(outcome));
    }
    if (outcome >= 50 && outcome <= 99) {
        return MapGetPathError(static_cast<proto::GetPathErrorCode>(outcome));
    }
    if (outcome >= 10 && outcome <= 49) {
        return MapMoveBaseError(static_cast<proto::MoveBaseErrorCode>(outcome));
    }
    if (outcome == 0) {
        return proto::NAV_TO_POSE_NONE;
    }
    return proto::NAV_TO_POSE_UNKNOWN;
}

proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    proto::GetPathErrorCode outcome) {
    return MapNavToThrough(MapGetPathError(outcome));
}

proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    proto::ExePathErrorCode outcome) {
    return MapNavToThrough(MapExePathError(outcome));
}

proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    proto::RecoveryErrorCode outcome) {
    return MapNavToThrough(MapRecoveryError(outcome));
}

proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    proto::MoveBaseErrorCode outcome) {
    return MapNavToThrough(MapMoveBaseError(outcome));
}

proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    const uint32_t outcome) {
    return MapNavToThrough(ToNavigateToPoseErrorCode(outcome));
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
