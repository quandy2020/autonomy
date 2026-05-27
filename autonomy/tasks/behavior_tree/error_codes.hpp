/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <cstdint>

#include "autonomy/tasks/proto/error_code.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/** MBF GetPath outcome constants for BT blackboard error_code_id. */
struct GetPathOutcome {
    static constexpr uint16_t SUCCESS =
        static_cast<uint16_t>(proto::GET_PATH_SUCCESS);
    static constexpr uint16_t FAILURE =
        static_cast<uint16_t>(proto::GET_PATH_FAILURE);
    static constexpr uint16_t CANCELED =
        static_cast<uint16_t>(proto::GET_PATH_CANCELED);
    static constexpr uint16_t INVALID_START =
        static_cast<uint16_t>(proto::GET_PATH_INVALID_START);
    static constexpr uint16_t INVALID_GOAL =
        static_cast<uint16_t>(proto::GET_PATH_INVALID_GOAL);
    static constexpr uint16_t BLOCKED_START =
        static_cast<uint16_t>(proto::GET_PATH_BLOCKED_START);
    static constexpr uint16_t BLOCKED_GOAL =
        static_cast<uint16_t>(proto::GET_PATH_BLOCKED_GOAL);
    static constexpr uint16_t NO_PATH_FOUND =
        static_cast<uint16_t>(proto::GET_PATH_NO_PATH_FOUND);
    static constexpr uint16_t PAT_EXCEEDED =
        static_cast<uint16_t>(proto::GET_PATH_PAT_EXCEEDED);
    static constexpr uint16_t EMPTY_PATH =
        static_cast<uint16_t>(proto::GET_PATH_EMPTY_PATH);
    static constexpr uint16_t TF_ERROR =
        static_cast<uint16_t>(proto::GET_PATH_TF_ERROR);
    static constexpr uint16_t NOT_INITIALIZED =
        static_cast<uint16_t>(proto::GET_PATH_NOT_INITIALIZED);
    static constexpr uint16_t INVALID_PLUGIN =
        static_cast<uint16_t>(proto::GET_PATH_INVALID_PLUGIN);
    static constexpr uint16_t INTERNAL_ERROR =
        static_cast<uint16_t>(proto::GET_PATH_INTERNAL_ERROR);
    static constexpr uint16_t OUT_OF_MAP =
        static_cast<uint16_t>(proto::GET_PATH_OUT_OF_MAP);
    static constexpr uint16_t MAP_ERROR =
        static_cast<uint16_t>(proto::GET_PATH_MAP_ERROR);
    static constexpr uint16_t STOPPED =
        static_cast<uint16_t>(proto::GET_PATH_STOPPED);
};

/** MBF ExePath outcome constants for BT blackboard error_code_id. */
struct ExePathOutcome {
    static constexpr uint16_t SUCCESS =
        static_cast<uint16_t>(proto::EXE_PATH_SUCCESS);
    static constexpr uint16_t FAILURE =
        static_cast<uint16_t>(proto::EXE_PATH_FAILURE);
    static constexpr uint16_t CANCELED =
        static_cast<uint16_t>(proto::EXE_PATH_CANCELED);
    static constexpr uint16_t NO_VALID_CMD =
        static_cast<uint16_t>(proto::EXE_PATH_NO_VALID_CMD);
    static constexpr uint16_t PAT_EXCEEDED =
        static_cast<uint16_t>(proto::EXE_PATH_PAT_EXCEEDED);
    static constexpr uint16_t COLLISION =
        static_cast<uint16_t>(proto::EXE_PATH_COLLISION);
    static constexpr uint16_t OSCILLATION =
        static_cast<uint16_t>(proto::EXE_PATH_OSCILLATION);
    static constexpr uint16_t ROBOT_STUCK =
        static_cast<uint16_t>(proto::EXE_PATH_ROBOT_STUCK);
    static constexpr uint16_t MISSED_GOAL =
        static_cast<uint16_t>(proto::EXE_PATH_MISSED_GOAL);
    static constexpr uint16_t MISSED_PATH =
        static_cast<uint16_t>(proto::EXE_PATH_MISSED_PATH);
    static constexpr uint16_t BLOCKED_GOAL =
        static_cast<uint16_t>(proto::EXE_PATH_BLOCKED_GOAL);
    static constexpr uint16_t BLOCKED_PATH =
        static_cast<uint16_t>(proto::EXE_PATH_BLOCKED_PATH);
    static constexpr uint16_t INVALID_PATH =
        static_cast<uint16_t>(proto::EXE_PATH_INVALID_PATH);
    static constexpr uint16_t TF_ERROR =
        static_cast<uint16_t>(proto::EXE_PATH_TF_ERROR);
    static constexpr uint16_t NOT_INITIALIZED =
        static_cast<uint16_t>(proto::EXE_PATH_NOT_INITIALIZED);
    static constexpr uint16_t INVALID_PLUGIN =
        static_cast<uint16_t>(proto::EXE_PATH_INVALID_PLUGIN);
    static constexpr uint16_t INTERNAL_ERROR =
        static_cast<uint16_t>(proto::EXE_PATH_INTERNAL_ERROR);
    static constexpr uint16_t OUT_OF_MAP =
        static_cast<uint16_t>(proto::EXE_PATH_OUT_OF_MAP);
    static constexpr uint16_t MAP_ERROR =
        static_cast<uint16_t>(proto::EXE_PATH_MAP_ERROR);
    static constexpr uint16_t STOPPED =
        static_cast<uint16_t>(proto::EXE_PATH_STOPPED);
};

/** MBF Recovery outcome constants. */
struct RecoveryOutcome {
    static constexpr uint16_t SUCCESS =
        static_cast<uint16_t>(proto::RECOVERY_SUCCESS);
    static constexpr uint16_t FAILURE =
        static_cast<uint16_t>(proto::RECOVERY_FAILURE);
    static constexpr uint16_t CANCELED =
        static_cast<uint16_t>(proto::RECOVERY_CANCELED);
    static constexpr uint16_t TF_ERROR =
        static_cast<uint16_t>(proto::RECOVERY_TF_ERROR);
    static constexpr uint16_t IMPASSABLE =
        static_cast<uint16_t>(proto::RECOVERY_IMPASSABLE);
};

proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    proto::GetPathErrorCode outcome);
proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    proto::ExePathErrorCode outcome);
proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    proto::RecoveryErrorCode outcome);
proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    proto::MoveBaseErrorCode outcome);
/** Map any MBF-style uint32 outcome (auto-detect by value range). */
proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(uint32_t outcome);

proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    proto::GetPathErrorCode outcome);
proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    proto::ExePathErrorCode outcome);
proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    proto::RecoveryErrorCode outcome);
proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    proto::MoveBaseErrorCode outcome);
proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    uint32_t outcome);

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
