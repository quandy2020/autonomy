/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file task_types.hpp
 * @brief Task-type aliases for Bridge muxer / idempotency (vehicle_msgs, not
 * bridge.proto).
 *
 * @details
 * Re-exports automsgs vehicle_msgs RobotTaskType / RobotTaskStatus enums as
 * TaskType / TaskStatus plus named constexpr aliases used by TaskMuxer,
 * CommandIdempotencyCache, and System RPC snapshots. Keeps bridge free of
 * bridge.proto task enums.
 *
 * @see TaskMuxer
 * @see ActiveTaskInfo
 */

#pragma once

#include <string>

#include <automsgs/msgs/vehicle_msgs/robot_task_status.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

/**
 * @brief Alias for vehicle_msgs::RobotTaskType (muxer / idempotency key).
 */
using TaskType = ::automsgs::msgs::vehicle_msgs::RobotTaskType;

/**
 * @brief Alias for vehicle_msgs::RobotTaskStatus (ActiveTaskInfo snapshot).
 */
using TaskStatus = ::automsgs::msgs::vehicle_msgs::RobotTaskStatus;

/** @brief No exclusive task / empty muxer slot. */
inline constexpr TaskType TASK_TYPE_NONE =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_NONE;
/** @brief Navigation exclusive task type. */
inline constexpr TaskType TASK_TYPE_NAVIGATION =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_NAVIGATION;
/** @brief Follow / tracking exclusive task type. */
inline constexpr TaskType TASK_TYPE_FOLLOW =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_FOLLOW;
/** @brief Teleop exclusive task type. */
inline constexpr TaskType TASK_TYPE_TELEOP =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_TELEOP;
/** @brief Exploration exclusive task type. */
inline constexpr TaskType TASK_TYPE_EXPLORATION =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_EXPLORATION;
/** @brief Dock / charge exclusive task type. */
inline constexpr TaskType TASK_TYPE_DOCK =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_DOCK;
/** @brief Mapping exclusive task type. */
inline constexpr TaskType TASK_TYPE_MAP =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_MAP;
/** @brief Voice exclusive task type. */
inline constexpr TaskType TASK_TYPE_VOICE =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_VOICE;

/** @brief Unknown / unset task status. */
inline constexpr TaskStatus TASK_STATUS_UNKNOWN =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_UNKNOWN;
/** @brief Idle (no running exclusive task). */
inline constexpr TaskStatus TASK_STATUS_IDLE =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_IDLE;
/** @brief Running exclusive task. */
inline constexpr TaskStatus TASK_STATUS_RUNNING =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_RUNNING;
/** @brief Paused exclusive task. */
inline constexpr TaskStatus TASK_STATUS_PAUSED =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_PAUSED;
/** @brief Succeeded terminal status. */
inline constexpr TaskStatus TASK_STATUS_SUCCEEDED =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_SUCCEEDED;
/** @brief Failed terminal status. */
inline constexpr TaskStatus TASK_STATUS_FAILED =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_FAILED;
/** @brief Canceled terminal status. */
inline constexpr TaskStatus TASK_STATUS_CANCELED =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_CANCELED;

/**
 * @brief Muxer / GetActiveGoal snapshot (no bridge.proto).
 *
 * Lightweight view of the exclusive active command slot. Produced by
 * TaskMuxer::GetSnapshot and consumed by System RPCs / StateHub overlays.
 * Not a live handle — fields are a point-in-time copy under the muxer lock.
 *
 * @par Invariants
 * - type == TASK_TYPE_NONE means no exclusive task (cmd_id / client_id empty).
 * - status is synthesized by readers when needed; muxer itself primarily
 * tracks type + ids (status often IDLE/RUNNING at the snapshot boundary).
 *
 * @par Ownership
 * value type; SharedPtr aliases available for API uniformity.
 */
struct ActiveTaskInfo {
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ActiveTaskInfo)

    /**
     * @brief Exclusive TaskType holding the muxer slot (NONE when free).
     */
    TaskType type{TASK_TYPE_NONE};

    /**
     * @brief Coarse status for System / overlay readers (often IDLE/RUNNING).
     */
    TaskStatus status{TASK_STATUS_IDLE};

    /**
     * @brief Command / goal id recorded by TryAcquire.
     */
    std::string cmd_id;

    /**
     * @brief Client id recorded by TryAcquire (may be empty).
     */
    std::string client_id;
};

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
