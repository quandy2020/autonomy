/*
 * Copyright 2026 The Openbot Authors
 *
 * Task-type aliases for Bridge muxer / idempotency (vehicle_msgs, not bridge.proto).
 */

#pragma once

#include <string>

#include <automsgs/msgs/vehicle_msgs/robot_task_status.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {

using TaskType = ::automsgs::msgs::vehicle_msgs::RobotTaskType;
using TaskStatus = ::automsgs::msgs::vehicle_msgs::RobotTaskStatus;

inline constexpr TaskType TASK_TYPE_NONE =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_NONE;
inline constexpr TaskType TASK_TYPE_NAVIGATION =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_NAVIGATION;
inline constexpr TaskType TASK_TYPE_FOLLOW =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_FOLLOW;
inline constexpr TaskType TASK_TYPE_TELEOP =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_TELEOP;
inline constexpr TaskType TASK_TYPE_EXPLORATION =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_EXPLORATION;
inline constexpr TaskType TASK_TYPE_DOCK =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_DOCK;
inline constexpr TaskType TASK_TYPE_MAP =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_MAP;

inline constexpr TaskStatus TASK_STATUS_UNKNOWN =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_UNKNOWN;
inline constexpr TaskStatus TASK_STATUS_IDLE =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_IDLE;
inline constexpr TaskStatus TASK_STATUS_RUNNING =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_RUNNING;
inline constexpr TaskStatus TASK_STATUS_PAUSED =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_PAUSED;
inline constexpr TaskStatus TASK_STATUS_SUCCEEDED =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_SUCCEEDED;
inline constexpr TaskStatus TASK_STATUS_FAILED =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_FAILED;
inline constexpr TaskStatus TASK_STATUS_CANCELED =
    ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_CANCELED;

/** @brief Muxer / GetActiveGoal snapshot (no bridge.proto). */
struct ActiveTaskInfo {
    TaskType type{TASK_TYPE_NONE};
    TaskStatus status{TASK_STATUS_IDLE};
    std::string cmd_id;
    std::string client_id;
};

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
