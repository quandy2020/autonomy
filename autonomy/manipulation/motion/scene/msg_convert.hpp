/*
 * Copyright 2026 The Openbot Authors
 *
 * Conversions between automsgs types used by manipulation.
 * JointState / JointTrajectory / CollisionObject are already wire types.
 */

#pragma once

#include <automsgs/msgs/moveit_msgs/robot_trajectory.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>
#include <automsgs/msgs/trajectory_msgs/joint_trajectory.pb.h>

#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/common/msg_types.hpp"
#include "autonomy/manipulation/motion/scene/collision_object_util.hpp"

namespace autonomy {
namespace manipulation {
namespace scene {

/** @brief Identity: JointState is already sensor_msgs. */
inline const JointState& ToMsg(const JointState& state) { return state; }
inline JointState FromMsg(const JointState& msg) { return msg; }

/**
 * @brief Wrap JointTrajectory as moveit_msgs.RobotTrajectory.
 */
inline RobotTrajectoryMsg ToRobotTrajectoryMsg(const JointTrajectory& trajectory) {
  RobotTrajectoryMsg msg;
  *msg.mutable_joint_trajectory() = trajectory;
  return msg;
}

/**
 * @brief Extract JointTrajectory from moveit_msgs.RobotTrajectory.
 */
inline JointTrajectory FromRobotTrajectoryMsg(const RobotTrajectoryMsg& msg) {
  return msg.joint_trajectory();
}

/** @brief Alias for moveit RobotTrajectory wrap. */
inline RobotTrajectoryMsg ToMsg(const JointTrajectory& trajectory) {
  return ToRobotTrajectoryMsg(trajectory);
}

inline JointTrajectory FromMsg(const RobotTrajectoryMsg& msg) {
  return FromRobotTrajectoryMsg(msg);
}

/** @brief Identity: CollisionObject is already moveit_msgs. */
inline const CollisionObject& ToMsg(const CollisionObject& object) {
  return object;
}
inline CollisionObject FromMsg(const CollisionObject& msg) { return msg; }

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
