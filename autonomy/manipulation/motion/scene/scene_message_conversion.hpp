/*
 * Copyright 2026 The Openbot Authors
 *
 * Conversions between automsgs types used by manipulation.
 * JointState / JointTrajectory / CollisionObject are already wire types.
 */

#pragma once

#include <automsgs/msgs/moveit_msgs/collision_object.pb.h>
#include <automsgs/msgs/moveit_msgs/robot_trajectory.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>
#include <automsgs/msgs/trajectory_msgs/joint_trajectory.pb.h>

namespace autonomy {
namespace manipulation {
namespace scene {

/** @brief Identity: JointState is already sensor_msgs. */
inline const automsgs::msgs::sensor_msgs::JointState& ToMessage(
    const automsgs::msgs::sensor_msgs::JointState& state) {
  return state;
}
inline automsgs::msgs::sensor_msgs::JointState FromMessage(
    const automsgs::msgs::sensor_msgs::JointState& msg) {
  return msg;
}

/**
 * @brief Wrap JointTrajectory as moveit_msgs.RobotTrajectory.
 */
inline automsgs::msgs::moveit_msgs::RobotTrajectory ToRobotTrajectoryMessage(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& trajectory) {
  automsgs::msgs::moveit_msgs::RobotTrajectory msg;
  *msg.mutable_joint_trajectory() = trajectory;
  return msg;
}

/**
 * @brief Extract JointTrajectory from moveit_msgs.RobotTrajectory.
 */
inline automsgs::msgs::trajectory_msgs::JointTrajectory FromRobotTrajectoryMessage(
    const automsgs::msgs::moveit_msgs::RobotTrajectory& msg) {
  return msg.joint_trajectory();
}

/** @brief Alias for moveit RobotTrajectory wrap. */
inline automsgs::msgs::moveit_msgs::RobotTrajectory ToMessage(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& trajectory) {
  return ToRobotTrajectoryMessage(trajectory);
}

inline automsgs::msgs::trajectory_msgs::JointTrajectory FromMessage(
    const automsgs::msgs::moveit_msgs::RobotTrajectory& msg) {
  return FromRobotTrajectoryMessage(msg);
}

/** @brief Identity: CollisionObject is already moveit_msgs. */
inline const automsgs::msgs::moveit_msgs::CollisionObject& ToMessage(
    const automsgs::msgs::moveit_msgs::CollisionObject& object) {
  return object;
}
inline automsgs::msgs::moveit_msgs::CollisionObject FromMessage(
    const automsgs::msgs::moveit_msgs::CollisionObject& msg) {
  return msg;
}

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
