/*
 * Copyright 2026 The Openbot Authors
 *
 * Wire-format helpers between C++ scene/core types and automsgs.
 *
 * C++ `scene::CollisionObject` stays a compact box/sphere proxy used by
 * planning/collision. On the wire, prefer
 * `automsgs.msgs.moveit_msgs.CollisionObject` (primitives + poses). Conversion
 * maps BOX/SPHERE/CYLINDER primitives and MESH→AABB (+ vertices when present).
 */

#pragma once

#include <automsgs/msgs/moveit_msgs/collision_object.pb.h>
#include <automsgs/msgs/moveit_msgs/robot_trajectory.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>
#include <automsgs/msgs/trajectory_msgs/joint_trajectory.pb.h>

#include "autonomy/manipulation/core/robot_model.hpp"
#include "autonomy/manipulation/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace scene {

/** @brief Convert core::JointState → sensor_msgs.JointState. */
automsgs::msgs::sensor_msgs::JointState ToMsg(const core::JointState& state);

/** @brief Convert sensor_msgs.JointState → core::JointState. */
core::JointState FromMsg(const automsgs::msgs::sensor_msgs::JointState& msg);

/**
 * @brief Convert core::RobotTrajectory → trajectory_msgs.JointTrajectory.
 * @param[in] trajectory Timed joint path (positions required; velocities optional).
 */
automsgs::msgs::trajectory_msgs::JointTrajectory ToJointTrajectoryMsg(
    const core::RobotTrajectory& trajectory);

/**
 * @brief Convert trajectory_msgs.JointTrajectory → core::RobotTrajectory.
 * @param[in] msg Wire joint trajectory.
 */
core::RobotTrajectory FromJointTrajectoryMsg(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& msg);

/**
 * @brief Convert core::RobotTrajectory → moveit_msgs.RobotTrajectory
 *        (joint_trajectory field only).
 */
automsgs::msgs::moveit_msgs::RobotTrajectory ToMsg(
    const core::RobotTrajectory& trajectory);

/**
 * @brief Convert moveit_msgs.RobotTrajectory → core::RobotTrajectory
 *        (reads joint_trajectory).
 */
core::RobotTrajectory FromMsg(
    const automsgs::msgs::moveit_msgs::RobotTrajectory& msg);

/**
 * @brief Convert compact CollisionObject → moveit_msgs.CollisionObject.
 *
 * Emits BOX / SPHERE / CYLINDER primitives; MESH uses AABB + vertices when set.
 */
automsgs::msgs::moveit_msgs::CollisionObject ToMsg(const CollisionObject& object);

/**
 * @brief Convert moveit_msgs.CollisionObject → compact CollisionObject.
 *
 * Maps first supported primitive; MESH falls back to AABB (+ vertices).
 */
CollisionObject FromMsg(const automsgs::msgs::moveit_msgs::CollisionObject& msg);

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
