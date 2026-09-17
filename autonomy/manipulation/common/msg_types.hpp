/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file msg_types.hpp
 * @brief Canonical wire / runtime message aliases for manipulation (automsgs only).
 *
 * No parallel POD structs — use these aliases and protobuf accessors.
 */

#pragma once

#include <automsgs/msgs/control_msgs/joint_jog.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/moveit_msgs/allowed_collision_matrix.pb.h>
#include <automsgs/msgs/moveit_msgs/attached_collision_object.pb.h>
#include <automsgs/msgs/moveit_msgs/collision_object.pb.h>
#include <automsgs/msgs/moveit_msgs/motion_plan.pb.h>
#include <automsgs/msgs/moveit_msgs/robot_trajectory.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>
#include <automsgs/msgs/trajectory_msgs/joint_trajectory.pb.h>
#include <automsgs/msgs/trajectory_msgs/joint_trajectory_point.pb.h>

namespace autonomy {
namespace manipulation {

/** @brief sensor_msgs/JointState (name / position / velocity / effort). */
using JointState = automsgs::msgs::sensor_msgs::JointState;

/** @brief trajectory_msgs/JointTrajectory. */
using JointTrajectory = automsgs::msgs::trajectory_msgs::JointTrajectory;

/** @brief trajectory_msgs/JointTrajectoryPoint. */
using JointTrajectoryPoint =
    automsgs::msgs::trajectory_msgs::JointTrajectoryPoint;

/** @brief geometry_msgs/Pose. */
using Pose = automsgs::msgs::geometry_msgs::Pose;

/** @brief geometry_msgs/PoseStamped. */
using PoseStamped = automsgs::msgs::geometry_msgs::PoseStamped;

/** @brief geometry_msgs/Twist. */
using Twist = automsgs::msgs::geometry_msgs::Twist;

/** @brief geometry_msgs/TwistStamped. */
using TwistStamped = automsgs::msgs::geometry_msgs::TwistStamped;

/** @brief control_msgs/JointJog. */
using JointJog = automsgs::msgs::control_msgs::JointJog;

/** @brief moveit_msgs/CollisionObject (canonical scene geometry). */
using CollisionObjectMsg = automsgs::msgs::moveit_msgs::CollisionObject;

/** @brief moveit_msgs/AttachedCollisionObject. */
using AttachedCollisionObjectMsg =
    automsgs::msgs::moveit_msgs::AttachedCollisionObject;

/** @brief moveit_msgs/AllowedCollisionMatrix. */
using AllowedCollisionMatrixMsg =
    automsgs::msgs::moveit_msgs::AllowedCollisionMatrix;

/** @brief moveit_msgs/RobotTrajectory (joint_trajectory wrapper). */
using RobotTrajectoryMsg = automsgs::msgs::moveit_msgs::RobotTrajectory;

/** @brief moveit_msgs/JointConstraint. */
using JointConstraintMsg = automsgs::msgs::moveit_msgs::JointConstraint;

/** @brief moveit_msgs/PositionConstraint. */
using PositionConstraintMsg = automsgs::msgs::moveit_msgs::PositionConstraint;

/** @brief moveit_msgs/OrientationConstraint. */
using OrientationConstraintMsg =
    automsgs::msgs::moveit_msgs::OrientationConstraint;

/** @brief Shorthand aliases used by scene / planners. */
using CollisionObject = CollisionObjectMsg;
using AttachedCollisionObject = AttachedCollisionObjectMsg;
using AllowedCollisionMatrix = AllowedCollisionMatrixMsg;
using JointConstraint = JointConstraintMsg;
using PositionConstraint = PositionConstraintMsg;
using OrientationConstraint = OrientationConstraintMsg;

namespace core {
/** @brief Alias of manipulation::JointState for core APIs. */
using JointState = ::autonomy::manipulation::JointState;
/** @brief Joint-space path as trajectory_msgs/JointTrajectory. */
using RobotTrajectory = ::autonomy::manipulation::JointTrajectory;
}  // namespace core

namespace kinematics {
/** @brief Alias of manipulation::Pose for kinematics APIs. */
using Pose = ::autonomy::manipulation::Pose;
}  // namespace kinematics

}  // namespace manipulation
}  // namespace autonomy
