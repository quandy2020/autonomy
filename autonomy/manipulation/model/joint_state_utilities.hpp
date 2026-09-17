/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file joint_state_utilities.hpp
 * @brief Helpers for automsgs JointState / JointTrajectory (no parallel POD).
 */

#pragma once

#include <string>
#include <vector>

#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>
#include <automsgs/msgs/trajectory_msgs/joint_trajectory.pb.h>
#include <automsgs/msgs/trajectory_msgs/joint_trajectory_point.pb.h>

namespace autonomy {
namespace manipulation {

/**
 * @brief Convert Duration to seconds.
 * @param[in] duration builtin_interfaces/Duration.
 * @return Time in seconds (sec + nanosec·1e-9).
 */
inline double GetDurationSeconds(
    const automsgs::msgs::builtin_interfaces::Duration& duration) {
  return static_cast<double>(duration.sec()) +
         static_cast<double>(duration.nanosec()) * 1e-9;
}

/**
 * @brief Write seconds into a Duration message.
 * @param[in] seconds Time in seconds.
 * @param[out] duration Target Duration (no-op if null).
 */
inline void SetDurationSeconds(
    double seconds, automsgs::msgs::builtin_interfaces::Duration* duration) {
  if (!duration) {
    return;
  }
  const int64_t sec = static_cast<int64_t>(seconds);
  const int32_t nanosec =
      static_cast<int32_t>((seconds - static_cast<double>(sec)) * 1e9);
  duration->set_sec(sec);
  duration->set_nanosec(nanosec);
}

/**
 * @brief Number of position degrees of freedom in a joint state.
 * @param[in] state sensor_msgs/JointState.
 * @return Size of the position field.
 */
inline int GetJointStateDegreesOfFreedom(
    const automsgs::msgs::sensor_msgs::JointState& state) {
  return state.position_size();
}

/**
 * @brief Assign names and positions from vectors (clears velocity/effort).
 * @param[out] state Target JointState (no-op if null).
 * @param[in] names Joint names.
 * @param[in] positions Joint positions (same order as @p names).
 */
inline void SetJointState(automsgs::msgs::sensor_msgs::JointState* state,
                          const std::vector<std::string>& names,
                          const std::vector<double>& positions) {
  if (!state) {
    return;
  }
  state->clear_name();
  state->clear_position();
  state->clear_velocity();
  state->clear_effort();
  for (const auto& n : names) {
    state->add_name(n);
  }
  for (double q : positions) {
    state->add_position(q);
  }
}

/**
 * @brief Resize position to @p dof (pads with 0 or truncates).
 * @param[in,out] state Target JointState (no-op if null or @p dof < 0).
 * @param[in] dof Desired position length.
 */
inline void ResizeJointState(automsgs::msgs::sensor_msgs::JointState* state,
                             int dof) {
  if (!state || dof < 0) {
    return;
  }
  while (state->position_size() < dof) {
    state->add_position(0.0);
  }
  while (state->position_size() > dof) {
    state->mutable_position()->RemoveLast();
  }
}

/**
 * @brief Copy joint names from trajectory header onto a joint state.
 * @param[in] traj Source trajectory (joint_names).
 * @param[out] state Target JointState (no-op if null); position unchanged.
 */
inline void CopyTrajectoryJointNames(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& traj,
    automsgs::msgs::sensor_msgs::JointState* state) {
  if (!state) {
    return;
  }
  state->clear_name();
  for (const auto& n : traj.joint_names()) {
    state->add_name(n);
  }
}

/**
 * @brief Build a JointState from a trajectory point + joint_names.
 * @param[in] traj Source trajectory.
 * @param[in] point_index Index into traj.points (OOB → names only).
 * @return JointState with names and, when in range, position/velocity/effort.
 */
inline automsgs::msgs::sensor_msgs::JointState MakeJointStateFromPoint(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& traj,
    int point_index) {
  automsgs::msgs::sensor_msgs::JointState state;
  CopyTrajectoryJointNames(traj, &state);
  if (point_index < 0 || point_index >= traj.points_size()) {
    return state;
  }
  const auto& pt = traj.points(point_index);
  state.clear_position();
  for (double q : pt.positions()) {
    state.add_position(q);
  }
  state.clear_velocity();
  for (double v : pt.velocities()) {
    state.add_velocity(v);
  }
  state.clear_effort();
  for (double e : pt.effort()) {
    state.add_effort(e);
  }
  return state;
}

/**
 * @brief Append a waypoint (positions) to trajectory; copies names if empty.
 * @param[in,out] traj Target trajectory (no-op / nullptr if null).
 * @param[in] state Waypoint positions (and optional velocity/effort).
 * @param[in] time_from_start_s Stamp of the new point in seconds.
 * @return Pointer to the added point, or nullptr if @p traj is null.
 */
inline automsgs::msgs::trajectory_msgs::JointTrajectoryPoint* AddTrajectoryPoint(
    automsgs::msgs::trajectory_msgs::JointTrajectory* traj,
    const automsgs::msgs::sensor_msgs::JointState& state,
    double time_from_start_s) {
  if (!traj) {
    return nullptr;
  }
  if (traj->joint_names_size() == 0) {
    for (const auto& n : state.name()) {
      traj->add_joint_names(n);
    }
  }
  auto* pt = traj->add_points();
  for (double q : state.position()) {
    pt->add_positions(q);
  }
  for (double v : state.velocity()) {
    pt->add_velocities(v);
  }
  for (double e : state.effort()) {
    pt->add_effort(e);
  }
  SetDurationSeconds(time_from_start_s, pt->mutable_time_from_start());
  return pt;
}

/**
 * @brief time_from_start of trajectory point @p point_index in seconds.
 * @param[in] traj Source trajectory.
 * @param[in] point_index Point index.
 * @return Seconds, or 0 if @p point_index is out of range.
 */
inline double GetTrajectoryPointTimeSeconds(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& traj,
    int point_index) {
  if (point_index < 0 || point_index >= traj.points_size()) {
    return 0.0;
  }
  return GetDurationSeconds(traj.points(point_index).time_from_start());
}

/**
 * @brief Set geometry_msgs Pose translation + quaternion.
 * @param[out] pose Target pose (no-op if null).
 * @param[in] x Translation x (m).
 * @param[in] y Translation y (m).
 * @param[in] z Translation z (m).
 * @param[in] qx Orientation x.
 * @param[in] qy Orientation y.
 * @param[in] qz Orientation z.
 * @param[in] qw Orientation w.
 */
inline void SetPose(automsgs::msgs::geometry_msgs::Pose* pose, double x,
                    double y, double z, double qx, double qy, double qz,
                    double qw) {
  if (!pose) {
    return;
  }
  pose->mutable_position()->set_x(x);
  pose->mutable_position()->set_y(y);
  pose->mutable_position()->set_z(z);
  pose->mutable_orientation()->set_x(qx);
  pose->mutable_orientation()->set_y(qy);
  pose->mutable_orientation()->set_z(qz);
  pose->mutable_orientation()->set_w(qw);
}

/**
 * @brief Identity pose (origin, quaternion w=1).
 * @return Pose at (0,0,0) with identity orientation.
 */
inline automsgs::msgs::geometry_msgs::Pose MakeIdentityPose() {
  automsgs::msgs::geometry_msgs::Pose pose;
  SetPose(&pose, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  return pose;
}

}  // namespace manipulation
}  // namespace autonomy
