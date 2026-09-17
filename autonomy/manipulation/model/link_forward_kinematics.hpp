/*
 * Copyright 2026 The Openbot Authors
 *
 * Lightweight URDF link forward-kinematics tree (no KDL).
 * Computes link poses from joint state using automsgs Pose / JointModel.
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/manipulation/model/robot_model.hpp"

#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>

namespace autonomy {
namespace manipulation {
namespace model {

/**
 * @class LinkForwardKinematicsTree
 * @brief Lightweight URDF link forward-kinematics tree (no KDL dependency).
 */
class LinkForwardKinematicsTree {
 public:
  /**
   * @brief Parse a URDF file into the internal joint/link tree.
   * @param[in] urdf_path Path to URDF file.
   * @param[out] error Optional human-readable failure reason.
   * @return true on successful load.
   */
  bool LoadFromUrdfFile(const std::string& urdf_path,
                        std::string* error = nullptr);

  /** @brief Parsed joints in tree order. */
  const std::vector<JointModel>& Joints() const { return joints_; }

  /** @brief All link names discovered in the URDF. */
  const std::vector<std::string>& LinkNames() const { return link_names_; }

  /** @brief Root link name of the kinematic tree. */
  std::string RootLinkName() const { return root_link_name_; }

  /**
   * @brief Forward kinematics for all links.
   * @param[in] joint_state Joint positions (mimic applied as needed).
   * @param[out] link_poses Map from link name to pose (must be non-null).
   * @return false if required joint values are missing.
   */
  bool ComputeAllLinkPoses(
      const automsgs::msgs::sensor_msgs::JointState& joint_state,
      std::unordered_map<std::string, automsgs::msgs::geometry_msgs::Pose>*
          link_poses) const;

  /**
   * @brief Pose of a single link under @p joint_state.
   */
  bool GetLinkPose(
      const automsgs::msgs::sensor_msgs::JointState& joint_state,
      const std::string& link_name,
      automsgs::msgs::geometry_msgs::Pose* link_pose) const;

 private:
  std::vector<JointModel> joints_;
  std::vector<std::string> link_names_;
  std::string root_link_name_;
  std::unordered_map<std::string, std::size_t> child_link_to_joint_index_;
};

}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
