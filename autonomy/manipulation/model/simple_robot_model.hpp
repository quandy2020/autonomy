/*
 * Copyright 2026 The Openbot Authors
 *
 * Concrete @ref RobotModel with URDF/SRDF groups, limits, and ACM seeds.
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "autonomy/manipulation/model/link_forward_kinematics.hpp"
#include "autonomy/manipulation/model/robot_model.hpp"
#include "autonomy/manipulation/model/srdf_planning_groups.hpp"

namespace autonomy {
namespace manipulation {
namespace model {

/**
 * @class SimpleRobotModel
 * @brief Concrete @ref RobotModel with URDF/SRDF groups, limits, and ACM seeds.
 */
class SimpleRobotModel : public RobotModel {
 public:
  bool Load(const std::string& urdf, const std::string& srdf) override;

  std::vector<std::string> GetJointNames(
      const std::string& group) const override;

  void SetGroupJoints(const std::string& group,
                      std::vector<std::string> joint_names);

  const std::vector<std::string>& AllJoints() const { return all_joints_; }

  const std::vector<JointModel>& Joints() const override { return joints_; }

  const JointModelGroup* GetJointModelGroup(
      const std::string& name) const override;

  const JointLimits* GetJointLimits(
      const std::string& joint_name) const override;

  void SetGroupChainFrames(const std::string& group,
                           const std::string& base_frame,
                           const std::string& tip_frame);

  bool HasGroup(const std::string& group) const {
    return groups_.count(group) > 0;
  }

  const std::vector<std::pair<std::string, std::string>>&
  DisabledCollisions() const {
    return disabled_collisions_;
  }

  const std::vector<SrdfEndEffector>& EndEffectors() const {
    return end_effectors_;
  }

  const std::vector<std::string>& PassiveJoints() const {
    return passive_joints_;
  }

  const std::string& UrdfPath() const { return urdf_path_; }

  const LinkForwardKinematicsTree& GetLinkForwardKinematicsTree() const {
    return link_tree_;
  }

  bool EnsureLinkForwardKinematicsTree(const std::string& urdf_path = {}) {
    const std::string path = urdf_path.empty() ? urdf_path_ : urdf_path;
    if (path.empty()) {
      return !link_tree_.LinkNames().empty();
    }
    return link_tree_.LoadFromUrdfFile(path);
  }

 private:
  std::vector<JointModel> joints_;
  std::vector<std::string> all_joints_;
  std::unordered_map<std::string, JointModelGroup> groups_;
  std::unordered_map<std::string, JointLimits> limits_;
  std::vector<std::pair<std::string, std::string>> disabled_collisions_;
  std::vector<SrdfEndEffector> end_effectors_;
  std::vector<std::string> passive_joints_;
  std::string urdf_path_;
  LinkForwardKinematicsTree link_tree_;
};

}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
