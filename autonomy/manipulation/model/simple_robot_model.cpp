/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/model/simple_robot_model.hpp"
#include "autonomy/manipulation/model/simple_robot_state.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/model/urdf_joint_loader.hpp"
#include "autonomy/manipulation/model/urdfdom_joint_loader.hpp"

namespace autonomy {
namespace manipulation {
namespace model {

bool SimpleRobotModel::Load(const std::string& urdf, const std::string& srdf) {
  all_joints_.clear();
  joints_.clear();
  groups_.clear();
  limits_.clear();
  disabled_collisions_.clear();
  end_effectors_.clear();
  passive_joints_.clear();
  urdf_path_.clear();
  link_tree_ = LinkForwardKinematicsTree{};

  if (urdf.empty()) {
    AWARN << "SimpleRobotModel: empty URDF; use SetGroupJoints()";
    return true;
  }
  urdf_path_ = urdf;

  std::vector<JointModel> urdf_joints;
  std::string error;
  if (!LoadJointsFromUrdfPreferUrdfdom(urdf, &urdf_joints, &error)) {
    AERROR << "SimpleRobotModel: " << error;
    return false;
  }

  joints_.reserve(urdf_joints.size());
  all_joints_.reserve(urdf_joints.size());
  for (auto& jm : urdf_joints) {
    limits_[jm.name()] = jm.limits();
    all_joints_.push_back(jm.name());
    joints_.push_back(std::move(jm));
  }

  JointModelGroup all;
  all.set_name("all");
  for (const auto& name : all_joints_) {
    all.add_joint_names(name);
  }
  for (const auto& j : joints_) {
    if (!j.child_link().empty()) {
      all.add_link_names(j.child_link());
    }
    if (!j.parent_link().empty()) {
      bool found = false;
      for (int i = 0; i < all.link_names_size(); ++i) {
        if (all.link_names(i) == j.parent_link()) {
          found = true;
          break;
        }
      }
      if (!found) {
        // Prepend parent by rebuilding — keep first parent as base.
        std::vector<std::string> links;
        links.push_back(j.parent_link());
        for (int i = 0; i < all.link_names_size(); ++i) {
          links.push_back(all.link_names(i));
        }
        all.clear_link_names();
        for (const auto& ln : links) {
          all.add_link_names(ln);
        }
      }
    }
  }
  if (all.link_names_size() > 0) {
    all.set_base_frame(all.link_names(0));
    all.set_tip_frame(all.link_names(all.link_names_size() - 1));
  }
  all.set_is_chain(true);
  groups_["all"] = all;

  if (!srdf.empty()) {
    std::unordered_map<std::string, JointModelGroup> srdf_groups;
    if (LoadPlanningGroupsFromSrdf(srdf, &srdf_groups, &error)) {
      for (auto& kv : srdf_groups) {
        groups_[kv.first] = std::move(kv.second);
      }
    } else {
      AWARN << "SimpleRobotModel: SRDF load skipped: " << error;
    }
    error.clear();
    if (!LoadDisabledCollisionsFromSrdf(srdf, &disabled_collisions_, &error)) {
      AWARN << "SimpleRobotModel: disable_collisions skipped: " << error;
      disabled_collisions_.clear();
    }
    error.clear();
    if (!LoadEndEffectorsFromSrdf(srdf, &end_effectors_, &error)) {
      AWARN << "SimpleRobotModel: end_effectors skipped: " << error;
      end_effectors_.clear();
    }
    error.clear();
    if (!LoadPassiveJointsFromSrdf(srdf, &passive_joints_, &error)) {
      AWARN << "SimpleRobotModel: passive_joints skipped: " << error;
      passive_joints_.clear();
    }
  }

  AINFO << "SimpleRobotModel: loaded " << all_joints_.size()
        << " movable joints from " << urdf
        << " acm_pairs=" << disabled_collisions_.size();
  if (!link_tree_.LoadFromUrdfFile(urdf, &error)) {
    AWARN << "SimpleRobotModel: LinkForwardKinematicsTree load skipped: "
          << error;
  }
  return true;
}

std::vector<std::string> SimpleRobotModel::GetJointNames(
    const std::string& group) const {
  const auto it = groups_.find(group);
  if (it == groups_.end()) {
    return {};
  }
  std::vector<std::string> names;
  names.reserve(static_cast<std::size_t>(it->second.joint_names_size()));
  for (int i = 0; i < it->second.joint_names_size(); ++i) {
    names.push_back(it->second.joint_names(i));
  }
  return names;
}

void SimpleRobotModel::SetGroupJoints(const std::string& group,
                                      std::vector<std::string> joint_names) {
  JointModelGroup g;
  auto it = groups_.find(group);
  if (it != groups_.end()) {
    g = it->second;
  }
  g.set_name(group);
  g.clear_joint_names();
  for (const auto& n : joint_names) {
    g.add_joint_names(n);
  }
  groups_[group] = std::move(g);
}

void SimpleRobotModel::SetGroupChainFrames(const std::string& group,
                                           const std::string& base_frame,
                                           const std::string& tip_frame) {
  auto& g = groups_[group];
  g.set_name(group);
  g.set_base_frame(base_frame);
  g.set_tip_frame(tip_frame);
  g.set_is_chain(true);
}

const JointModelGroup* SimpleRobotModel::GetJointModelGroup(
    const std::string& name) const {
  const auto it = groups_.find(name);
  if (it == groups_.end()) {
    return nullptr;
  }
  return &it->second;
}

const JointLimits* SimpleRobotModel::GetJointLimits(
    const std::string& joint_name) const {
  const auto it = limits_.find(joint_name);
  if (it == limits_.end()) {
    return nullptr;
  }
  return &it->second;
}

}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
