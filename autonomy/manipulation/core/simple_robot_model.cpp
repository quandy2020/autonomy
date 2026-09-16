/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/core/simple_robot_model.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/core/link_fk.hpp"
#include "autonomy/manipulation/core/srdf_groups.hpp"
#include "autonomy/manipulation/core/urdf_joints.hpp"

namespace autonomy {
namespace manipulation {
namespace core {
namespace {

JointLimits LimitsFromUrdf(const UrdfJointInfo& j) {
  JointLimits lim;
  lim.has_position_limits = j.has_position_limits;
  lim.min_position = j.lower;
  lim.max_position = j.upper;
  lim.max_velocity = j.velocity > 0.0 ? j.velocity : 1.0;
  if (j.type == "continuous") {
    lim.has_position_limits = false;
    lim.min_position = -1e9;
    lim.max_position = 1e9;
  }
  return lim;
}

}  // namespace

bool SimpleRobotModel::Load(const std::string& urdf, const std::string& srdf) {
  all_joints_.clear();
  joints_.clear();
  groups_.clear();
  limits_.clear();
  disabled_collisions_.clear();

  if (urdf.empty()) {
    AWARN << "SimpleRobotModel: empty URDF; use SetGroupJoints()";
    return true;
  }

  std::vector<UrdfJointInfo> urdf_joints;
  std::string error;
  if (!LoadUrdfJoints(urdf, &urdf_joints, &error)) {
    AERROR << "SimpleRobotModel: " << error;
    return false;
  }

  joints_.reserve(urdf_joints.size());
  all_joints_.reserve(urdf_joints.size());
  for (const auto& j : urdf_joints) {
    JointModel jm;
    jm.name = j.name;
    jm.type = j.type;
    jm.limits = LimitsFromUrdf(j);
    jm.mimic_joint = j.mimic_joint;
    jm.mimic_factor = j.mimic_multiplier;
    jm.mimic_offset = j.mimic_offset;
    limits_[jm.name] = jm.limits;
    all_joints_.push_back(jm.name);
    joints_.push_back(std::move(jm));
  }

  JointModelGroup all;
  all.name = "all";
  all.joint_names = all_joints_;
  groups_["all"] = all;

  if (!srdf.empty()) {
    std::unordered_map<std::string, JointModelGroup> srdf_groups;
    if (LoadSrdfGroups(srdf, &srdf_groups, &error)) {
      for (auto& kv : srdf_groups) {
        groups_[kv.first] = std::move(kv.second);
      }
    } else {
      AWARN << "SimpleRobotModel: SRDF load skipped: " << error;
    }
    error.clear();
    if (!LoadSrdfDisableCollisions(srdf, &disabled_collisions_, &error)) {
      AWARN << "SimpleRobotModel: disable_collisions skipped: " << error;
      disabled_collisions_.clear();
    }
  }

  AINFO << "SimpleRobotModel: loaded " << all_joints_.size()
        << " movable joints from " << urdf
        << " acm_pairs=" << disabled_collisions_.size();
  return true;
}

std::vector<std::string> SimpleRobotModel::GetJointNames(
    const std::string& group) const {
  const auto it = groups_.find(group);
  if (it == groups_.end()) {
    return {};
  }
  return it->second.joint_names;
}

void SimpleRobotModel::SetGroupJoints(const std::string& group,
                                      std::vector<std::string> joint_names) {
  JointModelGroup g;
  auto it = groups_.find(group);
  if (it != groups_.end()) {
    g = it->second;
  }
  g.name = group;
  g.joint_names = std::move(joint_names);
  groups_[group] = std::move(g);
}

void SimpleRobotModel::SetGroupMeta(const std::string& group,
                                    const std::string& base,
                                    const std::string& tip) {
  auto& g = groups_[group];
  g.name = group;
  g.base_frame = base;
  g.tip_frame = tip;
  g.is_chain = true;
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

SimpleRobotState::SimpleRobotState(
    std::shared_ptr<const SimpleRobotModel> model)
    : model_(std::move(model)) {}

void SimpleRobotState::SetModel(std::shared_ptr<const SimpleRobotModel> model) {
  std::unique_lock lock(mutex_);
  model_ = std::move(model);
}

void SimpleRobotState::SetJointState(const JointState& state) {
  std::unique_lock lock(mutex_);
  state_ = state;
  if (model_) {
    ApplyMimicJoints(model_->Joints(), &state_);
  }
}

JointState SimpleRobotState::GetJointState() const {
  std::shared_lock lock(mutex_);
  return state_;
}

void SimpleRobotState::SetJointGroupPositions(
    const std::string& group, const std::vector<double>& positions) {
  std::unique_lock lock(mutex_);
  if (!model_) {
    state_.positions = positions;
    return;
  }
  const auto names = model_->GetJointNames(group);
  if (names.size() != positions.size()) {
    return;
  }
  if (state_.names.empty()) {
    state_.names = names;
    state_.positions = positions;
    return;
  }
  for (std::size_t i = 0; i < names.size(); ++i) {
    auto it = std::find(state_.names.begin(), state_.names.end(), names[i]);
    if (it == state_.names.end()) {
      state_.names.push_back(names[i]);
      state_.positions.push_back(positions[i]);
    } else {
      const auto idx =
          static_cast<std::size_t>(std::distance(state_.names.begin(), it));
      if (idx < state_.positions.size()) {
        state_.positions[idx] = positions[i];
      }
    }
  }
}

std::vector<double> SimpleRobotState::GetJointGroupPositions(
    const std::string& group) const {
  std::shared_lock lock(mutex_);
  if (!model_) {
    return state_.positions;
  }
  const auto names = model_->GetJointNames(group);
  std::vector<double> out(names.size(), 0.0);
  for (std::size_t i = 0; i < names.size(); ++i) {
    for (std::size_t j = 0; j < state_.names.size(); ++j) {
      if (state_.names[j] == names[i] && j < state_.positions.size()) {
        out[i] = state_.positions[j];
        break;
      }
    }
  }
  return out;
}

void SimpleRobotState::Interpolate(const RobotState& to, double t,
                                   RobotState* result) const {
  if (!result) {
    return;
  }
  const double s = std::clamp(t, 0.0, 1.0);
  const JointState a = GetJointState();
  const JointState b = to.GetJointState();
  JointState out = a;
  const std::size_t n = std::min(a.positions.size(), b.positions.size());
  out.positions.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    out.positions[i] = a.positions[i] * (1.0 - s) + b.positions[i] * s;
  }
  result->SetJointState(out);
}

void SimpleRobotState::EnforceBounds() {
  std::unique_lock lock(mutex_);
  if (!model_) {
    return;
  }
  for (std::size_t i = 0; i < state_.names.size(); ++i) {
    if (i >= state_.positions.size()) {
      break;
    }
    const auto* lim = model_->GetJointLimits(state_.names[i]);
    if (!lim || !lim->has_position_limits) {
      continue;
    }
    state_.positions[i] =
        std::clamp(state_.positions[i], lim->min_position, lim->max_position);
  }
}

bool SimpleRobotState::SatisfiesBounds() const {
  std::shared_lock lock(mutex_);
  if (!model_) {
    return true;
  }
  for (std::size_t i = 0; i < state_.names.size(); ++i) {
    if (i >= state_.positions.size()) {
      break;
    }
    const auto* lim = model_->GetJointLimits(state_.names[i]);
    if (!lim || !lim->has_position_limits) {
      continue;
    }
    const double q = state_.positions[i];
    if (q < lim->min_position || q > lim->max_position) {
      return false;
    }
  }
  return true;
}

double SimpleRobotState::Distance(const RobotState& other) const {
  const JointState a = GetJointState();
  const JointState b = other.GetJointState();
  const std::size_t n = std::min(a.positions.size(), b.positions.size());
  double sum = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const double d = a.positions[i] - b.positions[i];
    sum += d * d;
  }
  return std::sqrt(sum);
}

bool SimpleRobotState::LoadLinkTree(const std::string& urdf_path) {
  std::unique_lock lock(mutex_);
  std::string error;
  return link_tree_.LoadUrdf(urdf_path, &error);
}

bool SimpleRobotState::GetLinkPose(const std::string& link,
                                   Transform* pose) const {
  std::shared_lock lock(mutex_);
  return link_tree_.GetLinkPose(state_, link, pose);
}

bool SimpleRobotState::ComputeLinkPoses(
    std::unordered_map<std::string, Transform>* poses) const {
  std::shared_lock lock(mutex_);
  return link_tree_.Compute(state_, poses);
}

}  // namespace core
}  // namespace manipulation
}  // namespace autonomy
