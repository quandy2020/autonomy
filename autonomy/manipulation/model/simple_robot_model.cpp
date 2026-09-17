/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/model/simple_robot_model.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/model/link_fk.hpp"
#include "autonomy/manipulation/model/srdf_groups.hpp"
#include "autonomy/manipulation/model/urdf_joints.hpp"
#include "autonomy/manipulation/model/urdfdom_loader.hpp"

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
  end_effectors_.clear();
  passive_joints_.clear();
  urdf_path_.clear();
  link_tree_ = LinkFkTree{};

  if (urdf.empty()) {
    AWARN << "SimpleRobotModel: empty URDF; use SetGroupJoints()";
    return true;
  }
  urdf_path_ = urdf;

  std::vector<UrdfJointInfo> urdf_joints;
  std::string error;
  if (!LoadUrdfJointsPreferred(urdf, &urdf_joints, &error)) {
    AERROR << "SimpleRobotModel: " << error;
    return false;
  }

  joints_.reserve(urdf_joints.size());
  all_joints_.reserve(urdf_joints.size());
  for (const auto& j : urdf_joints) {
    JointModel jm;
    jm.name = j.name;
    jm.type = j.type;
    jm.parent_link = j.parent_link;
    jm.child_link = j.child_link;
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
  for (const auto& j : joints_) {
    if (!j.child_link.empty()) {
      all.link_names.push_back(j.child_link);
    }
    if (!j.parent_link.empty() &&
        (all.link_names.empty() || all.link_names.front() != j.parent_link)) {
      // Keep unique base as first parent if not already listed.
      bool found = false;
      for (const auto& ln : all.link_names) {
        if (ln == j.parent_link) {
          found = true;
          break;
        }
      }
      if (!found) {
        all.link_names.insert(all.link_names.begin(), j.parent_link);
      }
    }
  }
  if (!all.link_names.empty()) {
    all.base_frame = all.link_names.front();
    all.tip_frame = all.link_names.back();
  }
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
    error.clear();
    if (!LoadSrdfEndEffectors(srdf, &end_effectors_, &error)) {
      AWARN << "SimpleRobotModel: end_effectors skipped: " << error;
      end_effectors_.clear();
    }
    error.clear();
    if (!LoadSrdfPassiveJoints(srdf, &passive_joints_, &error)) {
      AWARN << "SimpleRobotModel: passive_joints skipped: " << error;
      passive_joints_.clear();
    }
  }

  AINFO << "SimpleRobotModel: loaded " << all_joints_.size()
        << " movable joints from " << urdf
        << " acm_pairs=" << disabled_collisions_.size();
  if (!link_tree_.LoadUrdf(urdf, &error)) {
    AWARN << "SimpleRobotModel: LinkFkTree load skipped: " << error;
  }
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
    state_.clear_position();
    for (double q : positions) {
      state_.add_position(q);
    }
    return;
  }
  const auto names = model_->GetJointNames(group);
  if (names.size() != positions.size()) {
    return;
  }
  if (state_.name_size() == 0) {
    ::autonomy::manipulation::SetJointState(&state_, names, positions);
    return;
  }
  for (std::size_t i = 0; i < names.size(); ++i) {
    int found = -1;
    for (int j = 0; j < state_.name_size(); ++j) {
      if (state_.name(j) == names[i]) {
        found = j;
        break;
      }
    }
    if (found < 0) {
      state_.add_name(names[i]);
      state_.add_position(positions[i]);
    } else if (found < state_.position_size()) {
      state_.set_position(found, positions[i]);
    }
  }
}

std::vector<double> SimpleRobotState::GetJointGroupPositions(
    const std::string& group) const {
  std::shared_lock lock(mutex_);
  if (!model_) {
    return std::vector<double>(state_.position().begin(),
                               state_.position().end());
  }
  const auto names = model_->GetJointNames(group);
  std::vector<double> out(names.size(), 0.0);
  for (std::size_t i = 0; i < names.size(); ++i) {
    for (int j = 0; j < state_.name_size(); ++j) {
      if (state_.name(j) == names[i] && j < state_.position_size()) {
        out[i] = state_.position(j);
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
  const int n = std::min(a.position_size(), b.position_size());
  while (out.position_size() > n) {
    out.mutable_position()->RemoveLast();
  }
  while (out.position_size() < n) {
    out.add_position(0.0);
  }
  for (int i = 0; i < n; ++i) {
    out.set_position(i, a.position(i) * (1.0 - s) + b.position(i) * s);
  }
  result->SetJointState(out);
}

void SimpleRobotState::EnforceBounds() {
  std::unique_lock lock(mutex_);
  if (!model_) {
    return;
  }
  for (int i = 0; i < state_.name_size(); ++i) {
    if (i >= state_.position_size()) {
      break;
    }
    const auto* lim = model_->GetJointLimits(state_.name(i));
    if (!lim || !lim->has_position_limits) {
      continue;
    }
    state_.set_position(
        i, std::clamp(state_.position(i), lim->min_position,
                      lim->max_position));
  }
}

bool SimpleRobotState::SatisfiesBounds() const {
  std::shared_lock lock(mutex_);
  if (!model_) {
    return true;
  }
  for (int i = 0; i < state_.name_size(); ++i) {
    if (i >= state_.position_size()) {
      break;
    }
    const auto* lim = model_->GetJointLimits(state_.name(i));
    if (!lim || !lim->has_position_limits) {
      continue;
    }
    const double q = state_.position(i);
    if (q < lim->min_position || q > lim->max_position) {
      return false;
    }
  }
  return true;
}

double SimpleRobotState::Distance(const RobotState& other) const {
  const JointState a = GetJointState();
  const JointState b = other.GetJointState();
  const int n = std::min(a.position_size(), b.position_size());
  double sum = 0.0;
  for (int i = 0; i < n; ++i) {
    const double d = a.position(i) - b.position(i);
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
