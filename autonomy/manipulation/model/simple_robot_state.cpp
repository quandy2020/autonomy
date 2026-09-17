/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/model/simple_robot_state.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/manipulation/model/apply_mimic_joints.hpp"
#include "autonomy/manipulation/model/joint_state_utilities.hpp"

namespace autonomy {
namespace manipulation {
namespace model {

SimpleRobotState::SimpleRobotState(
    std::shared_ptr<const SimpleRobotModel> model)
    : model_(std::move(model)) {}

void SimpleRobotState::SetModel(std::shared_ptr<const SimpleRobotModel> model) {
  std::unique_lock lock(mutex_);
  model_ = std::move(model);
}

void SimpleRobotState::SetJointState(
    const automsgs::msgs::sensor_msgs::JointState& state) {
  std::unique_lock lock(mutex_);
  state_ = state;
  if (model_) {
    ApplyMimicJoints(model_->Joints(), &state_);
  }
}

automsgs::msgs::sensor_msgs::JointState SimpleRobotState::GetJointState()
    const {
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
  const automsgs::msgs::sensor_msgs::JointState a = GetJointState();
  const automsgs::msgs::sensor_msgs::JointState b = to.GetJointState();
  automsgs::msgs::sensor_msgs::JointState out = a;
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
    if (!lim || !lim->has_position_limits()) {
      continue;
    }
    state_.set_position(
        i, std::clamp(state_.position(i), lim->min_position(),
                      lim->max_position()));
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
    if (!lim || !lim->has_position_limits()) {
      continue;
    }
    const double q = state_.position(i);
    if (q < lim->min_position() || q > lim->max_position()) {
      return false;
    }
  }
  return true;
}

double SimpleRobotState::Distance(const RobotState& other) const {
  const automsgs::msgs::sensor_msgs::JointState a = GetJointState();
  const automsgs::msgs::sensor_msgs::JointState b = other.GetJointState();
  const int n = std::min(a.position_size(), b.position_size());
  double sum = 0.0;
  for (int i = 0; i < n; ++i) {
    const double d = a.position(i) - b.position(i);
    sum += d * d;
  }
  return std::sqrt(sum);
}

bool SimpleRobotState::LoadLinkForwardKinematicsTree(
    const std::string& urdf_path) {
  std::unique_lock lock(mutex_);
  std::string error;
  return link_tree_.LoadFromUrdfFile(urdf_path, &error);
}

bool SimpleRobotState::GetLinkPose(
    const std::string& link_name,
    automsgs::msgs::geometry_msgs::Pose* link_pose) const {
  std::shared_lock lock(mutex_);
  return link_tree_.GetLinkPose(state_, link_name, link_pose);
}

bool SimpleRobotState::ComputeLinkPoses(
    std::unordered_map<std::string, automsgs::msgs::geometry_msgs::Pose>*
        link_poses) const {
  std::shared_lock lock(mutex_);
  return link_tree_.ComputeAllLinkPoses(state_, link_poses);
}

}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
