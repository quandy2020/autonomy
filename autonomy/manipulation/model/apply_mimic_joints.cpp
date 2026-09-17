/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/model/apply_mimic_joints.hpp"

#include <unordered_map>

namespace autonomy {
namespace manipulation {
namespace model {

void ApplyMimicJoints(const std::vector<JointModel>& joints,
                      automsgs::msgs::sensor_msgs::JointState* joint_state) {
  if (!joint_state) {
    return;
  }
  std::unordered_map<std::string, double> positions_by_name;
  for (int i = 0;
       i < joint_state->name_size() && i < joint_state->position_size(); ++i) {
    positions_by_name[joint_state->name(i)] = joint_state->position(i);
  }
  for (const auto& joint : joints) {
    if (joint.mimic_joint().empty() ||
        !positions_by_name.count(joint.mimic_joint())) {
      continue;
    }
    positions_by_name[joint.name()] =
        joint.mimic_factor() * positions_by_name[joint.mimic_joint()] +
        joint.mimic_offset();
  }
  for (int i = 0;
       i < joint_state->name_size() && i < joint_state->position_size(); ++i) {
    const auto it = positions_by_name.find(joint_state->name(i));
    if (it != positions_by_name.end()) {
      joint_state->set_position(i, it->second);
    }
  }
  for (const auto& joint : joints) {
    if (joint.mimic_joint().empty() || !positions_by_name.count(joint.name())) {
      continue;
    }
    bool found = false;
    for (int k = 0; k < joint_state->name_size(); ++k) {
      if (joint_state->name(k) == joint.name()) {
        found = true;
        break;
      }
    }
    if (found) {
      continue;
    }
    joint_state->add_name(joint.name());
    joint_state->add_position(positions_by_name[joint.name()]);
  }
}

}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
