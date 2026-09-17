/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/model/urdfdom_loader.hpp"

#include "autonomy/common/logging.hpp"

#if defined(AUTONOMY_HAS_URDFDOM)
#include <urdf_parser/urdf_parser.h>
#endif

namespace autonomy {
namespace manipulation {
namespace core {

bool LoadUrdfJointsPreferred(const std::string& path,
                             std::vector<UrdfJointInfo>* joints,
                             std::string* error) {
#if defined(AUTONOMY_HAS_URDFDOM)
  if (!joints) {
    return false;
  }
  joints->clear();
  const urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(path);
  if (!model) {
    AWARN << "urdfdom parse failed for " << path << "; falling back to regex";
    return LoadUrdfJoints(path, joints, error);
  }
  for (const auto& kv : model->joints_) {
    const urdf::JointConstSharedPtr& j = kv.second;
    if (!j) {
      continue;
    }
    if (j->type != urdf::Joint::REVOLUTE && j->type != urdf::Joint::CONTINUOUS &&
        j->type != urdf::Joint::PRISMATIC) {
      continue;
    }
    UrdfJointInfo info;
    info.name = j->name;
    if (j->type == urdf::Joint::REVOLUTE) {
      info.type = "revolute";
    } else if (j->type == urdf::Joint::CONTINUOUS) {
      info.type = "continuous";
    } else {
      info.type = "prismatic";
    }
    if (j->parent_link_name.size()) {
      info.parent_link = j->parent_link_name;
    }
    if (j->child_link_name.size()) {
      info.child_link = j->child_link_name;
    }
    if (j->axis) {
      info.axis_x = j->axis->x;
      info.axis_y = j->axis->y;
      info.axis_z = j->axis->z;
    }
    if (j->type == urdf::Joint::CONTINUOUS) {
      info.has_position_limits = false;
    } else if (j->limits) {
      info.has_position_limits = true;
      info.lower = j->limits->lower;
      info.upper = j->limits->upper;
      info.velocity = j->limits->velocity > 0 ? j->limits->velocity : 1.0;
      info.effort = j->limits->effort;
    }
    if (j->mimic) {
      info.mimic_joint = j->mimic->joint_name;
      info.mimic_multiplier = j->mimic->multiplier;
      info.mimic_offset = j->mimic->offset;
    }
    joints->push_back(std::move(info));
  }
  if (joints->empty()) {
    if (error) {
      *error = "urdfdom: no movable joints in " + path;
    }
    return false;
  }
  AINFO << "urdfdom loaded joints=" << joints->size() << " from " << path;
  return true;
#else
  return LoadUrdfJoints(path, joints, error);
#endif
}

}  // namespace core
}  // namespace manipulation
}  // namespace autonomy
