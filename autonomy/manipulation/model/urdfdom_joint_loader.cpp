/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/model/urdfdom_joint_loader.hpp"

#include "autonomy/common/logging.hpp"

#if defined(AUTONOMY_HAS_URDFDOM)
#include <urdf_parser/urdf_parser.h>
#endif

namespace autonomy {
namespace manipulation {
namespace model {

bool LoadJointsFromUrdfPreferUrdfdom(const std::string& path,
                                     std::vector<JointModel>* joints,
                                     std::string* error) {
#if defined(AUTONOMY_HAS_URDFDOM)
  if (!joints) {
    return false;
  }
  joints->clear();
  const urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(path);
  if (!model) {
    AWARN << "urdfdom parse failed for " << path << "; falling back to regex";
    return LoadJointsFromUrdfFile(path, joints, error);
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
    JointModel info;
    info.set_name(j->name);
    if (j->type == urdf::Joint::REVOLUTE) {
      info.set_type("revolute");
    } else if (j->type == urdf::Joint::CONTINUOUS) {
      info.set_type("continuous");
    } else {
      info.set_type("prismatic");
    }
    if (j->parent_link_name.size()) {
      info.set_parent_link(j->parent_link_name);
    }
    if (j->child_link_name.size()) {
      info.set_child_link(j->child_link_name);
    }
    // urdf::Joint::axis is a value Vector3 (not a pointer).
    const double ax = j->axis.x;
    const double ay = j->axis.y;
    const double az = j->axis.z;
    if (ax * ax + ay * ay + az * az > 1e-18) {
      info.mutable_axis()->set_x(ax);
      info.mutable_axis()->set_y(ay);
      info.mutable_axis()->set_z(az);
    } else {
      info.mutable_axis()->set_z(1.0);
    }
    auto* limits = info.mutable_limits();
    limits->set_max_acceleration(2.0);
    if (j->type == urdf::Joint::CONTINUOUS) {
      limits->set_has_position_limits(false);
      limits->set_min_position(-1e9);
      limits->set_max_position(1e9);
      limits->set_max_velocity(1.0);
    } else if (j->limits) {
      limits->set_has_position_limits(true);
      limits->set_min_position(j->limits->lower);
      limits->set_max_position(j->limits->upper);
      limits->set_max_velocity(j->limits->velocity > 0 ? j->limits->velocity
                                                       : 1.0);
    }
    if (j->mimic) {
      info.set_mimic_joint(j->mimic->joint_name);
      info.set_mimic_factor(j->mimic->multiplier);
      info.set_mimic_offset(j->mimic->offset);
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
  return LoadJointsFromUrdfFile(path, joints, error);
#endif
}

}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
