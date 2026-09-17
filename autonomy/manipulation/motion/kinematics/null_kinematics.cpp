/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/kinematics/null_kinematics.hpp"

#include "autonomy/common/logging.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {

bool NullKinematics::Init(const std::string& group,
                          const std::string& base_frame,
                          const std::string& tip_frame) {
  group_ = group;
  base_frame_ = base_frame;
  tip_frame_ = tip_frame;
  AWARN << "NullKinematics: FK/IK are placeholders group=" << group_;
  return true;
}

bool NullKinematics::GetPositionFK(const automsgs::msgs::sensor_msgs::JointState& /*joints*/,
                                   automsgs::msgs::geometry_msgs::Pose* tip_pose) const {
  if (!tip_pose) {
    return false;
  }
  *tip_pose = MakeIdentityPose();
  return true;
}

ErrorCode NullKinematics::GetPositionIK(const automsgs::msgs::geometry_msgs::Pose& /*tip_pose*/,
                                        const automsgs::msgs::sensor_msgs::JointState& seed,
                                        const InverseKinematicsOptions& /*options*/,
                                        automsgs::msgs::sensor_msgs::JointState* solution) const {
  if (!solution || seed.position_size() == 0) {
    return ErrorCode::NO_INVERSE_KINEMATICS_SOLUTION;
  }
  *solution = seed;
  return ErrorCode::SUCCESS;
}


AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(NullKinematics, KinematicsInterface);

}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
