/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/kinematics/ikfast_kinematics.hpp"

#include "autonomy/common/logging.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/model/joint_state_utilities.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {

bool IkFastKinematics::Init(const std::string& group,
                            const std::string& base_frame,
                            const std::string& tip_frame) {
  group_ = group;
  base_frame_ = base_frame;
  tip_frame_ = tip_frame;
  AWARN << "IkFastKinematics: interface placeholder (A+②); "
           "link generated IKFast sources for this robot to enable IK";
  return true;
}

bool IkFastKinematics::GetPositionFK(const automsgs::msgs::sensor_msgs::JointState& /*joints*/,
                                     automsgs::msgs::geometry_msgs::Pose* tip_pose) const {
  if (!tip_pose) {
    return false;
  }
  *tip_pose = MakeIdentityPose();
  return false;
}

ErrorCode IkFastKinematics::GetPositionIK(const automsgs::msgs::geometry_msgs::Pose& /*tip_pose*/,
                                          const automsgs::msgs::sensor_msgs::JointState& /*seed*/,
                                          const common::InverseKinematicsOptions& /*options*/,
                                          automsgs::msgs::sensor_msgs::JointState* /*solution*/) const {
  return ErrorCode::NO_INVERSE_KINEMATICS_SOLUTION;
}


AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(IkFastKinematics, common::KinematicsInterface);

}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
