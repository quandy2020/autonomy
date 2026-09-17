/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/kinematics/ikfast_kinematics.hpp"

#include "autonomy/common/logging.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"

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

bool IkFastKinematics::GetPositionFK(const core::JointState& /*joints*/,
                                     Pose* tip_pose) const {
  if (!tip_pose) {
    return false;
  }
  *tip_pose = MakeIdentityPose();
  return false;
}

ErrorCode IkFastKinematics::GetPositionIK(const Pose& /*tip_pose*/,
                                          const core::JointState& /*seed*/,
                                          const IkOptions& /*options*/,
                                          core::JointState* /*solution*/) const {
  return ErrorCode::kNoIkSolution;
}


AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(IkFastKinematics, KinematicsBase);

}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
