/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/kinematics/stub_kinematics.hpp"

#include "autonomy/common/logging.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {

bool StubKinematics::Init(const std::string& group,
                          const std::string& base_frame,
                          const std::string& tip_frame) {
  group_ = group;
  base_frame_ = base_frame;
  tip_frame_ = tip_frame;
  AWARN << "StubKinematics: FK/IK are placeholders group=" << group_;
  return true;
}

bool StubKinematics::GetPositionFK(const core::JointState& /*joints*/,
                                   Pose* tip_pose) const {
  if (!tip_pose) {
    return false;
  }
  *tip_pose = Pose{};
  tip_pose->qw = 1.0;
  return true;
}

ErrorCode StubKinematics::GetPositionIK(const Pose& /*tip_pose*/,
                                        const core::JointState& seed,
                                        const IkOptions& /*options*/,
                                        core::JointState* solution) const {
  if (!solution || seed.positions.empty()) {
    return ErrorCode::kNoIkSolution;
  }
  *solution = seed;
  return ErrorCode::kSuccess;
}


AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(StubKinematics, KinematicsBase);

}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
