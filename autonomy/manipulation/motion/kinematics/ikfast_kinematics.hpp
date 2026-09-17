/*
 * Copyright 2026 The Openbot Authors
 *
 * IKFast plugin shell — replace SolveGenerated with codegen output.
 */

#pragma once

#include "autonomy/manipulation/common/kinematics_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {

/**
 * @brief IKFast plugin shell; wire robot-specific codegen before production use.
 */
class IkFastKinematics : public KinematicsInterface {
 public:
  /**
   * @brief Store group / frame names.
   * @param[in] group Planning group name.
   * @param[in] base_frame Base frame.
   * @param[in] tip_frame Tip frame.
   * @return true on successful init.
   */
  bool Init(const std::string& group, const std::string& base_frame,
            const std::string& tip_frame) override;

  /**
   * @brief FK placeholder until generated code is linked.
   * @return false (unimplemented).
   */
  bool GetPositionFK(const automsgs::msgs::sensor_msgs::JointState& joints,
                     automsgs::msgs::geometry_msgs::Pose* tip_pose) const override;

  /**
   * @brief IK placeholder until generated code is linked.
   * @return ErrorCode::NO_INVERSE_KINEMATICS_SOLUTION.
   */
  ErrorCode GetPositionIK(const automsgs::msgs::geometry_msgs::Pose& tip_pose, const automsgs::msgs::sensor_msgs::JointState& seed,
                          const InverseKinematicsOptions& options,
                          automsgs::msgs::sensor_msgs::JointState* solution) const override;

 private:
  std::string group_;
  std::string base_frame_;
  std::string tip_frame_;
};

}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
