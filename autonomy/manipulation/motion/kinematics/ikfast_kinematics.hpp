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
class IkFastKinematics : public KinematicsBase {
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
  bool GetPositionFK(const core::JointState& joints,
                     Pose* tip_pose) const override;

  /**
   * @brief IK placeholder until generated code is linked.
   * @return ErrorCode::kNoIkSolution.
   */
  ErrorCode GetPositionIK(const Pose& tip_pose, const core::JointState& seed,
                          const IkOptions& options,
                          core::JointState* solution) const override;

 private:
  std::string group_;
  std::string base_frame_;
  std::string tip_frame_;
};

}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
