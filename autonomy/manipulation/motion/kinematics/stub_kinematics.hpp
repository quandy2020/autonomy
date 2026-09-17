/*
 * Copyright 2026 The Openbot Authors
 *
 * Stub FK/IK — FK returns identity tip; IK copies seed (bring-up only).
 */

#pragma once

#include <string>

#include "autonomy/manipulation/common/kinematics_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {

/**
 * @brief Bring-up FK/IK stub: FK returns identity; IK copies the seed state.
 */
class StubKinematics : public KinematicsBase {
 public:
  /**
   * @brief Store group / frame names (no kinematic model required).
   * @param[in] group Planning group name.
   * @param[in] base_frame Base frame name.
   * @param[in] tip_frame Tip frame name.
   * @return Always true.
   */
  bool Init(const std::string& group, const std::string& base_frame,
            const std::string& tip_frame) override;

  /**
   * @brief Write an identity tip pose.
   * @param[in] joints Ignored joint state.
   * @param[out] tip_pose Identity pose (must be non-null).
   * @return true if @p tip_pose is non-null.
   */
  bool GetPositionFK(const core::JointState& joints,
                     Pose* tip_pose) const override;

  /**
   * @brief Copy @p seed into @p solution (does not solve IK).
   * @param[in] tip_pose Ignored desired pose.
   * @param[in] seed Joint seed copied to the solution.
   * @param[in] options Ignored.
   * @param[out] solution Receives a copy of @p seed.
   * @return ErrorCode::kSuccess when @p solution is non-null.
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
