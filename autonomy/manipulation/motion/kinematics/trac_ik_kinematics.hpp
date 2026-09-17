/*
 * Copyright 2026 The Openbot Authors
 *
 * TRAC-IK-style multi-seed IK (real TRAC_IK FEATURE or KDL lite).
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/manipulation/common/kinematics_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {

/**
 * @brief TRAC-IK solver: AUTONOMY_HAS_TRAC_IK → library; else multi-seed KDL.
 */
class TracIkKinematics : public KinematicsBase {
 public:
  bool Init(const std::string& group, const std::string& base_frame,
            const std::string& tip_frame) override;

  bool LoadUrdf(const std::string& urdf_path);

  bool GetPositionFK(const core::JointState& joints,
                     Pose* tip_pose) const override;

  ErrorCode GetPositionIK(const Pose& tip_pose, const core::JointState& seed,
                          const IkOptions& options,
                          core::JointState* solution) const override;

  /** @brief true when compiled/linked with AUTONOMY_HAS_TRAC_IK. */
  static bool HasTracIkLibrary();

 private:
  std::string group_;
  std::string base_frame_;
  std::string tip_frame_;
  std::string urdf_path_;
  std::shared_ptr<KinematicsBase> inner_;  // KDL FK / lite IK fallback
  struct TracIkBackend;
  std::shared_ptr<TracIkBackend> backend_;
};

}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
