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
class TracIkKinematics : public KinematicsInterface {
 public:
  bool Init(const std::string& group, const std::string& base_frame,
            const std::string& tip_frame) override;

  bool LoadUrdf(const std::string& urdf_path);

  bool GetPositionFK(const automsgs::msgs::sensor_msgs::JointState& joints,
                     automsgs::msgs::geometry_msgs::Pose* tip_pose) const override;

  ErrorCode GetPositionIK(const automsgs::msgs::geometry_msgs::Pose& tip_pose, const automsgs::msgs::sensor_msgs::JointState& seed,
                          const InverseKinematicsOptions& options,
                          automsgs::msgs::sensor_msgs::JointState* solution) const override;

  /** @brief true when compiled/linked with AUTONOMY_HAS_TRAC_IK. */
  static bool HasTracIkLibrary();

 private:
  std::string group_;
  std::string base_frame_;
  std::string tip_frame_;
  std::string urdf_path_;
  KinematicsInterface::SharedPtr inner_;  // KDL FK / lite IK fallback
  struct TracIkBackend;
  std::shared_ptr<TracIkBackend> backend_;
};

}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
