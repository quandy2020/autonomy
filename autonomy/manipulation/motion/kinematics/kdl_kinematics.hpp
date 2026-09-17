/*
 * Copyright 2026 The Openbot Authors
 *
 * Orocos KDL FK / IK (MoveIt kdl_kinematics_plugin analogue).
 */

#pragma once

#include <string>
#include <vector>

#include "autonomy/manipulation/model/urdf_kdl.hpp"
#include "autonomy/manipulation/common/kinematics_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {

/**
 * @brief Orocos KDL-based FK/IK for a URDF chain (requires AUTONOMY_HAS_KDL).
 */
class KdlKinematics : public KinematicsBase {
 public:
  /**
   * @brief Store group / frame names; call LoadUrdf before solving.
   * @param[in] group Planning group name.
   * @param[in] base_frame Chain base link.
   * @param[in] tip_frame Chain tip link.
   * @return true on successful init.
   */
  bool Init(const std::string& group, const std::string& base_frame,
            const std::string& tip_frame) override;

  /**
   * @brief Build the KDL chain from a URDF file using Init frames.
   * @param[in] urdf_path Path to URDF.
   * @return true if the chain was built.
   */
  bool LoadUrdf(const std::string& urdf_path);

  /**
   * @brief Forward kinematics via KDL.
   * @param[in] joints Joint configuration (mapped by name).
   * @param[out] tip_pose Computed tip pose.
   * @return true on success.
   */
  bool GetPositionFK(const core::JointState& joints,
                     Pose* tip_pose) const override;

  /**
   * @brief Inverse kinematics via KDL (with retries from @p options).
   * @param[in] tip_pose Desired tip pose.
   * @param[in] seed Seed joint state.
   * @param[in] options Timeout / attempts / position-only.
   * @param[out] solution Joint solution on success.
   * @return ErrorCode::kSuccess or ErrorCode::kNoIkSolution / failure.
   */
  ErrorCode GetPositionIK(const Pose& tip_pose, const core::JointState& seed,
                          const IkOptions& options,
                          core::JointState* solution) const override;

  /** @brief Movable joint names of the loaded chain. */
  const std::vector<std::string>& JointNames() const {
    return model_.joint_names;
  }

 private:
  bool MapJoints(const core::JointState& joints, KDL::JntArray* q) const;
  bool SolveOnce(const Pose& tip_pose, const KDL::JntArray& q_seed,
                 bool position_only, KDL::JntArray* q_out) const;

  std::string group_;
  std::string base_frame_;
  std::string tip_frame_;
  core::KdlChainModel model_;
  bool ready_ = false;
};

}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
