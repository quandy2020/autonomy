/*
 * Copyright 2026 The Openbot Authors
 *
 * FK / IK plugin base (MoveIt moveit_kinematics analogue).
 */

#pragma once

#include <string>
#include <vector>

#include "autonomy/manipulation/core/error_codes.hpp"
#include "autonomy/manipulation/core/robot_model.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {

/** @brief End-effector pose as translation plus quaternion (x,y,z, qx,qy,qz,qw). */
struct Pose {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
};

/** @brief Options for iterative / multi-attempt IK solves. */
struct IkOptions {
  double timeout = 0.05;  // s
  bool position_only = false;
  int max_attempts = 8;
  std::vector<double> consistency_limits;  // per-joint, empty = unlimited
};

/**
 * @brief Abstract FK/IK solver plugin for a planning group chain.
 */
class KinematicsBase {
 public:
  virtual ~KinematicsBase() = default;

  /**
   * @brief Bind the solver to a planning group and chain frames.
   * @param[in] group Planning group name.
   * @param[in] base_frame Base / root frame of the chain.
   * @param[in] tip_frame Tip / end-effector frame.
   * @return true on successful initialization.
   */
  virtual bool Init(const std::string& group, const std::string& base_frame,
                    const std::string& tip_frame) = 0;

  /**
   * @brief Forward kinematics: joint state → tip pose.
   * @param[in] joints Joint configuration.
   * @param[out] tip_pose Computed tip pose (must be non-null).
   * @return true on success.
   */
  virtual bool GetPositionFK(const core::JointState& joints,
                             Pose* tip_pose) const = 0;

  /**
   * @brief Inverse kinematics with default @ref IkOptions.
   * @param[in] tip_pose Desired tip pose.
   * @param[in] seed Seed joint state.
   * @param[out] solution Joint solution on success.
   * @return true if @ref ErrorCode::kSuccess.
   */
  virtual bool GetPositionIK(const Pose& tip_pose,
                             const core::JointState& seed,
                             core::JointState* solution) const {
    return GetPositionIK(tip_pose, seed, IkOptions{}, solution) ==
           ErrorCode::kSuccess;
  }

  /**
   * @brief Inverse kinematics with explicit options.
   * @param[in] tip_pose Desired tip pose.
   * @param[in] seed Seed joint state.
   * @param[in] options Timeout, position-only, attempts, consistency limits.
   * @param[out] solution Joint solution on success.
   * @return ErrorCode describing success or failure mode.
   */
  virtual ErrorCode GetPositionIK(const Pose& tip_pose,
                                  const core::JointState& seed,
                                  const IkOptions& options,
                                  core::JointState* solution) const = 0;
};

}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
