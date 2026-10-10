/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file controller.hpp
 * @brief Kinematic MPC for a fixed-base arm, ported from the OCS2 formulation.
 *
 * OCS2 (Optimal Control for Switched Systems) models a fixed-base manipulator
 * as qdot-integrator dynamics: state x = q, input u = qdot. The stage cost is
 * a quadratic end-effector pose error plus input regularization, with joint
 * limits as a soft penalty. This class solves that problem with short-horizon
 * Gauss-Newton multiple shooting — the real-time form of OCS2's SQP on this
 * kinematic model. It does not vendor Pinocchio or CppAD.
 */

#ifndef AUTOMANIP_ARM_OSC2_CONTROLLER_HPP_
#define AUTOMANIP_ARM_OSC2_CONTROLLER_HPP_

#include "arm/chain.hpp"

#include <Eigen/Geometry>

namespace automanip {
namespace arm {
namespace osc2 {

/**
 * @brief Weights and horizon of the kinematic problem.
 */
struct Osc2Settings {
  /** @brief Number of constant-velocity intervals. */
  int horizon = 8;
  /** @brief Seconds per shooting interval. */
  double mpc_dt = 0.05;
  /** @brief Gauss-Newton iterations per control tick. */
  int iterations = 2;
  /** @brief Position error weight (m). */
  double position_weight = 40.0;
  /** @brief Orientation error weight (rad, rotation vector). */
  double orientation_weight = 5.0;
  /** @brief Joint-velocity regularization. */
  double input_weight = 0.02;
  /** @brief Joint-space tracking weight used by the home objective. */
  double joint_weight = 8.0;
  /** @brief Extra pull toward the home posture while tracking a pose. */
  double posture_weight = 0.2;
  /** @brief Multiplier on the final shooting node. */
  double terminal_scale = 4.0;
  /** @brief Soft joint-limit weight. */
  double limit_weight = 30.0;
  /** @brief Meters of margin inside the hard position limits. */
  double limit_margin = 0.02;
  /** @brief Levenberg-Marquardt damping on the normal equations. */
  double damping = 1e-3;
};

/**
 * @brief What the quadratic state cost tracks.
 */
enum class Osc2Objective {
  kPose,    ///< End-effector pose in the arm base frame.
  kJoints,  ///< Joint posture (home / joint target).
};

/**
 * @brief Real-time kinematic MPC. Compute() returns the first joint velocity.
 */
class Osc2Controller {
 public:
  Osc2Controller(SerialChain chain, Osc2Settings settings);

  const SerialChain& chain() const { return chain_; }
  const Osc2Settings& settings() const { return settings_; }

  /**
   * @brief Forget the previous input warm start.
   *
   * Does not change the pose or joint target. @p q is unused and kept so
   * callers can pass the state they just sampled.
   */
  void Reset(const Eigen::VectorXd& q);
  void SetObjective(Osc2Objective objective);
  void SetPoseTarget(const Eigen::Isometry3d& pose);
  void SetJointTarget(const Eigen::VectorXd& q);

  /**
   * @brief Solve one MPC tick.
   * @param[in] q Current joint position.
   * @return Joint velocity command (rad/s), size dof(). Empty if dof is 0.
   */
  Eigen::VectorXd Compute(const Eigen::VectorXd& q);

 private:
  SerialChain chain_;
  Osc2Settings settings_;
  Osc2Objective objective_ = Osc2Objective::kPose;
  Eigen::Isometry3d pose_target_ = Eigen::Isometry3d::Identity();
  Eigen::VectorXd joint_target_;
  Eigen::VectorXd u_prev_;
};

/**
 * @brief 6D pose error: position difference plus rotation vector of R_des R^T.
 */
Eigen::Matrix<double, 6, 1> PoseError(const Eigen::Isometry3d& current,
                                       const Eigen::Isometry3d& desired);

}  // namespace osc2
}  // namespace arm
}  // namespace automanip

#endif  // AUTOMANIP_ARM_OSC2_CONTROLLER_HPP_
