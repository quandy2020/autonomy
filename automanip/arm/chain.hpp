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
 * @file chain.hpp
 * @brief Serial-chain forward kinematics and geometric Jacobian.
 *
 * Product of joint translations and revolute rotations. The geometric
 * Jacobian is the linear map used by the OCS2-style kinematic MPC.
 */

#ifndef AUTOMANIP_ARM_CHAIN_HPP_
#define AUTOMANIP_ARM_CHAIN_HPP_

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace automanip {
namespace arm {

/**
 * @brief One revolute joint, expressed in its parent frame.
 */
struct JointSpec {
  std::string name;
  /** @brief Rotation axis in the parent frame (normalized at use). */
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  /** @brief Translation from the parent origin, parent frame, before rotation. */
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  double lower = -2.8;
  double upper = 2.8;
  double velocity_limit = 1.5;
  double home = 0.0;
};

/**
 * @brief Fixed-base serial arm.
 */
struct SerialChain {
  std::vector<JointSpec> joints;
  /** @brief Tool translation in the last joint frame, after the last rotation. */
  Eigen::Vector3d tool = Eigen::Vector3d::Zero();

  int dof() const { return static_cast<int>(joints.size()); }

  Eigen::VectorXd Home() const;
  Eigen::VectorXd Lower() const;
  Eigen::VectorXd Upper() const;
  Eigen::VectorXd VelocityLimit() const;

  /** @brief Clamp each joint into [lower, upper]. */
  void ClampPosition(Eigen::VectorXd* q) const;

  /** @brief Clamp each velocity into [-velocity_limit, velocity_limit]. */
  void ClampVelocity(Eigen::VectorXd* qdot) const;

  /**
   * @brief Tool pose in the arm base frame.
   * @param[in] q Joint positions, size dof().
   */
  Eigen::Isometry3d Forward(const Eigen::VectorXd& q) const;

  /**
   * @brief Spatial geometric Jacobian (6 x dof) of the tool pose.
   *
   * Top three rows are linear velocity, bottom three are angular velocity.
   * Columns are expressed in the arm base frame.
   */
  Eigen::MatrixXd Jacobian(const Eigen::VectorXd& q) const;
};

/**
 * @brief Built-in 6R tabletop arm used when YAML omits `joints`.
 *
 * At q = 0 the tool sits at approximately (0.51, 0, 0.65) m.
 */
SerialChain MakeDefaultArm();

}  // namespace arm
}  // namespace automanip

#endif  // AUTOMANIP_ARM_CHAIN_HPP_
