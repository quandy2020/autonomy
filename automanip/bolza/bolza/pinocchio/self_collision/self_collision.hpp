/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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

#pragma once

/**
 * @file self_collision.hpp
 * @brief Constructor.
 */


#include "bolza/pinocchio/interface/pinocchio_interface.hpp"
#include "bolza/pinocchio/self_collision/pinocchio_geometry_interface.hpp"

namespace bolza {

class SelfCollision {
 public:
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

  /**
   * @brief Constructor.
   * @param [in] pinocchioGeometryInterface: pinocchio geometry interface of the robot model
   * @param [in] minimumDistance: Minimum Distance.
   */
  SelfCollision(PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance);

  /**
   * @brief Get the number of collision pairs.
   * @return Result value.
   */
  size_t GetNumCollisionPairs() const { return pinocchioGeometryInterface_.getNumCollisionPairs(); }

  /**
   * Evaluate the distance violation
   * This method computes the distance results of all collision pairs through PinocchioGeometryInterface
   * and compare each of them with the specified minimum distance.
   *
   * @note Requires updated forwardKinematics() on pinocchioInterface.
   *
   * @param [in] pinocchioInterface: pinocchio interface of the robot model
   * @return The differences between the distance of each collision pair and the minimum distance
   */
  vector_t GetValue(const PinocchioInterface& pinocchioInterface) const;

  /**
   * Evaluate the linear approximation of the distance function
   * This method analytically computes the first derivative of distance against the pinocchio generalized coordinates
   *
   * @note Requires updated forwardKinematics(), updateGlobalPlacements() and computeJointJacobians() on pinocchioInterface.
   *
   * @param [in] pinocchioInterface: pinocchio interface of the robot model
   * @param [in] pinocchioGeometryInterface: pinocchio geometry interface of the robot model
   * @return The pair of the distance violation and the first derivative of the distance against q
   */
  std::pair<vector_t, matrix_t> GetLinearApproximation(const PinocchioInterface& pinocchioInterface) const;

 private:
  PinocchioGeometryInterface pinocchioGeometryInterface_;
  scalar_t minimumDistance_;
};

}  // namespace bolza
