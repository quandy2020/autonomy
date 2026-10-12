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
 * @file self_collision_constraint.hpp
 * @brief This class provides a variant of the Self-collision constraints, which allows for caching. Therefore It is the user's.
 */


#include <memory>

#include <bolza/core/constraint/state_constraint.hpp>
#include "bolza/pinocchio/interface/pinocchio_state_input_mapping.hpp"
#include "bolza/pinocchio/self_collision/self_collision.hpp"

namespace bolza {

/**
 *  This class provides a variant of the Self-collision constraints, which allows for caching. Therefore It is the user's
 *  responsibility to call the required updates on the PinocchioInterface in pre-computation requests.
 */
class SelfCollisionConstraint : public StateConstraint {
 public:
  /**
   * Constructor
   *
   * @param [in] mapping: The pinocchio mapping from pinocchio states to ocs2 states.
   * @param [in] pinocchioGeometryInterface: Pinocchio geometry interface of the robot model.
   * @param [in] minimumDistance: The minimum allowed distance between collision pairs.
   */
  SelfCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping, PinocchioGeometryInterface pinocchioGeometryInterface,
                          scalar_t minimumDistance);

  /**
   * @brief Destructor.
   */
  ~SelfCollisionConstraint() override = default;

  /**
   * @brief Get Num Constraints.
   * @param [in] time: Time.
   * @return Result value.
   */
  size_t GetNumConstraints(scalar_t time) const final;

  /**
   * @brief Get the self collision distance values.
   * @note Requires pinocchio::forwardKinematics().
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] preComputation: Pre Computation.
   * @return Result value.
   */
  vector_t GetValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const final;

  /**
   * @brief Get the self collision distance approximation.
   *
   * pinocchio::updateGlobalPlacements(),
   *
   * pinocchio::computeJointJacobians().
   *
   * you should also call tham as well.
   * @note Requires pinocchio::forwardKinematics(),
   * @note In the cases that PinocchioStateInputMapping requires some additional update calls on PinocchioInterface,
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] preComputation: Pre Computation.
   * @return Result value.
   */
  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComputation) const final;

 protected:
  /**
   * @brief Get the pinocchio interface updated with the requested computation.
   * @param [in] preComputation: Pre Computation.
   * @return Result value.
   */
  virtual const PinocchioInterface& GetPinocchioInterface(const PreComputation& preComputation) const = 0;

  /**
   * @brief Constructs a SelfCollisionConstraint.
   * @param [in] rhs: Rhs.
   */
  SelfCollisionConstraint(const SelfCollisionConstraint& rhs);

  SelfCollision selfCollision_;
  std::unique_ptr<PinocchioStateInputMapping<scalar_t>> mappingPtr_;
};

}  // namespace bolza
