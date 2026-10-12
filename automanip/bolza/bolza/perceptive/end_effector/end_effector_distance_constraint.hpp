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
 * @file end_effector_distance_constraint.hpp
 * @brief End-effector distance constraint Function.
 */


#include <memory>
#include <string>
#include <utility>

#include <bolza/core/types.hpp>
#include <bolza/core/constraint/state_constraint.hpp>
#include <bolza/tools/end_effector/end_effector_kinematics.hpp>

#include "bolza/perceptive/distance_transform/distance_transform_interface.hpp"

namespace bolza {

/**
 * End-effector distance constraint Function.
 */
class EndEffectorDistanceConstraint final : public bolza::StateConstraint {
 public:
  /**
   * @brief Constructor.
   * @param [in] stateDim: State Dim.
   * @param [in] weight: Weight.
   * @param [in] kinematicsPtr: Kinematics Ptr.
   */
  EndEffectorDistanceConstraint(size_t stateDim, scalar_t weight, std::unique_ptr<EndEffectorKinematics<scalar_t>> kinematicsPtr);

  /**
   * @brief Constructor.
   * @param [in] stateDim: State Dim.
   * @param [in] weight: Weight.
   * @param [in] kinematicsPtr: Kinematics Ptr.
   * @param [in] clearances: Clearances.
   */
  EndEffectorDistanceConstraint(size_t stateDim, scalar_t weight, std::unique_ptr<EndEffectorKinematics<scalar_t>> kinematicsPtr,
                                scalar_array_t clearances);

  /** Default destructor */
  ~EndEffectorDistanceConstraint() override = default;

  /**
   * @brief Set.
   * @param [in] distanceTransform: Distance Transform.
   */
  void set(const DistanceTransformInterface& distanceTransform);
  /**
   * @brief Set.
   * @param [in] clearance: Clearance.
   * @param [in] distanceTransform: Distance Transform.
   */
  void set(scalar_t clearance, const DistanceTransformInterface& distanceTransform);
  /**
   * @brief Set.
   * @param [in] clearances: Clearances.
   * @param [in] distanceTransform: Distance Transform.
   */
  void set(const scalar_array_t& clearances, const DistanceTransformInterface& distanceTransform);

  /**
   * @brief Clone.
   * @param [in] this: This.
   * @return Pointer; may be null.
   */
  EndEffectorDistanceConstraint* Clone() const override { return new EndEffectorDistanceConstraint(*this); }
  /**
   * @brief Get Num Constraints.
   * @return Result value.
   */
  size_t GetNumConstraints(scalar_t time) const override { return kinematicsPtr_->getIds().size(); }
  /**
   * @brief Get IDs.
   * @return Result value.
   */
  const std::vector<std::string>& GetIDs() const { return kinematicsPtr_->getIds(); }

  /**
   * @brief Get Value.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  vector_t GetValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
  /**
   * @brief Get Linear Approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComp) const override;

  /**
   * @brief Get State Dimensions.
   * @return Result value.
   */
  size_t GetStateDimensions() const { return stateDim_; }
  /**
   * @brief Get Kinematics.
   * @return Result value.
   */
  EndEffectorKinematics<scalar_t>& GetKinematics() { return *kinematicsPtr_; }

 private:
  /**
   * @brief Constructs a EndEffectorDistanceConstraint.
   * @param [in] other: Other.
   */
  EndEffectorDistanceConstraint(const EndEffectorDistanceConstraint& other);

  const size_t stateDim_;
  const scalar_t weight_;

  std::unique_ptr<EndEffectorKinematics<scalar_t>> kinematicsPtr_;

  scalar_array_t clearances_;
  const DistanceTransformInterface* distanceTransformPtr_ = nullptr;
};

}  // namespace bolza
