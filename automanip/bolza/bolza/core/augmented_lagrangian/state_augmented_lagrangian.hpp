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
 * @file state_augmented_lagrangian.hpp
 * @brief The base class for Augmented Lagrangian penalty of state constraint.
 */


#include <memory>

#include <bolza/core/augmented_lagrangian/state_augmented_lagrangian_interface.hpp>
#include <bolza/core/penalties/multidimensional_penalty.hpp>

namespace bolza {

/** The base class for Augmented Lagrangian penalty of state constraint. */
class StateAugmentedLagrangian final : public StateAugmentedLagrangianInterface {
 public:
  /**
   * Constructor.
   * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
   * @param [in] penaltyPtrArray: An array of pointers to the penalty function on the constraint.
   */
  StateAugmentedLagrangian(std::unique_ptr<StateConstraint> constraintPtr,
                           std::vector<std::unique_ptr<augmented::AugmentedPenaltyBase>> penaltyPtrArray);

  /**
   * Constructor.
   * @note This allows a varying number of constraints and uses the same penalty function for each constraint.
   * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
   * @param [in] penaltyPtr: A pointer to the penalty function on the constraint.
   */
  StateAugmentedLagrangian(std::unique_ptr<StateConstraint> constraintPtr, std::unique_ptr<augmented::AugmentedPenaltyBase> penaltyPtr);

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  StateAugmentedLagrangian* Clone() const override;
  /**
   * @brief Is Active.
   * @param [in] time: Time.
   * @return True on success.
   */
  bool IsActive(scalar_t time) const override;
  /**
   * @brief Get Num Constraints.
   * @param [in] time: Time.
   * @return Result value.
   */
  size_t GetNumConstraints(scalar_t time) const override;

  /**
   * @brief Get Value.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] multiplier: Multiplier.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  LagrangianMetrics GetValue(scalar_t time, const vector_t& state, const Multiplier& multiplier,
                             const PreComputation& preComp) const override;

  /**
   * @brief Get Quadratic Approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] multiplier: Multiplier.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state, const Multiplier& multiplier,
                                                                 const PreComputation& preComp) const override;

  /**
   * @brief Update Lagrangian.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] constraint: Constraint.
   * @param [in] multiplier: Multiplier.
   * @return Result value.
   */
  std::pair<Multiplier, scalar_t> UpdateLagrangian(scalar_t time, const vector_t& state, const vector_t& constraint,
                                                   const Multiplier& multiplier) const override;

  /**
   * @brief Initialize Lagrangian.
   * @param [in] time: Time.
   * @return Result value.
   */
  Multiplier InitializeLagrangian(scalar_t time) const override;

  /** Gets the wrapped constraint. */
  template <typename Derived = StateConstraint>
  /**
   * @brief Get.
   * @return Result value.
   */
  Derived& get() {
    static_assert(std::is_base_of<StateConstraint, Derived>::Value, "Template argument must derive from StateConstraint!");
    return dynamic_cast<Derived&>(*constraintPtr_);
  }

 private:
  /**
   * @brief Constructs a StateAugmentedLagrangian.
   * @param [in] other: Other.
   */
  StateAugmentedLagrangian(const StateAugmentedLagrangian& other);

  std::unique_ptr<StateConstraint> constraintPtr_;
  MultidimensionalPenalty penalty_;
};

}  // namespace bolza
