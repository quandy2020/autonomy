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
 * @file state_soft_constraint.hpp
 * @brief Implements the cost penalty for state constraint terms.
 */


#include <memory>

#include <bolza/core/types.hpp>
#include <bolza/core/constraint/state_constraint.hpp>
#include <bolza/core/cost/state_cost.hpp>
#include <bolza/core/penalties/multidimensional_penalty.hpp>

namespace bolza {

/**
 *   Implements the cost penalty for state constraint terms
 *   \f$ h_i(x) \quad \forall  i \in [1,..,M] \f$
 *
 *   penalty(t, x) = \f$ \sum_{i=1}^{M} p(t, h_i(x)) \f$
 *
 *   The scalar penalty function \f$ p() \f$ and its derivatives are provided by the user.
 *   This class uses the chain rule to compute the second-order approximation of the constraint-penalty. In the case that the
 *   second-order approximation of constraint is not provided, it employs a Gauss-Newton approximation technique which only
 *   relies on the first-order approximation. In general, the penalty function can be a function of time.
 *
 *   A few commonly-used penalty functions have been provided by the toolbox such as Relaxed-Barrier and Squared-Hinge
 *   penalty functions.
 */
class StateSoftConstraint final : public StateCost {
 public:
  /**
   * Constructor.
   * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
   * @param [in] penaltyPtrArray: An array of pointers to the penalty function on the constraint.
   */
  StateSoftConstraint(std::unique_ptr<StateConstraint> constraintPtr, std::vector<std::unique_ptr<PenaltyBase>> penaltyPtrArray);

  /**
   * Constructor.
   * @note This allows a varying number of constraints and uses the same penalty function for each constraint.
   * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
   * @param [in] penaltyFunction: A pointer to the penalty function on the constraint.
   */
  StateSoftConstraint(std::unique_ptr<StateConstraint> constraintPtr, std::unique_ptr<PenaltyBase> penaltyFunction);

  /**
   * @brief Destructor.
   */
  ~StateSoftConstraint() override = default;

  /** Gets the wrapped constraint. */
  template <typename Derived = StateConstraint>
  /**
   * @brief Get.
   * @return Result value.
   */
  Derived& get() {
    static_assert(std::is_base_of<StateConstraint, Derived>::Value, "Template argument must derive from StateConstraint");
    return dynamic_cast<Derived&>(*constraintPtr_);
  }

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  StateSoftConstraint* Clone() const override;

  /**
   * @brief Is Active.
   * @param [in] time: Time.
   * @return True on success.
   */
  bool IsActive(scalar_t time) const override;

  /**
   * @brief Get Value.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t time, const vector_t& state, const TargetTrajectories& /* targetTrajectories */,
                    const PreComputation& preComp) const override;

  /**
   * @brief Get Quadratic Approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const TargetTrajectories& /* targetTrajectories */,
                                                                 const PreComputation& preComp) const override;

 private:
  /**
   * @brief Constructs a StateSoftConstraint.
   * @param [in] other: Other.
   */
  StateSoftConstraint(const StateSoftConstraint& other);

  std::unique_ptr<StateConstraint> constraintPtr_;
  MultidimensionalPenalty penalty_;
};

}  // namespace bolza
