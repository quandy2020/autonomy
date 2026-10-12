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
 * @file state_input_soft_box_constraint.hpp
 * @brief Implements the cost penalty for state-input box constraints.
 */


#include <memory>

#include <bolza/core/types.hpp>
#include <bolza/core/cost/state_input_cost.hpp>
#include <bolza/core/penalties/penalties/penalty_base.hpp>

namespace bolza {

/**
 *   Implements the cost penalty for state-input box constraints
 */
class StateInputSoftBoxConstraint final : public StateInputCost {
 public:
  struct BoxConstraint {
    //! Index of the constraint in the state or input vector
    size_t index = 0;

    //! Lower bound of the box Constraint (default is low, but not numeric::lowest to prevent underflow)
    scalar_t lowerBound = -1e30;

    //! Upper bound of the box Constraint (default is high, but not numeric::max to prevent overflow)
    scalar_t upperBound = 1e30;

    //! Penalty function
    std::unique_ptr<PenaltyBase> penaltyPtr;

    /* Constructors and assignment operators */
    /**
     * @brief Constructs a BoxConstraint.
     */
    BoxConstraint() = default;
    /**
     * @brief Destructor.
     */
    ~BoxConstraint() = default;
    /**
     * @brief Constructs a BoxConstraint.
     * @param [in] other: Other.
     */
    BoxConstraint(const BoxConstraint& other);
    BoxConstraint& operator=(const BoxConstraint& other);
    /**
     * @brief Constructs a BoxConstraint.
     * @param [in,out] other: Other.
     */
    BoxConstraint(BoxConstraint&& other) noexcept = default;
    BoxConstraint& operator=(BoxConstraint&& other) noexcept = default;
  };

  /**
   * Constructor.
   * @param [in] stateBoxConstraints: box constraint specification for states
   * @param [in] inputBoxConstraints: box constraint specification for inputs
   */
  StateInputSoftBoxConstraint(std::vector<BoxConstraint> stateBoxConstraints, std::vector<BoxConstraint> inputBoxConstraints);

  /**
   * @brief Destructor.
   */
  ~StateInputSoftBoxConstraint() override = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  StateInputSoftBoxConstraint* Clone() const override;

  /**
   * @brief Is Active.
   * @param [in] time: Time.
   * @return True on success.
   */
  bool IsActive(scalar_t time) const override;

    /**
   * @brief.
   *
   * Takes the cost value at the given time, state, input and adds it as constant offset to further evaluations.
   *
   * Taking a penalty on constraint bounds that are far away can create a large (negative) value inside the feasible set, for example when
   *
   * using a relaxed barrier constraint. Adding a constant offset does not change the optimal solution, but gives peace of mind that the
   *
   * cost values are in a reasonable absolute range.
   *
   * /
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   */
  void InitializeOffset(scalar_t time, const vector_t& state, const vector_t& input);

  /**
   * @brief Get Value.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& /* targetTrajectories */,
                    const PreComputation& preComp) const override;

  /**
   * @brief Get Quadratic Approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const TargetTrajectories& /* targetTrajectories */,
                                                                 const PreComputation& preComp) const override;

 private:
  /**
   * @brief Constructs a StateInputSoftBoxConstraint.
   * @param [in] other: Other.
   */
  StateInputSoftBoxConstraint(const StateInputSoftBoxConstraint& other) = default;

  /**
   * @brief Sort By Index.
   * @param [in,out] boxConstraints: Box Constraints.
   */
  void SortByIndex(std::vector<BoxConstraint>& boxConstraints) const;

  /**
   * @brief Get Value.
   * @param [in] t: T.
   * @param [in] h: H.
   * @param [in] boxConstraints: Box Constraints.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t t, const vector_t& h, const std::vector<BoxConstraint>& boxConstraints) const;

  /**
   * @brief Fill Quadratic Approximation.
   * @param [in] t: T.
   * @param [in] h: H.
   * @param [in] boxConstraints: Box Constraints.
   * @param [in,out] value: Value.
   * @param [in,out] firstDerivative: First Derivative.
   * @param [in,out] secondDerivative: Second Derivative.
   */
  void FillQuadraticApproximation(scalar_t t, const vector_t& h, const std::vector<BoxConstraint>& boxConstraints, scalar_t& value,
                                  vector_t& firstDerivative, matrix_t& secondDerivative) const;

  std::vector<BoxConstraint> stateBoxConstraints_;
  std::vector<BoxConstraint> inputBoxConstraints_;
  scalar_t offset_;
};

}  // namespace bolza
