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
 * @file linear_state_input_constraint.hpp
 * @brief Linear state-input constraint.
 */


#include <bolza/core/constraint/state_input_constraint.hpp>

namespace bolza {

/**
 * Linear state-input constraint
 */
class LinearStateInputConstraint : public StateInputConstraint {
 public:
  /**
   * Constructor
   *
   * @param [in] e: Constant term in C * x + D * u + e = 0
   * @param [in] C: x factor in C * x + D * u + e = 0
   * @param [in] D: u factor in C * x + D * u + e = 0
   */
  LinearStateInputConstraint(vector_t e, matrix_t C, matrix_t D);

  /**
   * @brief Destructor.
   */
  ~LinearStateInputConstraint() override = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  LinearStateInputConstraint* Clone() const override;

  /**
   * @brief Get Num Constraints.
   * @param [in] time: Time.
   * @return Result value.
   */
  size_t GetNumConstraints(scalar_t time) const final;

  /**
   * @brief Get Value.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return Result value.
   */
  vector_t GetValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& /* preComputation */) const final;

  /**
   * @brief Get Linear Approximation.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return Result value.
   */
  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                           const PreComputation& /* preComputation */) const final;

 public:
  vector_t e_; /**< State input constraint */
  matrix_t C_; /**< State input constraint derivative wrt. state */
  matrix_t D_; /**< State input constraint derivative wrt. input */
};

}  // namespace bolza
