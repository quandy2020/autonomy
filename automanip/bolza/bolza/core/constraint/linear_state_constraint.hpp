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
 * @file linear_state_constraint.hpp
 * @brief Linear state-only constraint.
 */


#include <bolza/core/constraint/state_constraint.hpp>

namespace bolza {

/**
 * Linear state-only constraint
 */
class LinearStateConstraint : public StateConstraint {
 public:
  /**
   * Constructor
   *
   * @param [in] h: Constant term in F * x + h = 0
   * @param [in] F: x factor in F * x + h = 0
   */
  LinearStateConstraint(vector_t h, matrix_t F);

  /**
   * @brief Destructor.
   */
  ~LinearStateConstraint() override = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  LinearStateConstraint* Clone() const override;

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
   * @return Result value.
   */
  vector_t GetValue(scalar_t t, const vector_t& x, const PreComputation& /* preComputation */) const final;

  /**
   * @brief Get Linear Approximation.
   * @param [in] t: T.
   * @param [in] x: X.
   * @return Result value.
   */
  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t t, const vector_t& x,
                                                           const PreComputation& /* preComputation */) const final;

 public:
  vector_t h_; /**< State only constraint */
  matrix_t F_; /**< State only constraint derivative wrt. state */
};

}  // namespace bolza
