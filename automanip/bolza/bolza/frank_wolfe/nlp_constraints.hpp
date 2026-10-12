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
 * @file nlp_constraints.hpp
 * @brief This class is an interface to a NLP constraints.
 */


#include <bolza/core/types.hpp>

namespace bolza {

/**
 * This class is an interface to a NLP constraints.
 */
class NLPConstraints {
 public:
  /**
   * Default constructor.
   */
  NLPConstraints() = default;

  /**
   * Default destructor.
   */
  virtual ~NLPConstraints() = default;

  /**
   * Sets the current parameter vector.
   *
   * @param [in] x: The value of parameter vector.
   */
  virtual void SetCurrentParameter(const vector_t& x) {}

  /**
   * Gets the linear equality constraints. \n
   * \f$ g_v = A_m x_v + b_v = 0\f$
   *
   * @param [out] g: The evaluation of the equality constraints, \f$ g_v \f$ vector.
   */
  virtual void GetLinearEqualityConstraint(vector_t& g) { g.resize(0); }

  /**
   * Gets the derivative of the linear equality constraints. \n
   * \f$ g_v = A_m x_v + b_v = 0\f$
   *
   * @param [out] dgdx: The Jacobian of the equality constraints, \f$ A_m \f$ vector.
   */
  virtual void GetLinearEqualityConstraintDerivative(matrix_t& dgdx) { dgdx.resize(0, 0); }

  /**
   * Gets the linear inequality constraints. \n
   * \f$ h_v = C_m x_v + d_v \geq 0\f$
   *
   * @param [out] h: The evaluation of the inequality constraints, \f$ h_v \f$ vector.
   */
  virtual void GetLinearInequalityConstraint(vector_t& h) { h.resize(0); }

  /**
   * Gets the derivative of the linear inequality constraints. \n
   * \f$ h_v = C_m x_v + d_v \geq 0\f$
   *
   * @param [out] dhdx: The Jacobian of the inequality constraints, \f$ C_m \f$ vector.
   */
  virtual void GetLinearInequalityConstraintDerivative(matrix_t& dhdx) { dhdx.resize(0, 0); }
};

}  // namespace bolza
