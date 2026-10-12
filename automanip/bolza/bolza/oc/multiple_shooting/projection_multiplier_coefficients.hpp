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
 * @file projection_multiplier_coefficients.hpp
 * @brief Coefficients to compute the Newton step of the Lagrange multiplier associated with the state-input equality constraint such that.
 */


#include <bolza/core/types.hpp>

namespace bolza {
namespace multiple_shooting {

/**
 * Coefficients to compute the Newton step of the Lagrange multiplier associated with the state-input equality constraint such that
 * dfdx * dx + dfdu * du + dfdcostate * dcostate + f
 */
struct ProjectionMultiplierCoefficients {
  matrix_t dfdx;
  matrix_t dfdu;
  matrix_t dfdcostate;
  vector_t f;

  /**
   * Computes the coefficients of the Lagrange multiplier associated with the state-input equality constraint.
   *
   * @param [in] cost: The cost quadratic approximation.
   * @param [in] dynamics: The dynamics linear approximation.
   * @param [in] constraintProjection: The constraint projection.
   * @param [in] pseudoInverse: Left pseudo-inverse of D^T of the state-input linearized equality Constraint (C dx + D du + e = 0).
   */
  void compute(const ScalarFunctionQuadraticApproximation& cost, const VectorFunctionLinearApproximation& dynamics,
               const VectorFunctionLinearApproximation& constraintProjection, const matrix_t& pseudoInverse);
};

}  // namespace multiple_shooting
}  // namespace bolza
