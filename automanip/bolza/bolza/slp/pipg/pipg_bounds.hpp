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
 * @file pipg_bounds.hpp
 * @brief Defines the lower and upper bounds of the total cost hessian, H, and the upper bound of \f$ G^TG \f$.
 */


#include <bolza/core/types.hpp>

namespace bolza {
namespace pipg {

/**
 * Defines the lower and upper bounds of the total cost hessian, H, and the upper bound of \f$ G^TG \f$.
 * Refer to "Proportional-Integral Projected Gradient Method for Model Predictive Control"
 * Link: https://arxiv.org/abs/2009.06980
 *
 * z := [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * min  0.5 z' H z + z' h
 * s.t. G z = g
 *
 * H = [ R0
 *       *   Q1  P1'
 *       *   P1  R1
 *       *   *   *   Qn  Pn'
 *       *   *   *   Pn  Rn
 *       *   *   *   *   *   Q{n+1}]
 *
 * G = [-B0  I
 *       *  -A1 -B1   I
 *
 *       *   *   *   -An -Bn  I
 *       D0  0
 *       *   C1  D1   0
 *
 *       *   *   *    Cn  Dn  0]
 */
struct PipgBounds {
  /**
   * @brief Constructs a PipgBounds.
   * @param [in] muArg: Mu Arg.
   * @param [in] lambdaArg: Lambda Arg.
   * @param [in] sigmaArg: Sigma Arg.
   */
  PipgBounds(scalar_t muArg, scalar_t lambdaArg, scalar_t sigmaArg) : mu(muArg), lambda(lambdaArg), sigma(sigmaArg) {}

  /**
   * @brief Dual Step Size.
   * @param [in] sigma: Sigma.
   * @return Result value.
   */
  scalar_t DualStepSize(size_t iteration) const { return (static_cast<scalar_t>(iteration) + 1.0) * mu / (2.0 * sigma); }

  /**
   * @brief Primal Step Size.
   * @param [in] lambda: Lambda.
   * @return Result value.
   */
  scalar_t PrimalStepSize(size_t iteration) const { return 2.0 / ((static_cast<scalar_t>(iteration) + 1.0) * mu + 2.0 * lambda); }

  scalar_t mu = 1e-5;     // mu I <= H
  scalar_t lambda = 1.0;  // H <= lambda I
  scalar_t sigma = 1.0;   // G' G <= sigma I
};

}  // namespace pipg
}  // namespace bolza
