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
 * @file qp_solver_types.hpp
 * @brief Defines the quadratic cost and  linear dynamics at a give stage.
 */


#include <bolza/core/types.hpp>

namespace bolza {
namespace qp_solver {

/** Defines the quadratic cost and  linear dynamics at a give stage */
struct LinearQuadraticStage {
  /** Quadratic approximation of the cost */
  ScalarFunctionQuadraticApproximation cost;
  /** Linear approximation of the dynamics */
  VectorFunctionLinearApproximation dynamics;
  /** Linear approximation of the constraints */
  VectorFunctionLinearApproximation constraints;

  /**
   * @brief Constructs a LinearQuadraticStage.
   */
  LinearQuadraticStage() = default;
  LinearQuadraticStage(ScalarFunctionQuadraticApproximation c, VectorFunctionLinearApproximation d, VectorFunctionLinearApproximation g)
      /**
       * @brief Cost.
       * @return Result value.
       */
      : Cost(std::move(c)), dynamics(std::move(d)), constraints(std::move(g)) {}
};

}  // namespace qp_solver
}  // namespace bolza
